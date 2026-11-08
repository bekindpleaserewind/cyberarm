/**
 * OpenClaw plugin for the CyberBrick robotic arm.
 *
 * Every tool runs the PC driver (src/arm_serial.py) with --json, which sends
 * the command to the arm and prints one JSON result. Once radio mode is set up
 * it goes PC --USB--> CyberBrick remote --radio--> arm; otherwise over the
 * arm's own USB. Commands are queued so only one uses the link at a time.
 *
 * Safety: the base never turns more than 90 degrees left or right of centre.
 * The tools refuse larger angles, and the board library enforces the same
 * limit itself (BASE_LIMIT_DEG in cyberbrick_arm.py).
 */
import { execFile } from "node:child_process";
import { existsSync } from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";
import { Type, type Static, type TSchema } from "typebox";
import { definePluginEntry } from "openclaw/plugin-sdk/plugin-entry";

const PLUGIN_ID = "cyberbrick-arm";
const BASE_LIMIT = 90; // degrees either side of centre
const JOINT_MIN = 0;
const JOINT_MAX = 180;

type ArmStatus = {
  lower_arm: number;
  upper_arm: number;
  claw: number; // 0 closed .. 1 open
  base_angle: number; // negative left, positive right
  base_speed: number;
  timer_error?: string;
};

// Joint calibration from `arm_serial.py arm-calibrate` (LOWER_ARM_MIN,
// LOWER_ARM_DIRECTION, ...). The driver adds it to the status command's result.
type Calibration = Record<string, number>;

type DriverResult = {
  ok: boolean;
  command: string;
  before: ArmStatus | null;
  status: ArmStatus | null;
  output: string;
  error: string | null;
  calibration?: Calibration;
};

type Settings = {
  python: string;
  driverPath: string;
  port?: string;
  transport?: "auto" | "radio" | "usb";
  timeoutMs: number;
};

function pluginDir(rootDir?: string): string {
  if (rootDir) return rootDir;
  try {
    return path.dirname(fileURLToPath(import.meta.url));
  } catch {
    return process.cwd();
  }
}

function resolveSettings(config: Record<string, unknown>, rootDir?: string): Settings {
  const str = (key: string) =>
    typeof config[key] === "string" && (config[key] as string).trim()
      ? (config[key] as string).trim()
      : undefined;

  const driverPath = path.resolve(
    str("driverPath") ?? path.join(pluginDir(rootDir), "..", "src", "arm_serial.py"),
  );
  const driverDir = path.dirname(driverPath);
  const venvPython =
    process.platform === "win32"
      ? path.join(driverDir, ".venv", "Scripts", "python.exe")
      : path.join(driverDir, ".venv", "bin", "python");
  const python =
    str("python") ??
    (existsSync(venvPython) ? venvPython : process.platform === "win32" ? "python" : "python3");

  const timeout = typeof config.timeoutSeconds === "number" ? config.timeoutSeconds : 90;
  const transport = str("transport");
  return {
    python,
    driverPath,
    port: str("port"),
    transport: transport === "radio" || transport === "usb" || transport === "auto" ? transport : undefined,
    timeoutMs: Math.round(timeout * 1000),
  };
}

// One link to the arm, one command at a time.
let queue: Promise<unknown> = Promise.resolve();

function runDriver(settings: Settings, args: string[]): Promise<DriverResult> {
  const job = queue.then(
    () =>
      new Promise<DriverResult>((resolve, reject) => {
        if (!existsSync(settings.driverPath)) {
          reject(
            new Error(
              `Arm driver not found at ${settings.driverPath}. ` +
                `Set plugins.entries.${PLUGIN_ID}.config.driverPath to arm_serial.py.`,
            ),
          );
          return;
        }
        const argv = [settings.driverPath, "--json"];
        if (settings.port) argv.push("--port", settings.port);
        if (settings.transport) argv.push("--transport", settings.transport);
        argv.push(...args);
        execFile(
          settings.python,
          argv,
          {
            cwd: path.dirname(settings.driverPath),
            timeout: settings.timeoutMs,
            windowsHide: true,
            maxBuffer: 4 * 1024 * 1024,
          },
          (err, stdout, stderr) => {
            const lastLine = stdout.trim().split(/\r?\n/).pop() ?? "";
            try {
              resolve(JSON.parse(lastLine) as DriverResult);
              return;
            } catch {
              // fall through: no JSON result
            }
            const detail = (stderr || stdout || String(err ?? "")).trim().split(/\r?\n/).slice(-8);
            if (err && (err as NodeJS.ErrnoException).code === "ENOENT") {
              reject(new Error(`Python not found (${settings.python}). Set the plugin's "python" config.`));
            } else if (err && (err as { killed?: boolean }).killed) {
              reject(new Error(`Arm command timed out after ${settings.timeoutMs / 1000}s.`));
            } else {
              reject(new Error(`Arm driver failed:\n${detail.join("\n")}`));
            }
          },
        );
      }),
  );
  queue = job.catch(() => undefined);
  return job;
}

function describeBase(angle: number): string {
  if (Math.abs(angle) < 3) return "centre";
  return `${Math.abs(Math.round(angle))}° ${angle < 0 ? "left" : "right"}`;
}

function describe(status: ArmStatus): string {
  const claw = Math.round(status.claw * 100);
  const clawText = claw <= 2 ? "closed" : claw >= 98 ? "fully open" : `${claw}% open`;
  let text =
    `Base faces ${describeBase(status.base_angle)} (base_angle ${Math.round(status.base_angle)}, ` +
    `limit ±${BASE_LIMIT}). Claw ${clawText} (${status.claw}). ` +
    `Lower arm at position ${status.lower_arm}, upper arm at ${status.upper_arm} (0 = home, 180 = fully extended).`;
  if (status.timer_error) text += ` WARNING: board timer error ${status.timer_error}.`;
  return text;
}

function describeJoints(cal: Calibration): string {
  const joint = (key: string, name: string, plus: string, minus: string) => {
    const lo = cal[`${key}_MIN`];
    const hi = cal[`${key}_MAX`];
    const home = cal[`${key}_HOME`];
    if (home !== lo && home !== hi) {
      return (
        `${name}: not calibrated yet, so its position is the raw servo angle ${lo}-${hi} ` +
        `(home ${home}) and which way it moves is unknown`
      );
    }
    const d = cal[`${key}_DIRECTION`];
    const way = d === 1 ? plus : d === -1 ? minus : "moves it an unknown way";
    return `${name}: position 0 = home (folded), 180 = fully extended; extending ${way}`;
  };
  return (
    joint("LOWER_ARM", "Lower arm", "leans it forward", "leans it back") +
    ". " +
    joint("UPPER_ARM", "Upper arm", "lifts the claw", "lowers the claw") +
    "."
  );
}

async function armCommand(settings: Settings, args: string[], summary?: (r: DriverResult) => string) {
  const result = await runDriver(settings, args);
  if (!result.ok || !result.status) {
    throw new Error(`Arm command '${args.join(" ")}' failed: ${result.error ?? "unknown error"}`);
  }
  const parts = [summary ? summary(result) : "", describe(result.status)];
  if (result.calibration) parts.push(describeJoints(result.calibration));
  return {
    content: [{ type: "text" as const, text: parts.filter(Boolean).join(" ") }],
    details: result,
  };
}

const noParams = Type.Object({});

const rotateParams = Type.Object({
  direction: Type.Unsafe<"left" | "right">({
    type: "string",
    enum: ["left", "right"],
    description: "Which way to turn, as seen from behind the arm.",
  }),
  degrees: Type.Number({
    minimum: 1,
    maximum: BASE_LIMIT,
    description: "How far to turn, 1-90 degrees.",
  }),
});

const faceParams = Type.Object({
  angle: Type.Number({
    minimum: -BASE_LIMIT,
    maximum: BASE_LIMIT,
    description: "Target angle in degrees, -90 (full left) to 90 (full right). 0 = centre.",
  }),
});

const clawParams = Type.Object({
  position: Type.Number({ minimum: 0, maximum: 1, description: "Opening from 0 (closed) to 1 (open)." }),
});

const moveParams = Type.Object({
  lower: Type.Optional(
    Type.Number({
      minimum: JOINT_MIN,
      maximum: JOINT_MAX,
      description: "Lower arm position: 0 = home (folded) .. 180 = fully extended.",
    }),
  ),
  upper: Type.Optional(
    Type.Number({
      minimum: JOINT_MIN,
      maximum: JOINT_MAX,
      description: "Upper arm position: 0 = home (folded) .. 180 = fully extended.",
    }),
  ),
  speed: Type.Optional(
    Type.Number({
      minimum: 1,
      maximum: 100,
      description: "1 (slow) to 100 (instant). Default 20, about 90°/s.",
    }),
  ),
});

// Default swings per gesture (GESTURES in arm_serial.py).
const GESTURE_TIMES: Record<string, number> = { wave: 3, nod: 2, shake: 2 };

const gestureParams = Type.Object({
  gesture: Type.Unsafe<"wave" | "nod" | "shake">({
    type: "string",
    enum: ["wave", "nod", "shake"],
    description: "wave = hello/goodbye, nod = yes, shake = no.",
  }),
  times: Type.Optional(
    Type.Integer({ minimum: 1, maximum: 10, description: "Swings. Default: wave 3, nod 2, shake 2." }),
  ),
  speed: Type.Optional(
    Type.Number({
      minimum: 1,
      maximum: 100,
      description: "Joint speed 1-100. Default 25 (smooth). Not used by shake.",
    }),
  ),
});

// The host hands parameters over untyped; type them from their schema.
const typed = <T extends TSchema>(_schema: T, raw: unknown) => (raw ?? {}) as Partial<Static<T>>;

export default definePluginEntry({
  id: PLUGIN_ID,
  name: "CyberBrick Arm",
  description:
    "Control a CyberBrick robotic arm: turn the base up to 90° left or right, move the arm joints, open and close the claw.",
  register(api) {
    const settings = () => resolveSettings(api.pluginConfig ?? {}, api.rootDir);

    api.registerTool({
      name: "cyberbrick_arm_status",
      label: "Arm status",
      description:
        "Read the CyberBrick arm's current state: base angle (-90 left .. 0 centre .. 90 right), " +
        "lower/upper arm joint positions (0 = home/folded .. 180 = fully extended) and claw opening " +
        "(0 closed .. 1 open), plus which way extending each joint moves it (from arm calibration). " +
        "Nothing moves. Base angle and claw are estimates from timing.",
      parameters: noParams,
      async execute() {
        return armCommand(settings(), ["status"]);
      },
    });

    api.registerTool({
      name: "cyberbrick_arm_rotate_base",
      label: "Rotate arm base",
      description:
        "Turn (spin) the arm's base left or right by a number of degrees from where it is now. " +
        "The base can never go more than 90° left or right of centre; a turn that would pass " +
        "the limit stops at the limit. Use cyberbrick_arm_face_angle to face an exact direction.",
      parameters: rotateParams,
      async execute(_id, raw) {
        const params = typed(rotateParams, raw);
        const degrees = Number(params.degrees);
        if (!Number.isFinite(degrees) || degrees <= 0 || degrees > BASE_LIMIT) {
          throw new Error(`degrees must be between 1 and ${BASE_LIMIT}`);
        }
        if (params.direction !== "left" && params.direction !== "right") {
          throw new Error('direction must be "left" or "right"');
        }
        const delta = params.direction === "left" ? -degrees : degrees;
        return armCommand(settings(), ["turn-by", String(delta)], (r) => {
          const start = r.before?.base_angle ?? 0;
          const wanted = start + delta;
          return Math.abs(wanted) > BASE_LIMIT
            ? `Stopped at the ${BASE_LIMIT}° ${params.direction} limit (asked for ${Math.round(Math.abs(wanted))}°).`
            : `Turned ${degrees}° ${params.direction}.`;
        });
      },
    });

    api.registerTool({
      name: "cyberbrick_arm_face_angle",
      label: "Point arm base",
      description:
        "Turn the arm's base to face an exact angle from centre: negative = left, 0 = straight ahead, " +
        "positive = right. Allowed range -90 to 90.",
      parameters: faceParams,
      async execute(_id, raw) {
        const params = typed(faceParams, raw);
        const angle = Number(params.angle);
        if (!Number.isFinite(angle) || Math.abs(angle) > BASE_LIMIT) {
          throw new Error(`angle must be between -${BASE_LIMIT} and ${BASE_LIMIT}`);
        }
        return armCommand(settings(), ["turn", String(angle)]);
      },
    });

    api.registerTool({
      name: "cyberbrick_arm_open_claw",
      label: "Open claw",
      description: "Open the arm's claw fully (to release or get ready to grab something).",
      parameters: noParams,
      async execute() {
        return armCommand(settings(), ["claw", "open"]);
      },
    });

    api.registerTool({
      name: "cyberbrick_arm_close_claw",
      label: "Close claw",
      description: "Close the arm's claw fully (to grab something or to tuck it away).",
      parameters: noParams,
      async execute() {
        return armCommand(settings(), ["claw", "close"]);
      },
    });

    api.registerTool({
      name: "cyberbrick_arm_set_claw",
      label: "Set claw opening",
      description:
        "Open the claw part way: 0 = closed, 0.5 = half open, 1 = fully open. " +
        "Use for a gentle grip on small objects.",
      parameters: clawParams,
      async execute(_id, raw) {
        const params = typed(clawParams, raw);
        const position = Number(params.position);
        if (!Number.isFinite(position) || position < 0 || position > 1) {
          throw new Error("position must be between 0 and 1");
        }
        return armCommand(settings(), ["claw", String(position)]);
      },
    });

    api.registerTool({
      name: "cyberbrick_arm_move",
      label: "Move arm joints",
      description:
        "Move the arm: lower arm joint (S1) and/or upper arm joint (S2) to a position from 0 " +
        "(home, folded) to 180 (fully extended); 90 is half way. Give one or both. Both move " +
        "together and the tool returns when they arrive.",
      parameters: moveParams,
      async execute(_id, raw) {
        const params = typed(moveParams, raw);
        const angle = (v: unknown, name: string) => {
          if (v === undefined || v === null) return undefined;
          const n = Number(v);
          if (!Number.isFinite(n) || n < JOINT_MIN || n > JOINT_MAX) {
            throw new Error(`${name} must be between ${JOINT_MIN} and ${JOINT_MAX}`);
          }
          return n;
        };
        const lower = angle(params.lower, "lower");
        const upper = angle(params.upper, "upper");
        if (lower === undefined && upper === undefined) {
          throw new Error("give lower, upper, or both");
        }
        let args: string[];
        if (lower !== undefined && upper !== undefined) args = ["move", String(lower), String(upper)];
        else if (lower !== undefined) args = ["lower", String(lower)];
        else args = ["upper", String(upper)];
        if (params.speed !== undefined) {
          const speed = Number(params.speed);
          if (!Number.isFinite(speed) || speed < 1 || speed > 100) {
            throw new Error("speed must be between 1 and 100");
          }
          args.push(String(speed));
        }
        return armCommand(settings(), args, (r) => {
          const notes: string[] = [];
          const check = (asked: number | undefined, got: number | undefined, name: string) => {
            if (asked !== undefined && got !== undefined && Math.abs(got - asked) > 1) {
              notes.push(`the ${name} stopped at ${got} (its limit), not ${asked}`);
            }
          };
          check(lower, r.status?.lower_arm, "lower arm");
          check(upper, r.status?.upper_arm, "upper arm");
          return notes.length ? `Note: ${notes.join("; ")}.` : "";
        });
      },
    });

    api.registerTool({
      name: "cyberbrick_arm_gesture",
      label: "Arm gesture",
      description:
        "Make a gesture in one smooth motion: wave (swing the upper arm), nod (yes, with the " +
        "lower arm) or shake (no, turning the base). The arm raises itself first if needed, " +
        "swings at one even speed, and returns to the pose it started in. Use this rather " +
        "than a series of cyberbrick_arm_move calls, which jerk and pause between swings.",
      parameters: gestureParams,
      async execute(_id, raw) {
        const params = typed(gestureParams, raw);
        const name = String(params.gesture ?? "");
        if (!["wave", "nod", "shake"].includes(name)) {
          throw new Error("gesture must be wave, nod or shake");
        }
        const args = ["gesture", name];
        const times = params.times === undefined ? undefined : Number(params.times);
        const speed = params.speed === undefined ? undefined : Number(params.speed);
        if (times !== undefined && !(Number.isInteger(times) && times >= 1 && times <= 10)) {
          throw new Error("times must be a whole number from 1 to 10");
        }
        if (speed !== undefined && !(Number.isFinite(speed) && speed >= 1 && speed <= 100)) {
          throw new Error("speed must be between 1 and 100");
        }
        if (times !== undefined || speed !== undefined) args.push(String(times ?? GESTURE_TIMES[name]));
        if (speed !== undefined) args.push(String(speed));
        return armCommand(settings(), args, () => `Done: ${name}.`);
      },
    });

    api.registerTool({
      name: "cyberbrick_arm_home",
      label: "Arm home",
      description:
        "Return the arm to its home pose: both joints to position 0 (home, folded) and the base " +
        "to centre. The claw is unchanged.",
      parameters: noParams,
      async execute() {
        return armCommand(settings(), ["home"]);
      },
    });

    api.registerTool({
      name: "cyberbrick_arm_stop",
      label: "Stop arm",
      description: "Stop the base and claw immediately and hold the joints where they are.",
      parameters: noParams,
      async execute() {
        return armCommand(settings(), ["stop"]);
      },
    });
  },
});
