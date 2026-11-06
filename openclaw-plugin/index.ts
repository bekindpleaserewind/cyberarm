/**
 * OpenClaw plugin for the CyberBrick robotic arm.
 *
 * Every tool runs the PC driver (src/arm_serial.py) with --json, which talks
 * to the arm's receiver board over USB serial and prints one JSON result.
 * Commands are queued so only one uses the serial port at a time.
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

type DriverResult = {
  ok: boolean;
  command: string;
  before: ArmStatus | null;
  status: ArmStatus | null;
  output: string;
  error: string | null;
};

type Settings = {
  python: string;
  driverPath: string;
  port?: string;
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
  return { python, driverPath, port: str("port"), timeoutMs: Math.round(timeout * 1000) };
}

// One serial port, one command at a time.
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
    `Lower arm ${status.lower_arm}°, upper arm ${status.upper_arm}°.`;
  if (status.timer_error) text += ` WARNING: board timer error ${status.timer_error}.`;
  return text;
}

async function armCommand(settings: Settings, args: string[], summary?: (r: DriverResult) => string) {
  const result = await runDriver(settings, args);
  if (!result.ok || !result.status) {
    throw new Error(`Arm command '${args.join(" ")}' failed: ${result.error ?? "unknown error"}`);
  }
  const lead = summary ? summary(result) + " " : "";
  return {
    content: [{ type: "text" as const, text: lead + describe(result.status) }],
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
    Type.Number({ minimum: JOINT_MIN, maximum: JOINT_MAX, description: "Lower arm angle, 0-180." }),
  ),
  upper: Type.Optional(
    Type.Number({ minimum: JOINT_MIN, maximum: JOINT_MAX, description: "Upper arm angle, 0-180." }),
  ),
  speed: Type.Optional(
    Type.Number({
      minimum: 1,
      maximum: 100,
      description: "1 (slow) to 100 (instant). Default 20, about 90°/s.",
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
        "lower/upper arm joint angles (0-180, 90 = home) and claw opening (0 closed .. 1 open). " +
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
        "Move the arm: lower arm joint (S1) and/or upper arm joint (S2) to absolute angles 0-180 " +
        "(90 = home pose). Give one or both. Both move together and the tool returns when they arrive.",
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
        return armCommand(settings(), args);
      },
    });

    api.registerTool({
      name: "cyberbrick_arm_home",
      label: "Arm home",
      description: "Return the arm to its home pose: both joints to 90° and the base to centre. The claw is unchanged.",
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
