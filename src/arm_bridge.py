# -*- coding: utf-8 -*-
"""
Radio bridge for the CyberBrick arm (MicroPython, runs on the CyberBrick
remote, i.e. the transmitter).

arm_serial.py talks to the remote over USB and, for each arm command, runs
`print(_b.call(...))` here. The bridge sends the request to the arm over
ESP-NOW and prints the arm's final reply. See arm_radio.py for the messages.
"""

import json
import os
import time

NO_CONTACT_MS = 5000    # not one send acked this long: "no radio contact"
NO_REPLY_MS = 10000     # acked but no answer this long: "no reply from the
                        # arm". Rides out short losses, the arm still
                        # starting up, and the arm's recovery steps
                        # (arm_radio.Server.recover)
REKEY_AFTER_MS = 2000   # acked but no answer this long: add the arm as a peer
                        # again (once per request); it may have restarted
RXBUF = 2048            # ESP-NOW receive buffer: 7 full-size messages
MAX_MSG = 250           # ESP-NOW payload limit (espnow.MAX_DATA_LEN)
WIFI_MARK = "/arm_wifi_try"    # see arm_radio.WIFI_MARK
# Heartbeat: a tiny message to the arm every HEARTBEAT_MS for as long as the
# bridge exists, like the stock remote's constant stream (2026-11-22: testing
# whether constant traffic stops the arm's radio going deaf). The arm counts
# it as ignored (not a request: it does not start with "["). 0: off.
HEARTBEAT_MS = 1000      # once a second (2026-11-22; 100 ms did not stop the
                         # deaf spells): keeps a timeline of them, little traffic
HEARTBEAT_TIMER = 0
HB_GAP = 2              # this many heartbeats unacked in a row: log the gap


def _exists(path):
    try:
        os.stat(path)
        return True
    except OSError:
        return False


def wifi_reset(wlan, channel, full, pm=True):
    """Copy of arm_radio.wifi_reset (each board file is installed on its own):
    station on, not connected, on `channel`, power saving off; with full, the
    station is switched off and on first. The access point is left alone:
    switching it off stops Wi-Fi altogether on this firmware. Raises OSError
    if Wi-Fi is not on and on `channel` afterwards."""
    if full and wlan.active():
        wlan.active(False)
    if not wlan.active():
        wlan.active(True)
    try:
        wlan.config(reconnects=0)    # never try to rejoin a saved network
    except Exception:
        pass
    try:
        wlan.disconnect()
    except OSError:
        pass
    wlan.config(channel=channel)
    if pm:
        try:
            wlan.config(pm=wlan.PM_NONE)
        except Exception:
            pass
    if not wlan.active() or wlan.config("channel") != channel:
        raise OSError("Wi-Fi did not start on channel %d" % channel)


class Bridge:
    def __init__(self, peer, pmk, lmk, channel, txpower=None):
        import binascii
        import espnow
        import network

        self.key = (peer, pmk, lmk, channel, txpower)
        wlan = network.WLAN(network.STA_IF)
        # The full reset, unless it hung the board last time (WIFI_MARK).
        self.wr = 0 if _exists(WIFI_MARK) else 1
        if self.wr:
            try:
                with open(WIFI_MARK, "w") as f:
                    f.write("1")
            except OSError:
                pass
        try:
            wifi_reset(wlan, int(channel), self.wr)
        except Exception:
            self._unmark()          # only a hang leaves the mark
            raise
        self._unmark()
        if txpower is not None:
            wlan.config(txpower=txpower)     # dBm; None: the default
        self.e = espnow.ESPNow()
        try:
            self.e.config(rxbuf=RXBUF)     # takes effect on active(True)
        except Exception:
            pass
        self.e.active(True)
        self.e.set_pmk(binascii.unhexlify(pmk))
        self.peer = binascii.unhexlify(peer.replace(":", ""))
        self.lmk = binascii.unhexlify(lmk) if lmk else None   # None: unencrypted
        try:
            self.e.add_peer(self.peer, self.lmk)
        except OSError:
            # Already added by an earlier Bridge in this session.
            self.e.mod_peer(self.peer, self.lmk)
        self.seq = int.from_bytes(os.urandom(2), "big")
        # Heartbeat counters (hbstat): sent, acked; unacked in a row now and
        # the tick that run started; the last 8 gaps of HB_GAP or more as
        # [start tick, length ms].
        self.hb = [0, 0]
        self.hb_run = 0
        self.hb_t = 0
        self.hb_last = None
        self.hb_gaps = []
        if HEARTBEAT_MS:
            import machine
            machine.Timer(HEARTBEAT_TIMER).init(
                period=HEARTBEAT_MS, mode=machine.Timer.PERIODIC,
                callback=self._beat)

    def _beat(self, _timer):
        """Timer callback: send one heartbeat, at most every HEARTBEAT_MS."""
        now = time.ticks_ms()
        if self.hb_last is not None and \
                time.ticks_diff(now, self.hb_last) < HEARTBEAT_MS:
            return
        self.hb_last = now
        try:
            ok = self.e.send(self.peer, b"h", True)
        except OSError:
            ok = False          # e.g. between del_peer and add_peer (rekey)
        self.hb[0] += 1
        if ok:
            self.hb[1] += 1
            if self.hb_run >= HB_GAP:
                self.hb_gaps.append([self.hb_t, time.ticks_diff(now, self.hb_t)])
                del self.hb_gaps[:-8]
            self.hb_run = 0
        else:
            if not self.hb_run:
                self.hb_t = now
            self.hb_run += 1

    def hbstat(self):
        """Heartbeat counters as JSON text: sent, acked, unacked in a row now
        (and since which tick), the logged gaps, and the tick now."""
        return json.dumps({"hb": self.hb, "run": self.hb_run, "since": self.hb_t,
                           "gaps": self.hb_gaps, "now": time.ticks_ms()})

    def rtest(self, seconds, rate=50, echo=1, wait_ms=20000):
        """Radio test against the arm's test mode (arm_radio.RadioTest),
        which the PC has just asked for. Returns JSON text.

        Sends b"T" + 4-byte sequence number `rate` times a second. Waits up
        to wait_ms for the arm's test mode to answer (an echo; without echo,
        an ack), then measures for seconds - 1 (the arm's test time started
        a little earlier). Per second: sent, acked by the arm's radio,
        echoed back, signal of the last echo (dBm). Longest stretches with
        no ack and no echo: [ms, start ms].
        """
        period = max(1, 1000 // rate)
        seq = acked = 0
        self.drain()
        t0 = time.ticks_ms()
        while True:
            try:
                ok = self.e.send(self.peer, b"T" + seq.to_bytes(4, "big"), True)
            except OSError:
                ok = False
            seq += 1
            acked += ok
            if ok and not echo:
                break
            mac, msg = self.e.irecv(100)
            if msg and mac == self.peer and msg[0] == 0x54:
                break
            if time.ticks_diff(time.ticks_ms(), t0) > wait_ms:
                # How many the arm's radio acked while we waited: some = its
                # radio hears but no echo came; none = no radio contact.
                return json.dumps({"err": "no echo from the arm's test mode",
                                   "wait": wait_ms, "wait_sent": seq,
                                   "wait_ack": acked})
        waited = time.ticks_diff(time.ticks_ms(), t0)
        n = max(1, seconds - 1)
        sent, ack, back, rssi = [0] * n, [0] * n, [0] * n, [None] * n
        ack_gap, echo_gap = [0, 0], [0, 0]
        start = time.ticks_ms()
        next_send = last_ack = last_echo = start
        while True:
            now = time.ticks_ms()
            t = time.ticks_diff(now, start)
            if t >= n * 1000:
                break
            i = t // 1000
            if time.ticks_diff(now, next_send) >= 0:
                try:
                    ok = self.e.send(self.peer, b"T" + seq.to_bytes(4, "big"), True)
                except OSError:
                    ok = False
                seq += 1
                sent[i] += 1
                now = time.ticks_ms()
                if ok:
                    ack[i] += 1
                    if time.ticks_diff(now, last_ack) > ack_gap[0]:
                        ack_gap = [time.ticks_diff(now, last_ack),
                                   time.ticks_diff(last_ack, start)]
                    last_ack = now
                next_send = time.ticks_add(next_send, period)
                if time.ticks_diff(now, next_send) > period:
                    next_send = now          # fell behind: no burst
            mac, msg = self.e.irecv(max(0, time.ticks_diff(next_send,
                                                           time.ticks_ms())))
            while msg:
                if mac == self.peer and msg[0] == 0x54:
                    now = time.ticks_ms()
                    j = min(n - 1, time.ticks_diff(now, start) // 1000)
                    back[j] += 1
                    rssi[j] = self.rssi()
                    if time.ticks_diff(now, last_echo) > echo_gap[0]:
                        echo_gap = [time.ticks_diff(now, last_echo),
                                    time.ticks_diff(last_echo, start)]
                    last_echo = now
                mac, msg = self.e.irecv(0)
        end = n * 1000
        for gap, last in ((ack_gap, last_ack), (echo_gap, last_echo)):
            at = time.ticks_diff(last, start)
            if end - at > gap[0]:            # a silent end is a gap too
                gap[0], gap[1] = end - at, at
        self.drain()
        return json.dumps({"n": n, "rate": rate, "echo": echo, "wait": waited,
                           "sent": sent, "ack": ack, "back": back, "rssi": rssi,
                           "ack_gap": ack_gap, "echo_gap": echo_gap})

    def _unmark(self):
        if self.wr:
            try:
                os.remove(WIFI_MARK)
            except OSError:
                pass

    def rssi(self):
        """Signal strength of the arm's last message in dBm, or None."""
        try:
            return self.e.peers_table[self.peer][0]
        except (KeyError, AttributeError):
            return None

    def rekey(self):
        """Remove the arm as a peer and add it again, so this side's link
        state for it starts afresh. The arm may have been switched off and on
        while this remote stayed powered over USB; its replies are then acked
        by this radio but never delivered."""
        try:
            self.e.del_peer(self.peer)
        except OSError:
            pass
        self.e.add_peer(self.peer, self.lmk)

    def drain(self):
        """Throw away waiting messages: late copies of earlier replies (the
        arm answers every resend) that would fill the receive buffer."""
        while True:
            mac, data = self.e.irecv(0)
            if not data:
                return

    def fresh(self):
        """Start of a PC session: the arm may have restarted since the last one."""
        self.drain()
        self.rekey()

    def call(self, body, timeout_ms=60000, quiet_ms=5000):
        """Send one request and return the arm's final reply (a JSON string).

        body is the request without its sequence number, as JSON text after
        the opening bracket: '"c", "turn_base", [45], {}, 60]'.
        Raises OSError if the arm does not answer: "no radio contact" when
        not one message was acknowledged by the arm's radio within
        NO_CONTACT_MS (so the arm never got the request), "no reply from the
        arm" when its radio heard us but nothing came back within NO_REPLY_MS.
        timeout_ms counts from the arm's first answer.
        """
        self.seq = (self.seq + 1) & 0xFFFF
        seq = self.seq
        msg = "[%d, %s" % (seq, body)
        if len(msg) > MAX_MSG:
            # send() would fail, and that would look like "no radio contact".
            raise ValueError("request too long: %d bytes (ESP-NOW max %d)"
                             % (len(msg), MAX_MSG))
        self.drain()
        start = time.ticks_ms()
        sent = heard = first = None
        acked = rekeyed = False
        while True:
            now = time.ticks_ms()
            # Until the arm answers, resend quickly. After that, check in now
            # and then so a lost final reply is sent again. Every resend has
            # the same seq, so the arm never runs the request twice.
            gap = 300 if heard is None else 1500
            if sent is None or time.ticks_diff(now, sent) >= gap:
                if heard is None:
                    waited = time.ticks_diff(now, start)
                    if waited >= (NO_REPLY_MS if acked else NO_CONTACT_MS):
                        raise OSError("no reply from the arm" if acked
                                      else "no radio contact")
                    if acked and not rekeyed and waited >= REKEY_AFTER_MS:
                        rekeyed = True
                        self.rekey()
                try:
                    if self.e.send(self.peer, msg, True):
                        acked = True
                except OSError:
                    pass                # e.g. ETIMEDOUT: this radio is jammed
                sent = now
            if first is not None and time.ticks_diff(now, first) > timeout_ms:
                raise OSError("timed out waiting for the arm")
            if heard is not None and time.ticks_diff(now, heard) > quiet_ms:
                raise OSError("lost contact with the arm")
            mac, data = self.e.irecv(20)
            if not data or mac != self.peer or data[0] != 0x5B:   # '['
                continue
            text = bytes(data).decode()
            try:
                reply = json.loads(text)
            except ValueError:
                continue
            if reply[0] != seq:
                continue
            heard = time.ticks_ms()
            if first is None:
                first = heard
            if reply[1] != "A":
                return text


_bridge = None


def get(peer, pmk, lmk, channel, txpower=None):
    """The bridge for these settings, made once per boot."""
    global _bridge
    key = (peer, pmk, lmk, channel, txpower)
    if _bridge is None or _bridge.key != key:
        _bridge = Bridge(peer, pmk, lmk, channel, txpower)
    return _bridge
