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


class Bridge:
    def __init__(self, peer, pmk, lmk, channel):
        import binascii
        import espnow
        import network

        self.key = (peer, pmk, lmk, channel)
        wlan = network.WLAN(network.STA_IF)
        # active(True) never returns if the stock firmware already started
        # Wi-Fi, so only switch it on when it is off.
        if not wlan.active():
            wlan.active(True)
        try:
            wlan.disconnect()
        except OSError:
            pass
        wlan.config(channel=int(channel))
        self.e = espnow.ESPNow()
        self.e.active(True)
        self.e.set_pmk(binascii.unhexlify(pmk))
        self.peer = binascii.unhexlify(peer.replace(":", ""))
        try:
            self.e.add_peer(self.peer, binascii.unhexlify(lmk))
        except OSError:
            # Already added by an earlier Bridge in this session.
            self.e.mod_peer(self.peer, binascii.unhexlify(lmk))
        self.seq = int.from_bytes(os.urandom(2), "big")

    def rssi(self):
        """Signal strength of the arm's last message in dBm, or None."""
        try:
            return self.e.peers_table[self.peer][0]
        except (KeyError, AttributeError):
            return None

    def call(self, body, timeout_ms=60000, quiet_ms=5000):
        """Send one request and return the arm's final reply (a JSON string).

        body is the request without its sequence number, as JSON text after
        the opening bracket: '"c", "turn_base", [45], {}, 60]'.
        Raises OSError if the arm does not answer.
        """
        self.seq = (self.seq + 1) & 0xFFFF
        seq = self.seq
        msg = "[%d, %s" % (seq, body)
        start = time.ticks_ms()
        sent = heard = None
        tries = 0
        while True:
            now = time.ticks_ms()
            # Until the arm answers, resend quickly. After that, check in now
            # and then so a lost final reply is sent again.
            gap = 300 if heard is None else 1500
            if sent is None or time.ticks_diff(now, sent) >= gap:
                if heard is None and tries >= 6:
                    raise OSError("no reply from the arm")
                tries += 1
                try:
                    self.e.send(self.peer, msg, True)
                except OSError:
                    pass
                sent = now
            if time.ticks_diff(now, start) > timeout_ms:
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
            if reply[1] != "A":
                return text


_bridge = None


def get(peer, pmk, lmk, channel):
    """The bridge for these settings, made once per boot."""
    global _bridge
    key = (peer, pmk, lmk, channel)
    if _bridge is None or _bridge.key != key:
        _bridge = Bridge(peer, pmk, lmk, channel)
    return _bridge
