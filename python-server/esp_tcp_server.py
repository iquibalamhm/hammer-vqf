#!/usr/bin/env python3
"""Minimal TCP bridge for ESP32 gait telemetry.

This script accepts a single ESP32 client connection, streams incoming foot
orientation angles, and lets you send commands (e.g. "ze") back to the board
from the terminal. It pairs with the firmware's Wi-Fi client that connects to a
laptop-hosted TCP server.

Usage:
  python esp_tcp_server.py --host 0.0.0.0 --port 12345

Once the ESP32 connects, type commands into the prompt (for example, enter
"ze" and press ENTER to zero the foot orientation).
"""
from __future__ import annotations

import argparse
import socket
import threading
import time
import re
import select
import sys
from typing import Optional

FOOT_RE = re.compile(
    r"footRoll_deg=([-+]?\d*\.?\d+|nan),"
    r"footPitch_deg=([-+]?\d*\.?\d+|nan),"
    r"footYaw_deg=([-+]?\d*\.?\d+|nan)",
    re.IGNORECASE,
)

FPA_RE = re.compile(r"fpa_deg=([-+]?\d*\.?\d+)", re.IGNORECASE)


def _to_float(token: str) -> float:
    try:
        return float(token)
    except ValueError:
        return float("nan")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="ESP32 TCP telemetry bridge")
    parser.add_argument("--host", default="0.0.0.0",
                        help="Host/IP for the server to bind (default: 0.0.0.0)")
    parser.add_argument("--port", type=int, default=12345,
                        help="TCP port to listen on (default: 12345)")
    parser.add_argument("--idle-timeout", type=float, default=0.0,
                        help="Optional idle timeout in seconds (0 disables)")
    return parser.parse_args()


class TcpBridge:
    def __init__(self, host: str, port: int, idle_timeout: float) -> None:
        self.host = host
        self.port = port
        self.idle_timeout = idle_timeout
        self.conn: Optional[socket.socket] = None
        self.conn_lock = threading.Lock()
        self.stop_evt = threading.Event()

    # ------------------------ command handling ------------------------
    def send_command(self, command: str) -> None:
        cmd = command.strip()
        if not cmd:
            return
        with self.conn_lock:
            if not self.conn:
                print("[warn] No active ESP32 connection; command ignored")
                return
            try:
                self.conn.sendall((cmd + "\n").encode("utf-8"))
                print(f"📤 sent: {cmd}")
            except OSError as exc:
                print(f"[error] failed to send command: {exc}")

    def _command_loop(self) -> None:
        mapping = {
            "z": "ze",
            "ze": "ze",
        }
        prompt_visible = False
        stdin = sys.stdin
        while not self.stop_evt.is_set():
            with self.conn_lock:
                active_conn = self.conn is not None
            if not active_conn:
                break
            if not prompt_visible:
                print("cmd> ", end="", flush=True)
                prompt_visible = True
            try:
                ready, _, _ = select.select([stdin], [], [], 0.2)
            except (OSError, ValueError):
                continue
            if self.stop_evt.is_set():
                break
            if not ready:
                continue
            line = stdin.readline()
            prompt_visible = False
            if line == "":
                self.stop_evt.set()
                break
            user_input = line.strip()
            if not user_input:
                continue
            if user_input.lower() in ("quit", "exit"):
                self.stop_evt.set()
                break
            mapped = mapping.get(user_input.lower(), user_input)
            self.send_command(mapped)
        if prompt_visible:
            print()

    # ------------------------ telemetry handling ----------------------
    @staticmethod
    def _parse_line(line: str) -> None:
        txt = line.strip()
        if not txt:
            return
        m = FOOT_RE.search(txt)
        if m:
            roll, pitch, yaw = (_to_float(tok) for tok in m.groups())
            roll_str = f"{roll:6.2f}" if roll == roll else "  nan"
            pitch_str = f"{pitch:6.2f}" if pitch == pitch else "  nan"
            yaw_str = f"{yaw:6.2f}" if yaw == yaw else "  nan"
            # print(f"📡 foot r/p/y = {roll_str} / {pitch_str} / {yaw_str} deg")
        else:
            fpa = FPA_RE.search(txt)
            if fpa:
                try:
                    value = float(fpa.group(1))
                    print(f"📈 FPA = {value:.2f} deg")
                except ValueError:
                    pass
            else:
                print(f"ℹ️  {txt}")

    def _telemetry_loop(self) -> None:
        buffer = b""
        last_activity = time.monotonic()
        while not self.stop_evt.is_set():
            with self.conn_lock:
                conn = self.conn
            if conn is None:
                break
            try:
                data = conn.recv(1024)
            except OSError as exc:
                print(f"[error] recv failed: {exc}")
                break
            if not data:
                print("[info] ESP32 disconnected")
                break
            buffer += data
            last_activity = time.monotonic()
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                try:
                    decoded = line.decode("utf-8", "ignore")
                except UnicodeDecodeError:
                    continue
                self._parse_line(decoded)
            if self.idle_timeout > 0.0 and (time.monotonic() - last_activity) > self.idle_timeout:
                print("[warn] idle timeout reached")
                break

    # ------------------------ server accept loop ----------------------
    def run(self) -> None:
        print(f"📶 waiting for ESP32 on {self.host}:{self.port} ...")
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as srv:
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            srv.bind((self.host, self.port))
            srv.listen(1)
            while not self.stop_evt.is_set():
                try:
                    srv.settimeout(1.0)
                    conn, addr = srv.accept()
                except socket.timeout:
                    continue
                except OSError as exc:
                    print(f"[error] accept failed: {exc}")
                    continue
                with self.conn_lock:
                    self.conn = conn
                print(f"✅ connected: {addr}")

                command_thread = threading.Thread(target=self._command_loop, daemon=True)
                telemetry_thread = threading.Thread(target=self._telemetry_loop, daemon=True)
                command_thread.start()
                telemetry_thread.start()

                telemetry_thread.join()

                with self.conn_lock:
                    conn.close()
                    self.conn = None
                command_thread.join()
                if self.stop_evt.is_set():
                    break
                print("[info] waiting for reconnection...")
        print("[info] server stopped")


def main() -> None:
    args = parse_args()
    bridge = TcpBridge(args.host, args.port, args.idle_timeout)
    try:
        bridge.run()
    except KeyboardInterrupt:
        print("\n[info] interrupted, shutting down")


if __name__ == "__main__":
    main()
