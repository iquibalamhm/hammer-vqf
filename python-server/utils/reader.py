import sys
import time
import threading
import queue

try:
    import serial  # optional dependency for serial ports
except Exception:
    serial = None


class ReaderThread(threading.Thread):
    """Background reader: serial or file → queue of parsed records."""

    def __init__(self, port, baud, filepath, out_q, stop_evt, parse_fn):
        super().__init__(daemon=True)
        if parse_fn is None:
            raise ValueError("parse_fn is required for ReaderThread")
        self.port = port
        self.baud = baud
        self.filepath = filepath
        self.q = out_q
        self.stop_evt = stop_evt
        self.parse_fn = parse_fn
        self.ser = None
        self.fp = None

    def open(self):
        if self.filepath:
            self.fp = open(self.filepath, "r", encoding="utf-8")
            return True
        if serial is None:
            print("pyserial not installed. Use --file or pip install pyserial", file=sys.stderr)
            return False
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            return True
        except Exception as exc:
            print(f"Failed to open serial: {exc}", file=sys.stderr)
            return False

    def readline(self):
        if self.fp:
            return self.fp.readline()
        if self.ser:
            data = self.ser.readline()
            if isinstance(data, bytes):
                try:
                    return data.decode("utf-8", "ignore")
                except Exception:
                    return ""
        return ""

    def close(self):
        try:
            if self.fp:
                self.fp.close()
        except Exception:
            pass
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass

    def run(self):
        if not self.open():
            return
        try:
            while not self.stop_evt.is_set():
                line = self.readline()
                if not line:
                    time.sleep(0.001)  # back off to keep CPU usage low
                    continue
                record = self.parse_fn(line)
                if record is None:
                    continue
                try:
                    self.q.put_nowait(record)
                except queue.Full:
                    # Drop samples if the consumer is lagging; lowers latency.
                    pass
        finally:
            self.close()


__all__ = ["ReaderThread"]
