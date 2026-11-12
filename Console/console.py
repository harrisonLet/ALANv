"""Web console for the ALANv serial debug output.

Usage: python3 console_portal.py [port] [baud] [--http-port 8000]

Opens a web page (default http://localhost:8000) with:
  * a raw serial log and a command line (sent with CRLF)
  * an extracted-info section (current values + graphs over time) fed by the extractors
    you enable in the page

Extractors are plugin files in test/extractors/*.py (files starting with "_" are ignored).
Each defines NAME, optionally DESCRIPTION, and:

    def extract(line: str):
        # return None, or a list of {"series": str, "value": float, "total": float (optional)}

Modules may keep their own state between lines. Use "Reload" in the page to pick up edits.
The enabled set is stored in extractors_enabled.json.
Everything is logged to a test/logs_<date>/ folder (raw .log and extracted .csv).
Requires: pyserial
"""
import argparse
import csv
import importlib.util
import json
import threading
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

import serial

HERE = Path(__file__).parent
EXTRACTOR_DIR = HERE / "extractors"
ENABLED_PATH = HERE / "extractors_enabled.json"
LOG_DIR = HERE / ("log_" + datetime.now().strftime("%Y%m%d"))
DEFAULT_ENABLED = ["stack"]

lock = threading.Lock()
log_lines = []   # raw lines
samples = []     # {"seq", "t", "name", "value", "total"}
extractors = {}  # id -> {"id", "name", "description", "module", "error", "enabled"}
t0 = time.time()
ser = None
raw_file = csv_writer = csv_file = None


def load_extractors():
    """(Re)import every extractor file, keeping the enabled selection."""
    enabled = set(json.loads(ENABLED_PATH.read_text()) if ENABLED_PATH.exists() else DEFAULT_ENABLED)
    found = {}
    for path in sorted(EXTRACTOR_DIR.glob("*.py")):
        if path.name.startswith("_"):
            continue
        eid = path.stem
        entry = {"id": eid, "name": eid, "description": "", "module": None, "error": "", "enabled": eid in enabled}
        try:
            spec = importlib.util.spec_from_file_location(f"extractor_{eid}", path)
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            if not callable(getattr(mod, "extract", None)):
                raise AttributeError("missing extract(line) function")
            entry.update(module=mod, name=getattr(mod, "NAME", eid), description=getattr(mod, "DESCRIPTION", ""))
        except Exception as e:
            entry["error"] = f"{type(e).__name__}: {e}"
        found[eid] = entry
    with lock:
        extractors.clear()
        extractors.update(found)


def set_enabled(ids):
    with lock:
        for eid, e in extractors.items():
            e["enabled"] = eid in ids
    ENABLED_PATH.write_text(json.dumps(sorted(ids)))


def extractor_list():
    with lock:
        return [{k: e[k] for k in ("id", "name", "description", "error", "enabled")} for e in extractors.values()]


def handle_line(line):
    now = time.time() - t0
    with lock:
        log_lines.append(line)
        raw_file.write(line + "\n")
        raw_file.flush()
        for e in extractors.values():
            if not e["enabled"] or e["module"] is None:
                continue
            try:
                results = e["module"].extract(line) or []
            except Exception as ex:
                e["error"] = f"{type(ex).__name__}: {ex}"
                continue
            for r in results:
                total = r.get("total")
                s = {"seq": len(samples), "t": round(now, 3), "name": str(r["series"]),
                     "value": float(r["value"]), "total": None if total is None else float(total)}
                samples.append(s)
                csv_writer.writerow([s["t"], s["name"], s["value"], "" if total is None else s["total"]])
        csv_file.flush()


def reader():
    buf = b""
    while True:
        data = ser.read(ser.in_waiting or 1)
        buf += data
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            handle_line(line.decode(errors="replace").rstrip("\r"))


PAGE_PATH = HERE / "portal.html"



class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def reply(self, body, ctype="application/json", code=200):
        b = body.encode() if isinstance(body, str) else body
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(b)))
        self.end_headers()
        self.wfile.write(b)

    def do_GET(self):
        u = urlparse(self.path)
        if u.path == "/":
            self.reply(PAGE_PATH.read_text(), "text/html; charset=utf-8")
        elif u.path == "/api/poll":
            q = parse_qs(u.query)
            n, sq = int(q["log"][0]), int(q["seq"][0])
            with lock:
                self.reply(json.dumps({"log": log_lines[n:], "samples": samples[sq:]}))
        elif u.path == "/api/extractors":
            self.reply(json.dumps(extractor_list()))
        else:
            self.reply("not found", "text/plain", 404)

    def do_POST(self):
        body = self.rfile.read(int(self.headers.get("Content-Length", 0)))
        if self.path == "/api/send":
            cmd = json.loads(body)["cmd"]
            ser.write((cmd + "\r\n").encode())
            self.reply("{}")
        elif self.path == "/api/extractors":
            set_enabled(set(json.loads(body)["enabled"]))
            self.reply("{}")
        elif self.path == "/api/extractors/reload":
            load_extractors()
            self.reply("{}")
        else:
            self.reply("not found", "text/plain", 404)


def main():
    global ser, raw_file, csv_file, csv_writer
    ap = argparse.ArgumentParser()
    ap.add_argument("port", nargs="?", default="/dev/ttyACM0")
    ap.add_argument("baud", nargs="?", type=int, default=115200)
    ap.add_argument("--http-port", type=int, default=8001)
    a = ap.parse_args()

    load_extractors()
    LOG_DIR.mkdir(exist_ok=True)
    stamp = f"{datetime.now():%H%M%S}"
    raw_file = open(LOG_DIR / f"raw_{stamp}.log", "w")
    csv_file = open(LOG_DIR / f"extracted_{stamp}.csv", "w", newline="")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(["time_s", "name", "value", "total"])

    ser = serial.Serial(a.port, a.baud, timeout=1)
    threading.Thread(target=reader, daemon=True).start()
    print(f"Serving on http://localhost:{a.http_port}  (serial {a.port} @ {a.baud})")
    ThreadingHTTPServer(("127.0.0.1", a.http_port), Handler).serve_forever()


if __name__ == "__main__":
    main()
