"""Web console for the ALANv serial debug output.

Usage: python3 console_portal.py [port] [baud] [--http-port 8000]

Opens a web page (default http://localhost:8000) with:
  * a raw serial log and a command line (sent with CRLF)
  * an extracted-info section (current values + graphs over time) fed by the extractors
    you enable in the page

Extractors are plugin files in test/extractors/*.py (files starting with "_" are ignored).
Each defines NAME (tab title), optionally SHORT (short name for the enable checkbox at the top
of the page, defaults to NAME), DESCRIPTION (checkbox tooltip), DISPLAY, and:

    def extract(line: str):
        # return None, or a list of {"series": str, "value": float,
        #                            "trace": str (optional), "total": float (optional)}

"series" is one graph/card. Samples sharing a series but with different "trace" names are
drawn as separate lines on that same graph (e.g. "now" and "peak").

DISPLAY chooses how the extractor's tab renders its series: "graphs" (default: a current-value
card plus a graph over time per series) or "value" (current-value cards only).
Graph options (all optional): WINDOW (seconds; graph rolls and shows only the last N seconds
instead of the whole run), YMIN / YMAX (fixed vertical axis limits).
Each enabled extractor gets its own tab in the page.

Modules may keep their own state between lines. Use "Reload" in the page to pick up edits.
The enabled set is stored in extractors_enabled.json.
The Raw log tab also has a stress test that blasts random console commands at the board.
Everything is logged to a test/logs_<date>/ folder (raw .log and extracted .csv).
Requires: pyserial
"""
import argparse
import csv
import importlib.util
import json
import random
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
samples = []     # {"seq", "t", "ext", "name", "trace", "value", "total"}
extractors = {}  # id -> {"id", "name", "description", "display", "module", "error", "enabled"}
t0 = time.time()
ser = None
send_lock = threading.Lock()
stress = {"running": False, "sent": 0, "stop": False}
raw_file = csv_writer = csv_file = None


def load_extractors():
    """(Re)import every extractor file, keeping the enabled selection."""
    enabled = set(json.loads(ENABLED_PATH.read_text()) if ENABLED_PATH.exists() else DEFAULT_ENABLED)
    found = {}
    for path in sorted(EXTRACTOR_DIR.glob("*.py")):
        if path.name.startswith("_"):
            continue
        eid = path.stem
        entry = {"id": eid, "name": eid, "short": eid, "description": "", "display": "graphs", "window": None, "ymin": None, "ymax": None, "module": None, "error": "", "enabled": eid in enabled}
        try:
            spec = importlib.util.spec_from_file_location(f"extractor_{eid}", path)
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            if not callable(getattr(mod, "extract", None)):
                raise AttributeError("missing extract(line) function")
            entry.update(module=mod, name=getattr(mod, "NAME", eid), short=getattr(mod, "SHORT", getattr(mod, "NAME", eid)), description=getattr(mod, "DESCRIPTION", ""),
                         display=getattr(mod, "DISPLAY", "graphs"), window=getattr(mod, "WINDOW", None),
                         ymin=getattr(mod, "YMIN", None), ymax=getattr(mod, "YMAX", None))
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
        return [{k: e[k] for k in ("id", "name", "short", "description", "display", "window", "ymin", "ymax", "error", "enabled")} for e in extractors.values()]


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
                s = {"seq": len(samples), "t": round(now, 3), "ext": e["id"], "name": str(r["series"]),
                     "trace": str(r.get("trace", "")),
                     "value": float(r["value"]), "total": None if total is None else float(total)}
                samples.append(s)
                csv_writer.writerow([s["t"], s["ext"], s["name"], s["trace"], s["value"], "" if total is None else s["total"]])
        csv_file.flush()


def send(cmd):
    with send_lock:
        ser.write((cmd + "\r\n").encode())


def random_command(bad):
    options = [f"heading {random.randint(0, 359)}", f"rudder {random.randint(-90, 90)}",
               f"tail {random.randint(-90, 90)}", f"sail {random.randint(-180, 180)}", "status", "center"]
    if bad:
        options += ["heading 99999", "heading -5", "heading", "xyzzy " + "A" * 80]
    return random.choice(options)


def stress_run(rate, duration, bad):
    """Send random commands for `duration` seconds at `rate` per second (0 = as fast as the port allows)."""
    end = time.time() + duration
    next_t = time.time()
    while time.time() < end and not stress["stop"]:
        send(random_command(bad))
        stress["sent"] += 1
        if rate > 0:
            next_t += 1.0 / rate
            time.sleep(max(0, next_t - time.time()))
    stress["running"] = False


def stress_start(rate, duration, bad):
    if stress["running"]:
        return
    stress.update(running=True, stop=False, sent=0)
    threading.Thread(target=stress_run, args=(rate, duration, bad), daemon=True).start()


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
                self.reply(json.dumps({"log": log_lines[n:], "samples": samples[sq:],
                                        "stress": {"running": stress["running"], "sent": stress["sent"]}}))
        elif u.path == "/api/extractors":
            self.reply(json.dumps(extractor_list()))
        else:
            self.reply("not found", "text/plain", 404)

    def do_POST(self):
        body = self.rfile.read(int(self.headers.get("Content-Length", 0)))
        if self.path == "/api/send":
            cmd = json.loads(body)["cmd"]
            send(cmd)
            self.reply("{}")
        elif self.path == "/api/stress":
            req = json.loads(body)
            if req["action"] == "start":
                stress_start(float(req["rate"]), float(req["duration"]), bool(req["bad"]))
            else:
                stress["stop"] = True
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
    csv_writer.writerow(["time_s", "extractor", "name", "trace", "value", "total"])

    ser = serial.Serial(a.port, a.baud, timeout=1)
    threading.Thread(target=reader, daemon=True).start()
    print(f"Serving on http://localhost:{a.http_port}  (serial {a.port} @ {a.baud})")
    ThreadingHTTPServer(("127.0.0.1", a.http_port), Handler).serve_forever()


if __name__ == "__main__":
    main()
