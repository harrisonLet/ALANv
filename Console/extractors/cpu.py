import re

NAME = "CPU usage"
SHORT = "CPU"
DESCRIPTION = "Parses [LIVE-CPU] lines from controller.c (percent of CPU per task, plus idle/busy)"
WINDOW = 30
YMIN = 0
YMAX = 100

IDLE_RE = re.compile(r"\[LIVE-CPU\]\s+idle\s+([\d.]+)%\s+\(busy\s+([\d.]+)%\)")
TASK_RE = re.compile(r"\[LIVE-CPU\]\s+(.+?):\s+([\d.]+)%")


def extract(line):
    m = IDLE_RE.search(line)
    if m:
        return [{"series": "idle %", "value": float(m.group(1))},
                {"series": "busy %", "value": float(m.group(2))}]
    m = TASK_RE.search(line)
    if m:
        return [{"series": m.group(1), "value": float(m.group(2))}]
    return None
