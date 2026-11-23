import re

NAME = "Stack / heap usage"
SHORT = "Stack"
DESCRIPTION = "Parses [LIVE-ST-] lines from controller.c (task stack now/peak words, minimum-ever free heap)"

TASK_RE = re.compile(r"\[LIVE-ST-\]\s+(.+?):\s+now\s+(\d+),\s+peak\s+(\d+)\s*/\s*(\d+)\s+words")
HEAP_RE = re.compile(r"\[LIVE-ST-\]\s+total Free:\s+(\d+)\s+B")


def extract(line):
    m = TASK_RE.search(line)
    if m:
        name, now, peak, total = m.group(1), int(m.group(2)), int(m.group(3)), int(m.group(4))
        return [{"series": name, "trace": "now", "value": now, "total": total},
                {"series": name, "trace": "peak", "value": peak, "total": total}]
    m = HEAP_RE.search(line)
    if m:
        return [{"series": "heap min free (B)", "value": int(m.group(1))}]
    return None
