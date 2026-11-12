import re

NAME = "Stack / heap usage"
DESCRIPTION = "Parses [LIVE-ST-] lines from controller.c (task stack words used, minimum-ever free heap)"

TASK_RE = re.compile(r"\[LIVE-ST-\]\s+(.+?):\s+(\d+)\s*/\s*(\d+)\s+words")
HEAP_RE = re.compile(r"\[LIVE-ST-\]\s+total Free:\s+(\d+)\s+B")


def extract(line):
    m = TASK_RE.search(line)
    if m:
        return [{"series": m.group(1), "value": int(m.group(2)), "total": int(m.group(3))}]
    m = HEAP_RE.search(line)
    if m:
        return [{"series": "heap min free (B)", "value": int(m.group(1))}]
    return None
