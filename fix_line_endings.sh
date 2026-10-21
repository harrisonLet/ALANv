#!/usr/bin/env bash
# Convert CRLF to LF in all tracked text files (binary files are skipped).
set -euo pipefail

cd "$(git rev-parse --show-toplevel)"

count=0
while IFS= read -r -d '' f; do
    [[ -f "$f" ]] || continue
    # Skip binary files
    grep -Iq . "$f" 2>/dev/null || continue
    # Skip files without CR
    grep -q $'\r' "$f" || continue

    sed -i 's/\r$//' "$f"
    echo "fixed: $f"
    count=$((count + 1))
done < <(git ls-files -z)

echo "Done. $count file(s) converted to LF."
