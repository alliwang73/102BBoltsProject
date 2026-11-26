#!/usr/bin/env python3
"""Copy bolt-control/index.html into BoltSorter/BoltSorter.ino so the ESP32
serves the same control page over Wi-Fi. Run after editing the HTML:

    python3 tools/embed_control_page.py
"""
import pathlib, re, sys

root = pathlib.Path(__file__).resolve().parent.parent
html = (root / "bolt-control" / "index.html").read_text(encoding="utf-8")
ino_path = root / "BoltSorter" / "BoltSorter.ino"
ino = ino_path.read_text(encoding="utf-8")

DELIM = "PAGE"
if f'){DELIM}"' in html:
    sys.exit(f'HTML contains the raw-string terminator ){DELIM}" - change DELIM')

begin, end = "// <<CONTROL_PAGE_BEGIN>>\n", "// <<CONTROL_PAGE_END>>"
if begin not in ino or end not in ino:
    sys.exit("embed markers not found in BoltSorter.ino")

block = f'const char CONTROL_PAGE[] PROGMEM = R"{DELIM}({html}){DELIM}";\n'
new = re.sub(re.escape(begin) + r".*?" + re.escape(end),
             lambda _: begin + block + end, ino, flags=re.S)
ino_path.write_text(new, encoding="utf-8")
print(f"embedded {len(html):,} bytes of HTML into {ino_path.relative_to(root)}")
