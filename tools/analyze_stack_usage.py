#!/usr/bin/env python3
"""Summarize GCC -fstack-usage files from target_build."""

from __future__ import annotations

import pathlib
import sys


def main() -> int:
    root = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else "target_build")
    rows: list[tuple[int, str, str]] = []
    for su in root.glob("*.su"):
        for line in su.read_text(errors="replace").splitlines():
            parts = line.split()
            if len(parts) < 2:
                continue
            try:
                stack = int(parts[1])
            except ValueError:
                continue
            rows.append((stack, su.name, parts[0]))

    if not rows:
        print("No .su stack-usage files found.")
        return 0

    rows.sort(reverse=True)
    total_static = sum(r[0] for r in rows)
    print("\n=== Stack Usage Summary (-fstack-usage) ===")
    print(f"Functions analysed : {len(rows)}")
    print(f"Largest frame      : {rows[0][0]} bytes  {rows[0][2]}  ({rows[0][1]})")
    print(f"Static sum         : {total_static} bytes  (not callgraph worst-case)")
    print("\nTop frames:")
    for stack, file_name, func in rows[:20]:
        print(f"  {stack:6d} B  {func}  [{file_name}]")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
