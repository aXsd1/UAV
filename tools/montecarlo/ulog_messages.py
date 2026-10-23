"""Count PX4 log messages (WARNING and above) across Monte Carlo ULogs.

python3 -m tools.montecarlo.ulog_messages runs/montecarlo/*_transition [--grep "Compass"]

Prints, for each message text (timestamps and numbers kept as logged), how many runs logged it,
and with --grep the runs and flight times where a matching message appeared.
"""

from __future__ import annotations

import argparse
import collections
import glob
from pathlib import Path

from pyulog import ULog


def messages(path: Path, min_level: int = 4):
    """(time_s, level, text) of logged messages; ULog levels: 0 emerg ... 4 warning ... 7 debug."""
    try:
        u = ULog(str(path), message_name_filter_list=["vehicle_status"])
    except Exception:  # truncated log
        return []
    t0 = u.start_timestamp
    out = []
    for m in u.logged_messages:
        lvl = m.log_level if isinstance(m.log_level, int) else ord(m.log_level)
        if lvl >= ord("0"):  # stored as the ASCII digit
            lvl -= ord("0")
        if lvl <= min_level:
            out.append(((m.timestamp - t0) / 1e6, lvl, m.message))
    return out


def main():
    ap = argparse.ArgumentParser(prog="python3 -m tools.montecarlo.ulog_messages")
    ap.add_argument("dirs", nargs="+")
    ap.add_argument("--grep", default=None)
    a = ap.parse_args()
    count = collections.Counter()
    hits = []
    logs = [Path(p) for d in a.dirs for p in glob.glob(f"{d}/logs/*.ulg")]
    for p in logs:
        seen = set()
        for t, lvl, text in messages(p):
            if text not in seen:
                count[text] += 1
                seen.add(text)
            if a.grep and a.grep.lower() in text.lower():
                hits.append((p, t, text))
    print(f"{len(logs)} logs")
    for text, n in count.most_common(40):
        print(f"{n:5d}  {text}")
    if a.grep:
        print(f"\n'{a.grep}': {len({h[0] for h in hits})} runs")
        for p, t, text in hits[:50]:
            print(f"  {p.parent.parent.name}/{p.name}  t={t:.1f} s  {text}")


if __name__ == "__main__":
    main()
