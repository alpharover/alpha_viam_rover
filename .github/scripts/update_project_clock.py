#!/usr/bin/env python3
import datetime as dt
import os
import re
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
README = ROOT / "README.md"
START_FILE = ROOT / "PROJECT_START_DATE"


def git_first_commit_iso() -> str:
    try:
        out = (
            subprocess.check_output(["git", "log", "--reverse", "--format=%cI"], cwd=ROOT)
            .decode()
            .strip()
            .splitlines()
        )
        return out[0] if out else None
    except Exception:
        return None


def get_start_iso() -> str:
    env = os.getenv("PROJECT_START_DATE")
    if env:
        return env
    if START_FILE.exists():
        return START_FILE.read_text().strip()
    first = git_first_commit_iso()
    if first:
        return first
    # Fallback: now
    return dt.datetime.now(dt.timezone.utc).isoformat()


def format_age(start_iso: str) -> str:
    start = dt.datetime.fromisoformat(start_iso.replace("Z", "+00:00"))
    now = dt.datetime.now(dt.timezone.utc)
    delta = now - start
    days = delta.days
    hours, rem = divmod(delta.seconds, 3600)
    minutes, _ = divmod(rem, 60)
    return f"Project age: {days} days, {hours:02d}:{minutes:02d} (since {start.date()})"


def update_readme(age_line: str) -> bool:
    text = README.read_text()
    start_marker = "<!--PROJECT_CLOCK_START-->"
    end_marker = "<!--PROJECT_CLOCK_END-->"
    pattern = re.compile(
        rf"{re.escape(start_marker)}[\s\S]*?{re.escape(end_marker)}",
        re.MULTILINE,
    )
    replacement = f"{start_marker}\n{age_line}\n{end_marker}"
    if pattern.search(text):
        new_text = pattern.sub(replacement, text)
    else:
        # Append a section if markers not present
        new_text = text + "\n\n## Project Clock\n" + replacement + "\n"
    if new_text != text:
        README.write_text(new_text)
        return True
    return False


def main():
    start_iso = get_start_iso()
    age_line = format_age(start_iso)
    changed = update_readme(age_line)
    if not changed:
        print("README unchanged")
    else:
        print(age_line)


if __name__ == "__main__":
    main()
