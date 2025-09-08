#!/usr/bin/env python3
"""
Seed GitHub issues from docs/ISSUE_BACKLOG.md.

Usage:
  - Preview (no network): python scripts/seed_issues.py --preview
  - Create issues (requires GITHUB_TOKEN):
      GITHUB_TOKEN=... python scripts/seed_issues.py --repo alpharover/alpha_viam_rover

Parses numbered backlog items and builds issue titles/bodies.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import sys
from pathlib import Path
from typing import List, Dict


BACKLOG = Path("docs/ISSUE_BACKLOG.md")


def parse_backlog(md: str) -> List[Dict[str, str]]:
    items: List[Dict[str, str]] = []
    lines = md.splitlines()
    i = 0
    pattern = re.compile(r"^\s*\d+\)\s+(.*)")
    while i < len(lines):
        m = pattern.match(lines[i])
        if m:
            title = m.group(1).strip()
            body_lines: List[str] = []
            i += 1
            while i < len(lines) and not pattern.match(lines[i]):
                body_lines.append(lines[i])
                i += 1
            body = "\n".join(body_lines).strip()
            # Normalize bullet labels
            body = body.replace("- AC:", "### Acceptance Criteria\n").replace(
                "- Evidence:", "\n### Evidence\n"
            )
            items.append({"title": title, "body": body or title})
        else:
            i += 1
    return items


def create_issue(repo: str, token: str, title: str, body: str) -> Dict:
    import urllib.request

    url = f"https://api.github.com/repos/{repo}/issues"
    data = json.dumps({"title": title, "body": body}).encode("utf-8")
    req = urllib.request.Request(url, data=data, method="POST")
    req.add_header("Authorization", f"token {token}")
    req.add_header("Accept", "application/vnd.github+json")
    with urllib.request.urlopen(req) as resp:
        return json.load(resp)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--repo", help="owner/repo (e.g., alpharover/alpha_viam_rover)")
    ap.add_argument("--preview", action="store_true", help="print JSON preview and exit")
    args = ap.parse_args()

    if not BACKLOG.exists():
        print(f"Backlog file not found: {BACKLOG}", file=sys.stderr)
        return 2

    items = parse_backlog(BACKLOG.read_text(encoding="utf-8"))
    if args.preview or not args.repo:
        out = {"count": len(items), "issues": items}
        print(json.dumps(out, indent=2))
        # also write preview file
        Path("out").mkdir(exist_ok=True)
        Path("out/issues_seed_preview.json").write_text(json.dumps(out, indent=2), encoding="utf-8")
        return 0

    token = os.environ.get("GITHUB_TOKEN")
    if not token:
        print(
            "GITHUB_TOKEN is required to create issues. Use --preview for dry-run.", file=sys.stderr
        )
        return 3

    created: List[Dict] = []
    for it in items:
        print(f"Creating: {it['title']} …", file=sys.stderr)
        resp = create_issue(args.repo, token, it["title"], it["body"])
        created.append({"title": it["title"], "url": resp.get("html_url")})
    print(json.dumps({"created": created}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
