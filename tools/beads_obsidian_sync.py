#!/usr/bin/env python3
"""Render beads issue data (.beads/issues.jsonl) into an Obsidian vault.

Reads the git-committed JSONL exports from one or more repos and writes:
  <vault>/PedalKernel/Board.md      — in-progress / ready / blocked / recent
  <vault>/PedalKernel/Backlog.md    — all open work, priority-ordered
  <vault>/PedalKernel/Sprints/*.md  — one page per `sprint:<name>` label

Sprint convention: label a bead `sprint:2026-W28` (bd label add <id> sprint:2026-W28)
and it appears on that sprint page; the current ISO week's page is the active sprint.

Run manually or from the post-commit hook (see tools/install-obsidian-hook.sh).
"""

import json
import sys
from collections import defaultdict
from datetime import datetime, timedelta, timezone
from pathlib import Path

VAULT = Path("/Users/ajmwagar/src/fpl/notes/PedalKernel")
REPOS = {
    "engine": Path("/Users/ajmwagar/src/pedalkernel/pedalkernel/.beads/issues.jsonl"),
    "pro": Path("/Users/ajmwagar/src/pedalkernel/pedalkernel/pedalkernel-pro/.beads/issues.jsonl"),
}
RECENT_DAYS = 14

def load(path, repo):
    issues = []
    if not path.exists():
        return issues
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            rec = json.loads(line)
        except json.JSONDecodeError:
            continue
        if rec.get("_type") not in (None, "issue"):
            continue
        rec["_repo"] = repo
        issues.append(rec)
    return issues

def dep_ids(issue):
    out = []
    for d in issue.get("dependencies") or []:
        if isinstance(d, str):
            out.append(d)
        elif isinstance(d, dict):
            out.append(d.get("depends_on") or d.get("depends_on_id") or d.get("id"))
    return [d for d in out if d]

def when(issue, field):
    v = issue.get(field)
    if not v:
        return None
    try:
        return datetime.fromisoformat(v.replace("Z", "+00:00"))
    except ValueError:
        return None

def fmt(issue, done=False):
    box = "x" if done else " "
    pri = issue.get("priority")
    pri = f"P{pri}" if pri is not None else "P?"
    typ = issue.get("issue_type", "task")
    labels = " ".join(f"#{l.replace(':', '/')}" for l in issue.get("labels") or [])
    title = (issue.get("title") or "(untitled)").replace("|", "-")
    return f"- [{box}] **{pri}** `{issue['id']}` {title} *({typ}, {issue['_repo']})* {labels}".rstrip()

def section(header, items, empty="*(none)*"):
    lines = [f"## {header}", ""]
    lines += [fmt(i, done=(i.get('status') == 'closed')) for i in items] or [empty]
    lines.append("")
    return lines

def prisort(issues):
    return sorted(issues, key=lambda i: (i.get("priority") if i.get("priority") is not None else 9,
                                         i.get("created_at") or ""))

def main():
    all_issues = []
    for repo, path in REPOS.items():
        all_issues.extend(load(path, repo))
    by_id = {i["id"]: i for i in all_issues}
    now = datetime.now(timezone.utc)
    stamp = now.strftime("%Y-%m-%d %H:%M UTC")

    is_open = lambda i: i.get("status") not in ("closed",)
    open_issues = [i for i in all_issues if is_open(i)]
    in_progress = [i for i in open_issues if i.get("status") == "in_progress"]
    blocked = []
    ready = []
    for i in open_issues:
        if i.get("status") in ("in_progress", "hooked", "deferred"):
            continue
        open_deps = [d for d in dep_ids(i) if d in by_id and is_open(by_id[d])]
        (blocked if open_deps or i.get("status") == "blocked" else ready).append(i)
    recent = [i for i in all_issues if i.get("status") == "closed"
              and (w := when(i, "closed_at")) and now - w < timedelta(days=RECENT_DAYS)]
    recent.sort(key=lambda i: i.get("closed_at") or "", reverse=True)

    VAULT.mkdir(parents=True, exist_ok=True)
    (VAULT / "Sprints").mkdir(exist_ok=True)

    # Board
    lines = [f"# PedalKernel Board", "",
             f"*Generated {stamp} from beads (`bd`) — do not edit; edit beads instead.*", "",
             f"**Open: {len(open_issues)}** · in progress: {len(in_progress)} · "
             f"ready: {len(ready)} · blocked: {len(blocked)} · closed last {RECENT_DAYS}d: {len(recent)}", ""]
    lines += section("🔨 In Progress", prisort(in_progress))
    lines += section("🟢 Ready (unblocked, by priority)", prisort(ready)[:20])
    lines += section("🔒 Blocked", prisort(blocked))
    lines += section(f"✅ Closed (last {RECENT_DAYS} days)", recent[:25])
    (VAULT / "Board.md").write_text("\n".join(lines))

    # Backlog
    lines = [f"# PedalKernel Backlog", "",
             f"*Generated {stamp}. Full open set, priority-ordered.*", ""]
    by_pri = defaultdict(list)
    for i in open_issues:
        by_pri[i.get("priority") if i.get("priority") is not None else 9].append(i)
    for pri in sorted(by_pri):
        label = f"P{pri}" if pri != 9 else "Unprioritized"
        lines += section(f"{label} ({len(by_pri[pri])})", prisort(by_pri[pri]))
    (VAULT / "Backlog.md").write_text("\n".join(lines))

    # Sprints
    sprints = defaultdict(list)
    for i in all_issues:
        for l in i.get("labels") or []:
            if l.startswith("sprint:"):
                sprints[l.split(":", 1)[1]].append(i)
    cur = now.strftime("%G-W%V")
    for name, items in sprints.items():
        done = [i for i in items if i.get("status") == "closed"]
        active = "**← current sprint**" if name == cur else ""
        lines = [f"# Sprint {name} {active}", "",
                 f"*Generated {stamp}. {len(done)}/{len(items)} done.*", ""]
        lines += section("In flight", prisort([i for i in items if is_open(i)]))
        lines += section("Done", done)
        (VAULT / "Sprints" / f"{name}.md").write_text("\n".join(lines))
    if cur not in sprints:
        (VAULT / "Sprints" / f"{cur}.md").write_text(
            f"# Sprint {cur} **← current sprint**\n\n*Empty — pull work in with "
            f"`bd label add <id> sprint:{cur}`, then re-sync.*\n")

    print(f"vault updated: {len(open_issues)} open, {len(sprints)} sprint(s), board+backlog written")

if __name__ == "__main__":
    sys.exit(main())
