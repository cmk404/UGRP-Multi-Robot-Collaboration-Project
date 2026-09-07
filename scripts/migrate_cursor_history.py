#!/usr/bin/env python3
"""Migrate Cursor agent transcripts for the UGRP project into a sanitized project archive."""
from __future__ import annotations

import json
import re
import shutil
from datetime import datetime
from pathlib import Path

PROJECT = Path('/Users/changmin/projects/ugrp')
CURSOR_UGRP = Path('/Users/changmin/.cursor/projects/Users-changmin-projects-ugrp/agent-transcripts')
CURSOR_MAIN = Path('/Users/changmin/.cursor/projects/Users-changmin-projects-main/agent-transcripts')
OUT = PROJECT / 'docs' / 'cursor-history'
RAW = OUT / 'sanitized-jsonl'
SESSIONS = OUT / 'sessions'

# Known continuation referenced by the latest UGRP session.
EXTRA_SESSION_IDS = {'58cded44-1f86-4885-8bae-11eaf6743412'}

SECRET_PATTERNS = [
    # Groq keys seen in a UGRP transcript.
    (re.compile(r'gsk_[A-Za-z0-9_-]{20,}'), 'gsk_[REDACTED]'),
    # Common OpenAI-style keys.
    (re.compile(r'sk-(?:proj-)?[A-Za-z0-9_-]{20,}'), 'sk-[REDACTED]'),
    # Bearer tokens in copied headers/logs.
    (re.compile(r'(?i)(Authorization\s*:\s*Bearer\s+)[A-Za-z0-9._~+/=-]{16,}'), r'\1[REDACTED]'),
]


def redact(text: str) -> str:
    for pat, repl in SECRET_PATTERNS:
        text = pat.sub(repl, text)
    return text


def session_files() -> list[Path]:
    found = list(CURSOR_UGRP.glob('*/*.jsonl'))
    for sid in EXTRA_SESSION_IDS:
        found.extend((CURSOR_MAIN / sid).glob('*.jsonl'))
    # Deduplicate by resolved path and order chronologically by mtime.
    unique = {p.resolve(): p for p in found}
    return sorted(unique.values(), key=lambda p: p.stat().st_mtime)


def content_text(message: dict) -> str:
    chunks = []
    for item in message.get('content', []) if isinstance(message, dict) else []:
        if item.get('type') == 'text':
            chunks.append(str(item.get('text', '')))
    return '\n'.join(chunks).strip()


def clean_user_text(text: str) -> str:
    m = re.search(r'<user_query>\n?(.*?)\n?</user_query>', text, re.S)
    if m:
        text = m.group(1).strip()
    text = re.sub(r'^<timestamp>.*?</timestamp>\s*', '', text, flags=re.S)
    return text.strip()


def short(text: str, n: int = 110) -> str:
    text = re.sub(r'\s+', ' ', text).strip()
    return text if len(text) <= n else text[: n - 1] + '…'


def infer_title(user_messages: list[str]) -> str:
    meaningful = []
    for u in user_messages:
        if u.startswith('Briefly inform the user') or u.startswith('<dynamic_tools>'):
            continue
        if u.startswith('Implement the plan as specified'):
            continue
        meaningful.append(u)
    if not meaningful:
        return 'Cursor UGRP session'
    first = meaningful[0]
    # Hand-tuned recognizable topics for the recent sessions, otherwise use first prompt.
    joined = '\n'.join(meaningful).lower()
    if 'sd 부팅' in joined or ('act' in joined and '3b' in joined):
        return 'Raspberry Pi 3B / SD boot diagnosis'
    if '빨간색 블럭' in joined or 'red_skills.py' in joined or '트래킹' in joined:
        return 'MasterPi red-block control and tracking'
    if 'llm' in joined and ('하네스' in joined or 'chat' in joined or 'groq' in joined):
        return 'Research robot-agent harness and browser UI'
    if '카메라 좀 꺼' in joined:
        return 'Cursor Mac camera cleanup'
    if 'qwen3-vl' in joined or 'gemma4' in joined:
        return 'Local VLM setup for robot vision'
    return short(first, 80)


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    RAW.mkdir(parents=True, exist_ok=True)
    SESSIONS.mkdir(parents=True, exist_ok=True)

    index_rows = []
    for src in session_files():
        sid = src.stem
        mtime = datetime.fromtimestamp(src.stat().st_mtime).astimezone()
        sanitized_lines = []
        user_messages: list[str] = []
        assistant_messages: list[str] = []
        tool_names: list[str] = []

        with src.open('r', encoding='utf-8', errors='replace') as fh:
            for line in fh:
                safe_line = redact(line.rstrip('\n'))
                sanitized_lines.append(safe_line)
                try:
                    obj = json.loads(safe_line)
                except json.JSONDecodeError:
                    continue
                role = obj.get('role')
                if role in {'user', 'assistant'}:
                    text = content_text(obj.get('message', {}))
                    if text:
                        text = redact(text)
                        if role == 'user':
                            user_messages.append(clean_user_text(text))
                        else:
                            assistant_messages.append(text)
                    for item in obj.get('message', {}).get('content', []):
                        if item.get('type') == 'tool_use' and item.get('name'):
                            tool_names.append(str(item['name']))

        raw_dest = RAW / f'{sid}.jsonl'
        raw_dest.write_text('\n'.join(sanitized_lines) + '\n', encoding='utf-8')

        title = infer_title(user_messages)
        md = [
            f'# {title}',
            '',
            f'- Session ID: `{sid}`',
            f'- Source: `{src}`',
            f'- Source modified: `{mtime.isoformat(timespec="seconds")}`',
            f'- User messages: {len(user_messages)}',
            f'- Assistant text messages: {len(assistant_messages)}',
            f'- Sanitized raw copy: `../sanitized-jsonl/{sid}.jsonl`',
            '',
            '## Conversation',
            '',
        ]
        # Preserve the human-readable conversation; omit system-generated wrapper prompts.
        ui = ai = 0
        events = []
        with raw_dest.open('r', encoding='utf-8', errors='replace') as fh:
            for line in fh:
                try:
                    obj = json.loads(line)
                except json.JSONDecodeError:
                    continue
                role = obj.get('role')
                if role not in {'user', 'assistant'}:
                    continue
                text = content_text(obj.get('message', {}))
                if not text:
                    continue
                if role == 'user':
                    text = clean_user_text(text)
                    if text.startswith('Briefly inform the user') or text.startswith('<dynamic_tools>'):
                        continue
                    ui += 1
                    events.append((f'### User {ui}', text))
                else:
                    # Include only assistant natural-language text; tool calls are preserved in JSONL.
                    ai += 1
                    events.append((f'### Assistant {ai}', text))
        for heading, text in events:
            md += [heading, '', text, '']

        if tool_names:
            md += ['## Tool names used', '', ', '.join(f'`{x}`' for x in sorted(set(tool_names))), '']

        session_md = SESSIONS / f'{mtime.strftime("%Y-%m-%d")}_{sid}.md'
        session_md.write_text('\n'.join(md).rstrip() + '\n', encoding='utf-8')
        index_rows.append((mtime, sid, title, session_md.name, len(user_messages)))

    index_rows.sort(key=lambda x: x[0], reverse=True)
    readme = [
        '# Cursor history migration',
        '',
        'This directory is a project-local archive of Cursor agent activity related to UGRP.',
        '',
        '## Safety',
        '',
        '- The original Cursor files are not modified.',
        '- Migrated JSONL is sanitized before being written here.',
        '- Known API-key/token patterns are replaced with `[REDACTED]`.',
        '- The latest UGRP session referenced one predecessor session from the `main` Cursor workspace; that session is included too.',
        '',
        '## Files',
        '',
        '- `sessions/`: readable Markdown transcripts.',
        '- `sanitized-jsonl/`: sanitized copies that retain tool-call structure.',
        '- `INDEX.md`: newest-first session list.',
        '- `scripts/migrate_cursor_history.py`: rerun the migration after future Cursor work.',
        '',
        '## Refresh',
        '',
        '```bash',
        'cd /Users/changmin/projects/ugrp',
        'python3 scripts/migrate_cursor_history.py',
        '```',
    ]
    (OUT / 'README.md').write_text('\n'.join(readme).rstrip() + '\n', encoding='utf-8')

    idx = [
        '# Cursor UGRP session index', '',
        f'Last migrated: {datetime.now().astimezone().isoformat(timespec="seconds")}', '',
        '| Source modified | Session | Topic | User msgs | Transcript |',
        '|---|---|---|---:|---|',
    ]
    for dt, sid, title, name, nuser in index_rows:
        idx.append(f'| {dt.strftime("%Y-%m-%d %H:%M %z")} | `{sid}` | {title.replace("|", "\\|")} | {nuser} | [open](sessions/{name}) |')
    (OUT / 'INDEX.md').write_text('\n'.join(idx) + '\n', encoding='utf-8')

    print(f'Migrated {len(index_rows)} sessions to {OUT}')
    for dt, sid, title, name, nuser in index_rows:
        print(f'- {dt:%Y-%m-%d %H:%M} {sid} :: {title}')


if __name__ == '__main__':
    main()
