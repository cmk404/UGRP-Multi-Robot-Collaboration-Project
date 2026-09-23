"""Observer-window-only presentation of the peer communication sidecar."""
from __future__ import annotations

from pathlib import Path
import time

import numpy as np

from harness.communication_observer import read_latest

KOREAN_FONTS = (
    '/System/Library/Fonts/AppleSDGothicNeo.ttc',
    '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
    '/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc',
    '/usr/share/fonts/truetype/nanum/NanumGothic.ttf',
)


def _overlay_text(value):
    return ''.join(char if char.isprintable() else f'\\u{ord(char):04x}' for char in str(value))


def render_dialogue_overlay(state, *, font_paths=KOREAN_FONTS):
    """Render recent text for a copied observer, never an actor camera."""
    from PIL import Image, ImageDraw, ImageFont

    font = None
    for path in font_paths:
        try:
            font = ImageFont.truetype(path, 17)
            break
        except OSError:
            continue
    korean_font = font is not None
    if font is None:
        font = ImageFont.load_default()
    width, height = 680, 196
    panel = Image.new('RGB', (width, height), '#14212a')
    draw = ImageDraw.Draw(panel)
    draw.rectangle((0, 0, width-1, height-1), outline='#65a9bc', width=2)
    if korean_font:
        title = f"동료 메시지 관찰 | {state['status']}"
        footer = f"전체 원문: {state.get('full_log', 'team/conversation.jsonl')}"
    else:
        title = f"Peer messages: {state['fresh_peer_messages']} (Korean font unavailable)"
        footer = f"Full UTF-8 text: {state.get('full_log', 'team/conversation.jsonl')}"

    def line(text, y, color):
        text = _overlay_text(text)
        while text and draw.textlength(text, font=font) > width-24:
            text = text[:-1]
        draw.text((12, y), text, fill=color, font=font)

    line(title, 12, '#e5f3f4')
    recent = state.get('recent_messages', [])[-3:]
    if not recent:
        line('새 자연어 메시지 없음' if korean_font else 'No fresh natural-language message',
             52, '#c2d6da')
    for index, message in enumerate(recent):
        arrow = '→' if korean_font else '->'
        prefix = f"#{message['seq']} {message['sender']} {arrow} {','.join(message['recipients'])} "
        line(prefix + (message['text'] if korean_font else '[see full UTF-8 log]'),
             52 + 35*index, '#d7f5e7')
    line(footer, 165, '#91c3cf')
    return np.asarray(panel), korean_font


class ObserverDialoguePanel:
    """At most five small-file polls per second; repaint only for new messages."""

    def __init__(self, latest_path: Path):
        self.latest_path = Path(latest_path)
        self.next_poll = 0.
        self.sequence = -1
        self.font_warning = False
        self.disabled = False

    def poll(self, viewer, mujoco, now=None):
        now = time.monotonic() if now is None else now
        if self.disabled or now < self.next_poll:
            return False
        self.next_poll = now + .2
        try:
            state = read_latest(self.latest_path)
            if state is None or state['seq'] == self.sequence:
                return False
            pixels, korean_font = render_dialogue_overlay(state)
            viewer.set_images((mujoco.MjrRect(12, 12, pixels.shape[1], pixels.shape[0]), pixels))
            self.sequence = state['seq']
            if not korean_font and not self.font_warning:
                print('Korean font unavailable; full UTF-8 dialogue is in '
                      'team/conversation.jsonl and terminal', flush=True)
                self.font_warning = True
            return True
        except ImportError:
            viewer.set_texts((None, None,
                'Peer dialogue overlay unavailable: Pillow is not installed',
                'Full UTF-8 text: team/conversation.jsonl and terminal'))
            self.disabled = True
        except Exception as exc:
            # Presentation must not become a new controller or physics failure.
            print(f'peer dialogue overlay disabled: {type(exc).__name__}: {exc}', flush=True)
            self.disabled = True
        return False
