"""Overlay recorded agent messages using the video's simulation clock."""
from __future__ import annotations

import argparse
import html
import json
from pathlib import Path
import subprocess

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont


COLORS = {"r1": "#ffdc28", "r2": "#3ca5ff", "r3": "#ff6478"}


def font(size):
    for path in ("/System/Library/Fonts/AppleSDGothicNeo.ttc",
                 "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"):
        if Path(path).exists():
            return ImageFont.truetype(path, size)
    raise RuntimeError("A TrueType font is required to render dialogue")


def wrap_text(text, face, width):
    # Character wrapping also handles long cargo IDs and Korean without spaces.
    lines = []
    for paragraph in text.splitlines() or [""]:
        line = ""
        for char in paragraph:
            if line and face.getlength(line + char) > width:
                split = line.rfind(" ")
                if split > len(line) // 2:
                    lines.append(line[:split]); line = line[split + 1:] + char
                else:
                    lines.append(line); line = char
            else:
                line += char
        lines.append(line)
    return lines


def dialogue_panel(events, width=1280, height=360):
    panel = Image.new("RGB", (width, height), "#101722")
    draw = ImageDraw.Draw(panel)
    heading, body, label = font(23), font(22), font(19)
    translated = any(event.get("message_ko") for event in events)
    title = "로봇 간 대화  ·  실제 발화의 한국어 번역" if translated else "로봇 간 대화  ·  실제 에이전트 발화 원문"
    draw.text((24, 12), title, font=heading, fill="#f0f4fa")
    draw.text((width-240, 17), f"{len(events):02d}개 메시지  |  SIM 1×", font=label, fill="#9fafc2")
    if not events:
        draw.text((24, 95), "첫 메시지를 기다리는 중", font=body, fill="#9fafc2")
        return cv2.cvtColor(np.asarray(panel), cv2.COLOR_RGB2BGR)
    cards, used = [], 0
    for event in reversed(events):
        message = event.get("message_ko") or event["message"]
        lines = wrap_text(message, body, width-76)
        card_height = 33 + len(lines)*27 + 12
        if used + card_height > height-62:
            if cards:
                break
            # Preserve the complete newest message if a policy returns a long reply.
            for size in range(21, 9, -1):
                body = font(size)
                lines = wrap_text(message, body, width-76)
                card_height = 45 + len(lines)*(size+5)
                if card_height <= height-62:
                    break
            if card_height > height-62:
                raise ValueError("Message exceeds panel capacity; enlarge the dialogue panel")
        cards.append((event, lines, card_height, body))
        used += card_height
    y = 56
    for event, lines, card_height, face in reversed(cards):
        color = COLORS.get(event["robot_id"], "#a9bacf")
        draw.rounded_rectangle((20, y, width-20, y+card_height-5), radius=8, fill="#1c2838")
        draw.rectangle((20, y+9, 24, y+card_height-14), fill=color)
        recipients = ", ".join(r.upper() for r in event.get("recipients", [r for r in COLORS if r != event["robot_id"]]))
        draw.text((38, y+5), f'{event["robot_id"].upper()} → {recipients}   ·   {event["video_time"]:.1f}s', font=label, fill=color)
        for index, line in enumerate(lines):
            draw.text((38, y+31+index*(face.size+5)), line, font=face, fill="#f1f5fb")
        y += card_height
    return cv2.cvtColor(np.asarray(panel), cv2.COLOR_RGB2BGR)


def write_transcript(output_path, events):
    data = json.dumps(events, ensure_ascii=False).replace("<", "\\u003c")
    video_name = html.escape(output_path.name, quote=True)
    page = '''<!doctype html><html lang="ko"><meta charset="utf-8">
<title>로봇 대화와 운반 영상</title><style>
body{margin:0;background:#101722;color:#f1f5fb;font:17px system-ui}main{max-width:1440px;margin:auto;padding:24px}
h1{font-size:24px}p{color:#a9bacf}.layout{display:grid;grid-template-columns:3fr 2fr;gap:24px}
video{width:100%;position:sticky;top:20px}#messages{max-height:85vh;overflow:auto}
button{display:block;text-align:left;width:100%;background:#1c2838;color:inherit;border:2px solid transparent;border-radius:10px;padding:16px;margin:0 0 12px;font:inherit;cursor:pointer}
button.active{border-color:#fff}button.future{opacity:.45}b{display:block;margin-bottom:8px}.message{white-space:pre-wrap}
@media(max-width:850px){.layout{display:block}#messages{max-height:none}}</style>
<main><h1>로봇 간 실제 대화</h1><p>대화를 누르면 해당 시점으로 이동합니다. 시간은 영상 시작 기준 시뮬레이션 시간입니다.</p>
<p id="translation-note" hidden>실제 발화를 한국어로 번역했습니다. <label><input id="original" type="checkbox"> 영어 원문 보기</label></p>
<div class="layout"><div><video id="video" controls src="VIDEO"></video></div><div id="messages"></div></div></main>
<script>const events=DATA, colors={r1:'#ffdc28',r2:'#3ca5ff',r3:'#ff6478'};
const video=document.getElementById('video'), box=document.getElementById('messages');
const buttons=events.map(e=>{const b=document.createElement('button'),h=document.createElement('b'),m=document.createElement('div');
h.textContent=e.robot_id.toUpperCase()+' → '+(e.recipients||Object.keys(colors).filter(r=>r!==e.robot_id)).map(r=>r.toUpperCase()).join(', ')+' · '+e.video_time.toFixed(2)+'s';h.style.color=colors[e.robot_id];
m.textContent=e.message_ko||e.message;m.className='message';b.append(h,m);b.onclick=()=>{video.currentTime=e.video_time;};box.append(b);return b;});
document.getElementById('translation-note').hidden=!events.some(e=>e.message_ko);
document.getElementById('original').onchange=e=>buttons.forEach((b,i)=>{b.querySelector('.message').textContent=e.target.checked?events[i].message:(events[i].message_ko||events[i].message);});
video.ontimeupdate=()=>{let current=-1;events.forEach((e,i)=>{if(e.video_time<=video.currentTime)current=i;});buttons.forEach((b,i)=>{b.classList.toggle('active',i===current);b.classList.toggle('future',i>current);});};video.ontimeupdate();</script></html>'''
    output_path.with_suffix(".html").write_text(page.replace("VIDEO", video_name).replace("DATA", data), encoding="utf-8")


def render_dialogue_video(video_path, dialogue_path, output_path):
    video_path, dialogue_path, output_path = map(Path, (video_path, dialogue_path, output_path))
    events = sorted((json.loads(line) for line in dialogue_path.read_text().splitlines() if line.strip()), key=lambda e: e["video_time"])
    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open {video_path}")
    fps = cap.get(cv2.CAP_PROP_FPS)
    width, height = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    process = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "bgr24", "-s", f"{width}x{height+360}", "-r", str(fps), "-i", "-", "-an", "-c:v", "libx264", "-preset", "veryfast", "-crf", "20", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(output_path)], stdin=subprocess.PIPE)
    count, index, cached_index = 0, 0, -1
    try:
        while True:
            ok, frame = cap.read()
            if not ok: break
            while index < len(events) and events[index]["video_time"] <= count/fps+1e-9:
                index += 1
            if index != cached_index:
                panel = dialogue_panel(events[:index], width)
                cached_index = index
            process.stdin.write(np.vstack((frame, panel)).tobytes())
            count += 1
    finally:
        cap.release()
        process.stdin.close()
        code = process.wait(timeout=60)
    if code or not count:
        raise RuntimeError("Dialogue video encoding failed")
    if events and events[-1]["video_time"] > count/fps:
        raise ValueError("Dialogue event occurs after the video ends")
    write_transcript(output_path, events)
    print(f"DIALOGUE_VIDEO {output_path} | {count} frames | {len(events)} real messages", flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--video", required=True)
    parser.add_argument("--dialogue", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    render_dialogue_video(args.video, args.dialogue, args.output)
