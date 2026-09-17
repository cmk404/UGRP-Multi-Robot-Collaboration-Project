"""Crop the same observer camera region and preserve one-times playback."""
from pathlib import Path
import json,sys,cv2,subprocess,math
from PIL import Image,ImageDraw,ImageFont
old,new,dest=map(Path,sys.argv[1:]);dest.mkdir(parents=True,exist_ok=True)
font='/System/Library/Fonts/AppleSDGothicNeo.ttc'
caps=[cv2.VideoCapture(str(p/'motion.mp4')) for p in [old,new]]
metrics=[]
for root in [old,new]:
 rows=[json.loads(x) for x in (root/'evaluation-only.jsonl').read_text().splitlines()]
 row=[r for r in rows if r['phase']=='grasp_hold'][-1]
 metrics.append(math.dist(row['bases']['r1'][:2],row['bases']['r3'][:2])*100)
raw=dest/'comparison-unencoded.mp4';w=cv2.VideoWriter(str(raw),cv2.VideoWriter_fourcc(*'mp4v'),4,(800,730))
selected=[]
for frame in range(52,91):
 canvas=Image.new('RGB',(800,730),'#111b2b');d=ImageDraw.Draw(canvas)
 for k,c in enumerate(caps):
  c.set(cv2.CAP_PROP_POS_FRAMES,frame);ok,im=c.read();assert ok
  crop=Image.fromarray(cv2.cvtColor(im,cv2.COLOR_BGR2RGB)).crop((480,260,640,480)).resize((400,550))
  canvas.paste(crop,(400*k,94))
  d.text((20+400*k,16),'수정 전 · 1500 PWM' if k==0 else '수정 후 · 1600 PWM',font=ImageFont.truetype(font,26),fill='white')
  d.text((20+400*k,54),f'대기 종료 간격 {metrics[k]:.1f} cm',font=ImageFont.truetype(font,22),fill='#ffb4a7' if k==0 else '#9fe1bf')
 d.text((22,661),'같은 카메라 영역 확대 · 실제 대기 영상 1배속',font=ImageFont.truetype(font,24),fill='white')
 d.text((22,697),'평가용 거리 표기 · 제어 입력 아님',font=ImageFont.truetype(font,18),fill='#b7cce6')
 w.write(cv2.cvtColor(__import__('numpy').array(canvas),cv2.COLOR_RGB2BGR))
 if frame in [52,72,90]:
  canvas.save(dest/f'comparison-frame-{frame}.jpg',quality=93);selected.append(frame)
w.release()
for c in caps:c.release()
subprocess.run(['ffmpeg','-y','-v','error','-i',str(raw),'-c:v','libx264','-pix_fmt','yuv420p','-movflags','+faststart',str(dest/'grasp-spacing-before-after.mp4')],check=True)
raw.unlink()
(dest/'comparison-source.json').write_text(json.dumps({'before':str(old),'after':str(new),'frames_0based_inclusive':[52,90],'fps':4,'playback_speed':1,'observer_crop_xyxy':[480,260,640,480],'final_spacing_cm':metrics,'scope':'Selected fixed post-lift hold interval; metric labels are output-only'},indent=2)+'\n')
