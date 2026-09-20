"""Stream aligned existing observer videos, retaining raw SIM timestamps."""
import argparse,bisect,hashlib,io,json,math,os,re,subprocess
from pathlib import Path
import cv2,numpy as np
from PIL import Image,ImageDraw,ImageFont
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--out',type=Path,required=True)
parser.add_argument('--dest',type=Path,required=True)
parser.add_argument('--font',type=Path,required=True)
parser.add_argument('--cases',nargs='+',default=['open-minus','turn-minus'])
args=parser.parse_args()
OUT=args.out.resolve();DEST=args.dest.resolve();DEST.mkdir(parents=True,exist_ok=True)
FONT=str(args.font);large=ImageFont.truetype(FONT,30);small=ImageFont.truetype(FONT,25)
cv2.setNumThreads(1)
def read(p):return json.loads(p.read_text())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def timestamp_index(folder,cache):
 p=folder/'execution.mp4'
 if cache.exists():
  r=read(cache);assert r['sha256']==sha(p);return r
 cap=cv2.VideoCapture(str(p));times=[];texts=[]
 while True:
  ok,im=cap.read()
  if not ok:break
  _,buf=cv2.imencode('.png',im[:54])
  out=subprocess.run(['tesseract','stdin','stdout','--psm','7'],input=buf.tobytes(),stdout=subprocess.PIPE,stderr=subprocess.PIPE,check=True,env={**os.environ,'OMP_THREAD_LIMIT':'1'})
  line=out.stdout.decode().strip();m=re.search(r't\s*=\s*-?(\d+\.\d+)\s*s',line)
  if not m:raise ValueError((p,len(times),line))
  t=float(m.group(1));assert not times or 0<=t-times[-1]<=.6
  times.append(t);texts.append(line)
 cap.release();samples=[json.loads(l) for l in (folder/'referee-only.jsonl').open()]
 assert abs(times[0]-samples[0]['sim_time_s'])<.3 and abs(times[-1]-samples[-1]['sim_time_s'])<.3
 r={'path':str(p),'sha256':sha(p),'frames':len(times),'times':times,'ocr':texts}
 cache.write_text(json.dumps(r,indent=2)+'\n');return r

audit=read(OUT/'audit.json');manifest={'playback_speed':4,'alignment':'Original SIM timestamp OCR, validated against referee clock; stream sample-and-hold, no interpolation. Each source freezes after beam release or actual end, with a label. Cropped to loaded carry; original complete videos retained.','cases':[]}
for case in args.cases:
 specs=[('교사','teacher'),('ACT · seed18','20260918'),('ACT · seed19','20260919')];sources=[]
 for title,cond in specs:
  folder=OUT/'final'/cond/case;meta=timestamp_index(folder,DEST/(case+'-'+cond+'-times.json'));row=next(r for r in audit['rows'] if r['phase']=='final' and r['case']==case and r['condition']==cond)
  cmds=read(folder/'issued-commands.json');release=[c['issued_at_s']+c.get('duration_s',0)+c.get('settle_s',0) for cc in cmds.values() for c in cc if c['stage']=='place_retract']
  end=min(meta['times'][-1],max(release)+1.3) if release else meta['times'][-1]
  sources.append({'title':title,'cond':cond,'meta':meta,'row':row,'end':end,'cap':cv2.VideoCapture(meta['path']),'index':-1,'frame':None})
 starts=[s['row']['carry_window'][0] for s in sources if s['row']['carry_window']];start=max(0,min(starts)-.5) if starts else 0;end=max(s['end'] for s in sources);fps=12;count=math.ceil((end-start)*fps/4)+13
 path=DEST/(case+'-comparison-4x.mp4');proc=subprocess.Popen(['ffmpeg','-y','-v','error','-f','rawvideo','-pix_fmt','rgb24','-s','1920x700','-r',str(fps),'-i','-','-an','-c:v','libx264','-threads','2','-preset','veryfast','-crf','20','-pix_fmt','yuv420p','-movflags','+faststart',str(path)],stdin=subprocess.PIPE)
 qa=[];qa_timestamps=[]
 try:
  for n in range(count):
   now=min(end,start+n*4/fps);frame=Image.new('RGB',(1920,700),'#101820');draw=ImageDraw.Draw(frame)
   draw.text((20,10),f'{case} | 실제 파지 이후 공동 운반 | 같은 SIM 시각 · 4배속',font=large,fill='white')
   for i,s in enumerate(sources):
    until=min(now,s['end']);idx=max(0,bisect.bisect_right(s['meta']['times'],until)-1)
    while s['index']<idx:
     ok,bgr=s['cap'].read();assert ok;s['index']+=1;s['frame']=cv2.cvtColor(bgr,cv2.COLOR_BGR2RGB)
    x=640*i;success=s['row']['beam_success'];status='빔 성공' if success else '빔 실패'
    if not s['row']['carry_entered']:status='운반 진입 전 실패'
    draw.text((x+15,58),f"{s['title']} | {status}",font=large,fill='#78dcaf' if success else '#ff9090')
    draw.text((x+15,96),f"t = {s['meta']['times'][s['index']]:.2f} s",font=small,fill='white')
    frame.paste(Image.fromarray(s['frame']).resize((640,480)),(x,130))
    if now>s['end']+.05:ImageDraw.Draw(frame).text((x+16,615),'구간 종료 · 마지막 화면 유지',font=small,fill='#ffcf78',stroke_width=2,stroke_fill='black')
   draw=ImageDraw.Draw(frame);draw.text((20,656),f'SIM {now:.1f}s | 원본 4fps · 영상/평가 좌표는 ACT 입력이 아님 | 계획·파지·방출은 기존 코드',font=small,fill='white')
   proc.stdin.write(np.asarray(frame).tobytes())
   if n in {0,count//4,count//2,count-1}:
    q=DEST/(case+f'-qa-{n:04}.jpg');frame.save(q);qa.append(str(q))
    for i,src in enumerate(sources):
     buf=io.BytesIO();Image.fromarray(src['frame']).crop((0,0,src['frame'].shape[1],54)).save(buf,format='PNG')
     observed=subprocess.run(['tesseract','stdin','stdout','--psm','7'],input=buf.getvalue(),stdout=subprocess.PIPE,stderr=subprocess.PIPE,check=True,env={**os.environ,'OMP_THREAD_LIMIT':'1'}).stdout.decode().strip()
     match=re.search(r't\s*=\s*-?(\d+[.,]\d+)\s*s',observed)
     assert match,(q,src['cond'],observed)
     timestamp=float(match.group(1).replace(',','.'));expected=src['meta']['times'][src['index']]
     assert abs(timestamp-expected)<.011,(q,src['cond'],timestamp,expected)
     qa_timestamps.append({'frame':n,'condition':src['cond'],'observed_sim_s':timestamp,'expected_sim_s':expected,'raw_frame_index':src['index'],'verification':'Original selected frame header OCR before display resize; large panel time uses same timestamp.'})
 finally:proc.stdin.close()
 assert proc.wait(timeout=60)==0
 for s in sources:s['cap'].release()
 subprocess.run(['ffmpeg','-v','error','-i',str(path),'-f','null','-'],check=True)
 manifest['cases'].append({'case':case,'path':str(path),'sha256':sha(path),'duration_s':count/fps,'start_sim_s':start,'end_sim_s':end,'sources':[{'condition':s['cond'],'video_sha256':s['meta']['sha256'],'source':s['meta']['path'],'freeze_sim_s':s['end']} for s in sources],'qa':qa,'qa_timestamp_checks':qa_timestamps,'decoded_entire_video':True})
 (DEST/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');print(json.dumps(manifest['cases'][-1]),flush=True)
