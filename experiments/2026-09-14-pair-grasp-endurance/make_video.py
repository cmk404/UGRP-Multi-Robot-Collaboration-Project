"""Crop only the observer recording; keep source time/height overlay visible."""
from pathlib import Path
import cv2,json,numpy as np,subprocess
root=Path('outputs/pair-endurance-baseline-2026-09-14');out=Path('outputs/endurance-artifacts/media');out.mkdir(parents=True,exist_ok=True)
tmp=out/'failure-review-silent.mp4';writer=cv2.VideoWriter(str(tmp),cv2.VideoWriter_fourcc(*'mp4v'),16,(960,720));selections=[];tiles=[]
for mode in ('stationary','shuttle'):
 cap=cv2.VideoCapture(str(root/mode/'motion.mp4'));fps=cap.get(cv2.CAP_PROP_FPS);n=int(cap.get(cv2.CAP_PROP_FRAME_COUNT));indices=[round(x*fps) for x in (22.5,60,88)]+[n-1]
 count=0
 while True:
  ok,frame=cap.read()
  if not ok:break
  canvas=np.full((720,960,3),(24,26,30),np.uint8)
  cv2.putText(canvas,f'{mode.upper()} | 4x recording | baseline configuration',(22,35),cv2.FONT_HERSHEY_SIMPLEX,.73,(245,245,245),2,cv2.LINE_AA)
  canvas[50:104]=frame[:54]
  zoom=cv2.resize(frame[170:570,380:780],(600,600));canvas[110:710,:600]=zoom
  canvas[130:400,600:960]=cv2.resize(frame,(360,270))
  for y,line in zip((450,483,516,560,590,620),('300 s planned','Stopped near 94 s','Carry silhouette changed','Observer evidence only','No weld; zero LLM calls','Original time shown above')):
   cv2.putText(canvas,line,(613,y),cv2.FONT_HERSHEY_SIMPLEX,.48,(220,224,231),1,cv2.LINE_AA)
  writer.write(canvas)
  if count in indices:
   name=f'{mode}-observer-frame-{count:04d}.jpg';cv2.imwrite(str(out/name),canvas);selections.append({'mode':mode,'source_video':str((root/mode/'motion.mp4').resolve()),'frame_index':count,'source_video_seconds':count/fps,'selected_image':name});tiles.append(cv2.resize(canvas,(480,360)))
  count+=1
 cap.release()
writer.release();subprocess.run(['ffmpeg','-v','error','-y','-i',str(tmp),'-c:v','libx264','-preset','fast','-crf','22','-pix_fmt','yuv420p','-movflags','+faststart',str(out/'endurance-failure-review.mp4')],check=True);tmp.unlink()
cv2.imwrite(str(out/'observer-review-overview.jpg'),np.concatenate([np.concatenate(tiles[i:i+4],axis=1) for i in range(0,len(tiles),4)],axis=0));(out/'observer-selections.json').write_text(json.dumps(selections,indent=2)+'\n');print(len(selections),'selected frames')
