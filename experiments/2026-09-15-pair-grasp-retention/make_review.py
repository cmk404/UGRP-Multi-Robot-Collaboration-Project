"""Review actual observer recordings and output-only retention measurements."""
from pathlib import Path
import argparse,cv2,json,numpy as np,subprocess

def main():
 p=argparse.ArgumentParser();p.add_argument('--raw',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True)
 cases=('stationary','shuttle','baseline-recovery');selections=[];tiles=[];tmp=a.out/'review-silent.mp4';writer=cv2.VideoWriter(str(tmp),cv2.VideoWriter_fourcc(*'mp4v'),16,(960,720))
 for name in cases:
  source=a.raw/name/'motion.mp4';cap=cv2.VideoCapture(str(source));fps=cap.get(cv2.CAP_PROP_FPS);n=int(cap.get(cv2.CAP_PROP_FRAME_COUNT));indices=set((66,118,190,n-1)) if name=='baseline-recovery' else {round(x*(n-1)) for x in (.08,.55,.98,1.)};count=0
  while True:
   ok,frame=cap.read()
   if not ok:break
   canvas=np.full((720,960,3),(24,26,30),np.uint8)
   cv2.putText(canvas,f'{name.upper()} | 4x observer recording',(20,33),cv2.FONT_HERSHEY_SIMPLEX,.72,(245,245,245),2,cv2.LINE_AA)
   canvas[50:104]=frame[:54]
   canvas[110:710,:600]=cv2.resize(frame[170:570,380:780],(600,600))
   canvas[130:400,600:960]=cv2.resize(frame,(360,270))
   lines=('Two independent RGB actors','Original cameras retained','No weld; zero LLM calls','Numerical contact fix' if name!='baseline-recovery' else 'Old contacts + recovery','Source time shown above','1 regrasp then place/stop' if name=='baseline-recovery' else '300 s + floor release')
   for y,line in zip((450,483,516,559,592,625),lines):cv2.putText(canvas,line,(611,y),cv2.FONT_HERSHEY_SIMPLEX,.43,(220,224,231),1,cv2.LINE_AA)
   writer.write(canvas)
   if count in indices:
    filename=f'{name}-frame-{count:04d}.jpg';cv2.imwrite(str(a.out/filename),canvas);tiles.append(cv2.resize(canvas,(480,360)));selections.append({'source_video':str(source.resolve()),'source_frame':count,'source_fps':fps,'selected_image':filename})
   count+=1
  cap.release()
 writer.release();subprocess.run(['ffmpeg','-v','error','-y','-i',str(tmp),'-c:v','libx264','-preset','fast','-crf','22','-pix_fmt','yuv420p','-movflags','+faststart',str(a.out/'retention-and-recovery.mp4')],check=True);tmp.unlink()
 cv2.imwrite(str(a.out/'observer-review.jpg'),np.concatenate([np.concatenate(tiles[i:i+4],axis=1) for i in range(0,len(tiles),4)],axis=0))
 (a.out/'observer-selections.json').write_text(json.dumps(selections,indent=2)+'\n')
if __name__=='__main__':main()
