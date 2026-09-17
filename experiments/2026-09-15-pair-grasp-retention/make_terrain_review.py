"""Assemble actual full-view terrain recordings and sampled visual checks."""
from pathlib import Path
import argparse,cv2,json,numpy as np,subprocess

def main():
 p=argparse.ArgumentParser();p.add_argument('--raw',type=Path,required=True);p.add_argument('--out',type=Path,required=True);a=p.parse_args();a.out.mkdir(parents=True,exist_ok=True)
 names=('narrow-door','l-corner','s-bends','staggered-obstacles','blocked-branch','fully-blocked');selections=[];tiles=[];tmp=a.out/'terrain-silent.mp4';writer=cv2.VideoWriter(str(tmp),cv2.VideoWriter_fourcc(*'mp4v'),16,(960,720))
 for name in names:
  source=a.raw/name/'motion.mp4';cap=cv2.VideoCapture(str(source));fps=cap.get(cv2.CAP_PROP_FPS);n=int(cap.get(cv2.CAP_PROP_FRAME_COUNT));indices={round(x*(n-1)) for x in (.3,.65,1.)};count=0
  while True:
   ok,frame=cap.read()
   if not ok:break
   cv2.rectangle(frame,(0,650),(960,720),(24,26,30),-1);cv2.putText(frame,name+' | retention profile | 4x',(18,684),cv2.FONT_HERSHEY_SIMPLEX,.72,(245,245,245),2,cv2.LINE_AA)
   writer.write(frame)
   if count in indices:
    filename=f'{name}-frame-{count:04d}.jpg';cv2.imwrite(str(a.out/filename),frame);tiles.append(cv2.resize(frame,(480,360)));selections.append({'source_video':str(source.resolve()),'source_frame':count,'source_fps':fps,'selected_image':filename})
   count+=1
  cap.release()
 writer.release();subprocess.run(['ffmpeg','-v','error','-y','-i',str(tmp),'-c:v','libx264','-preset','fast','-crf','22','-pix_fmt','yuv420p','-movflags','+faststart',str(a.out/'terrain-retention.mp4')],check=True);tmp.unlink()
 for i in range(0,6,2):
  chosen=tiles[i*3:(i+2)*3];cv2.imwrite(str(a.out/f'terrain-review-{i//2+1}.jpg'),np.concatenate([np.concatenate(chosen[j:j+3],axis=1) for j in (0,3)],axis=0))
 (a.out/'terrain-selections.json').write_text(json.dumps(selections,indent=2)+'\n')
if __name__=='__main__':main()
