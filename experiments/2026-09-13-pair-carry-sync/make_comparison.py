import argparse,cv2,subprocess,pathlib,shutil,numpy as np
parser=argparse.ArgumentParser();parser.add_argument('--out-dir',type=pathlib.Path,required=True);args=parser.parse_args()
root=pathlib.Path(__file__).resolve().parent/'media';out=args.out_dir;out.mkdir(parents=True,exist_ok=False)
a=cv2.VideoCapture(str(root/'departure-r1-100-baseline.mp4'));b=cv2.VideoCapture(str(root/'departure-r1-100-sync.mp4'))
counts=[int(c.get(cv2.CAP_PROP_FRAME_COUNT)) for c in (a,b)];fps=a.get(cv2.CAP_PROP_FPS);assert fps==b.get(cv2.CAP_PROP_FPS)==4
path=out/'r1-delay-comparison.mp4'
proc=subprocess.Popen([shutil.which('ffmpeg') or 'ffmpeg','-v','error','-nostdin','-y','-f','rawvideo','-pix_fmt','bgr24','-s','1280x520','-r',str(fps),'-i','-','-an','-c:v','libx264','-pix_fmt','yuv420p','-crf','22','-movflags','+faststart',str(path)],stdin=subprocess.PIPE)
last=[None,None]
for i in range(max(counts)):
 panels=[]
 for j,c in enumerate((a,b)):
  ok,f=c.read()
  if ok:last[j]=cv2.resize(f,(640,480))
  panel=np.zeros((520,640,3),np.uint8);panel[40:]=last[j]
  label=('BASELINE','PAIR SYNC')[j]
  if i>=counts[j]:label+=' | STOPPED - LAST FRAME'
  cv2.putText(panel,label,(10,27),cv2.FONT_HERSHEY_SIMPLEX,.65,(255,255,255),1,cv2.LINE_AA);panels.append(panel)
 frame=np.hstack(panels);proc.stdin.write(frame.tobytes())
 if i==76:cv2.imwrite(str(out/'r1-delay-comparison-preview.jpg'),frame)
a.release();b.release();proc.stdin.close();assert proc.wait()==0
print(path)
