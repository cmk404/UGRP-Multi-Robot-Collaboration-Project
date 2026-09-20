"""Output-only observer contact sheets for visible motion QA."""
from pathlib import Path
import cv2,json,sys
from PIL import Image,ImageDraw,ImageFont
raw,dest=map(Path,sys.argv[1:]);dest.mkdir(parents=True,exist_ok=True)
font=ImageFont.truetype('/System/Library/Fonts/AppleSDGothicNeo.ttc',19)
review=[]
for case in json.loads((raw/'cohort.json').read_text())['trials']:
 p=raw/case['id'];r=json.loads((p/'result.json').read_text());v=cv2.VideoCapture(str(p/'motion.mp4'));n=int(v.get(cv2.CAP_PROP_FRAME_COUNT));fps=v.get(cv2.CAP_PROP_FPS)
 frames=[90,90+(n-91)//3,90+2*(n-91)//3,n-1]
 sheet=Image.new('RGB',(960,792),'#101b2b');d=ImageDraw.Draw(sheet)
 for i,idx in enumerate(frames):
  v.set(cv2.CAP_PROP_POS_FRAMES,idx);ok,im=v.read();assert ok
  x=i%2*480;y=i//2*396
  sheet.paste(Image.fromarray(cv2.cvtColor(im,cv2.COLOR_BGR2RGB)).resize((480,360)),(x,y+36))
  d.text((x+8,y+6),f'{case["id"]} | frame {idx}',font=font,fill='white')
 v.release();sheet.save(dest/(case['id']+'-observer-review.jpg'),quality=91)
 review.append({'id':case['id'],'observer_video':str(p/'motion.mp4'),'frames_0based':frames,'fps':fps,'frame_count':n})
(dest/'observer-review-selection.json').write_text(json.dumps(review,indent=2)+'\n')
