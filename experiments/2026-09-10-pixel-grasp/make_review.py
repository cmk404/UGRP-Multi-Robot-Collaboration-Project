"""Output-only side-by-side RGB evidence; optional fitted points are annotations."""
from pathlib import Path
import json,sys
from PIL import Image,ImageDraw
root=Path(sys.argv[1]);out=Path(sys.argv[2]);rounds=[int(v) for v in sys.argv[3:]]
r=json.loads((root/'result.json').read_text());calls={c['round']:c for c in r['calls'] if c['robot_id']=='r1'}
canvas=Image.new('RGB',(1280,520*len(rounds)),'white');d=ImageDraw.Draw(canvas)
for i,n in enumerate(rounds):
 c=calls[n];y0=i*520
 for j,key in enumerate(['own','top']):
  im=Image.open(root/c['images'][key]['path']);im.thumbnail((640,480));canvas.paste(im,(640*j,y0+35));d.text((640*j+8,y0+8),f'{root.name} / r{n} / {key} RGB',fill='black')
 a=c['observation'].get('alignment');g=c['observation']['gripper']
 if a and g.get('center'):
  for point,color in [(a['target'],'red'),(g['center'],'cyan')]:
   x,y=640+point[0]*640,y0+35+point[1]*480;d.ellipse((x-5,y-5,x+5,y+5),outline=color,width=2)
canvas.save(out)
