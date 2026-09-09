"""Create offline trial comparison data and review sheets, never control robots."""
from __future__ import annotations
import argparse,json
from pathlib import Path
import cv2
from PIL import Image,ImageDraw


def sheet(tiles,path):
    canvas=Image.new('RGB',(1440,326*((len(tiles)+2)//3)),'white')
    for i,(label,im) in enumerate(tiles):
        im=im.copy();im.thumbnail((480,300));x=(i%3)*480;y=(i//3)*326
        canvas.paste(im,(x,y+26));ImageDraw.Draw(canvas).text((x+8,y+7),label,fill='black')
    canvas.save(path)


def main():
    ap=argparse.ArgumentParser();ap.add_argument('run',type=Path);args=ap.parse_args();p=args.run
    result=json.loads((p/'result.json').read_text())
    rows=[json.loads(l) for l in (p/'llm-decisions.jsonl').read_text().splitlines()]
    nav=[];calls=[]
    for r in rows:
        if r.get('event')=='llm_request':
            for e in r.get('images',[]):
                if e['camera']=='nav':nav.append((r['call_id'],Image.open(p/e['path'])))
        elif r.get('event')=='llm_result':
            a=r.get('audit') or {};c=a.get('model_context') or {};calls.append({
                'call':r['call_id'],'decision':r.get('decision'),'disposition':r.get('disposition'),
                'tokens':(a.get('usage') or {}).get('prompt_tokens'),'image_count':a.get('image_count'),
                'nav_evidence':c.get('nav_evidence'),'previous_navigation_note':c.get('previous_navigation_note'),
                'navigation_temporal':c.get('navigation_temporal'),
                'navigation_reference':c.get('navigation_reference'),
                'navigation_commitment':c.get('navigation_commitment'),
                'execution_feedback':c.get('execution_feedback')})
    if nav:sheet(nav,p/'nav-qa.jpg')
    cap=cv2.VideoCapture(str(p/'motion-1x.mp4'));n=int(cap.get(cv2.CAP_PROP_FRAME_COUNT));fps=cap.get(cv2.CAP_PROP_FPS)
    wanted=set(round(i*(n-1)/11) for i in range(12));frames=[];count=0
    while True:
        ok,frame=cap.read()
        if not ok:break
        if count in wanted:frames.append((f'{count/fps:.1f}s / frame {count}',Image.fromarray(cv2.cvtColor(frame,cv2.COLOR_BGR2RGB))))
        count+=1
    cap.release()
    if frames:sheet(frames,p/'motion-qa.jpg')
    summary={'success':result['success'],'reason':result['outcomes']['r1']['reason'],
       'calls':result['llm_calls']['r1'],'input_tokens':result['input_usage']['r1']['reported_prompt_tokens'],
       'sim_seconds':result['decision_elapsed_sim_s'],'position_error_m':result['final']['cargo']['small_box_01']['evaluation']['position_error_m'],
       'gates':result['outcomes']['r1']['gates'],'guard_rejections':sum(r['disposition']=='rejected_guard' for r in calls),
       'navigation_notes':sum(bool((r['decision'] or {}).get('navigation_note')) for r in calls),
       'decoded_frames':count,'expected_frames':n,'manual_visual_review':False}
    (p/'review-data.json').write_text(json.dumps({'summary':summary,'calls':calls},ensure_ascii=False,indent=2))
    print(json.dumps(summary,ensure_ascii=False,indent=2))

if __name__=='__main__':main()
