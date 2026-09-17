from pathlib import Path
import base64,json,shutil,hashlib,sys
root=Path(sys.argv[1]); out=root/'experiments/research-dispatch-arena-20260917'; out.mkdir(exist_ok=True)
names=[('open','기본 공간','장애물 없이 역할·목적지 합의를 비교합니다.'),('shared_crossing','공유 통로','두 경로와 출하 앞마당에서 통행 순서를 조율합니다.'),('north_blocked','예상 밖 장애물','사전 지도에는 없는 북쪽 장애물을 영상으로 확인합니다.'),('narrow_south','좁은 남쪽 통로','39cm 통로에서 화물과 팀의 통과 가능성을 판단합니다.'),('rough_south','낮은 턱','8mm 턱을 지날지 우회할지 결정합니다.')]
data=[];raw=[]
for key,title,desc in names:
    src=root/'outputs'/f'dispatch-{key if key!="shared_crossing" else "shared"}-v5'
    result=json.loads((src/'result.json').read_text()); config=json.loads((src/'episode-setup-only.json').read_text())
    row={'id':key,'title':title,'desc':desc,'result':result,'map':config['static_map'],'images':{}}
    media=out/'media'/key;media.mkdir(parents=True,exist_ok=True)
    for name,rel in [('overview','initial-overview.jpg'),('top','rgb/initial-top.jpg'),('r1','rgb/initial-r1.jpg'),('r2','rgb/initial-r2.jpg'),('r3','rgb/initial-r3.jpg'),('smoke','motion-smoke-overview.jpg')]:
        p=src/rel
        if p.exists():
            target=media/(name+'.jpg');shutil.copyfile(p,target)
            row['images'][name]='data:image/jpeg;base64,'+base64.b64encode(p.read_bytes()).decode()
    team=src/'team/summary.json'
    if not team.exists():
        choices=list((src/'team').glob('*.json')) if (src/'team').exists() else []
        team=next((p for p in choices if isinstance(json.loads(p.read_text()),dict) and 'rounds' in json.loads(p.read_text())),team)
    row['team']=json.loads(team.read_text()) if team.exists() else None
    if (src/'robot-programs.json').exists():row['programs']=json.loads((src/'robot-programs.json').read_text())
    data.append(row)
    for p in sorted(src.rglob('*')):
        if p.is_file():raw.append({'path':str(p),'bytes':p.stat().st_size,'sha256':hashlib.sha256(p.read_bytes()).hexdigest()})
(out/'results.json').write_text(json.dumps([{k:v for k,v in x.items() if k not in ('images','team')} for x in data],ensure_ascii=False,indent=2)+'\n')
(out/'raw-manifest.json').write_text(json.dumps(raw,indent=2)+'\n')
template=(Path(__file__).parent/'templates/dispatch_review.html').read_text()
(out/'index.html').write_text(template.replace('__DATA__',json.dumps(data,ensure_ascii=False).replace('</','<\\/')))
print(out/'index.html')
