"""Build a Korean review page from completed single-box sensor/video runs."""
import argparse
import html
import json
from pathlib import Path

PHASES = {'approach':'접근', 'lower':'집게 하강', 'close':'집기', 'lift':'들어 올리기',
          'verify_lift':'들기 검사', 'attachment_left':'좌우 확인', 'carry':'운반',
          'release':'내려놓기', 'retract':'팔 회수', 'verify_release':'놓기 확인'}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('output')
    args = parser.parse_args()
    root = Path(args.output).resolve()
    runs = []
    cards = []
    for result_path in sorted(root.glob('*/result.json'), key=lambda p: (p.parent.name.startswith('negative'), p.parent.name)):
        result = json.loads(result_path.read_text())
        folder = result_path.parent.name
        rows = [json.loads(line) for line in (result_path.parent/'control.jsonl').read_text().splitlines()]
        first = {}
        for row in rows:
            first.setdefault(row['phase'], row)
        negative = result.get('negative_control_open_gripper', False)
        score = result['offline_score']
        complete = bool(score['success'] and result['reason'] == 'VISUAL_RELEASE_CONFIRMED')
        run = {'directory': folder, 'seed': result['seed'], 'negative': negative,
               'completed': complete, 'score': score, 'reason': result['reason'],
               'source_hash': result['source_hash'], 'impratio': result['physics_impratio'],
               'noslip_iterations': result['physics_noslip_iterations'],
               'cargo_ids': list(result['initial']['cargo'])}
        runs.append(run)
        label = f"집게 열림 대조 · 시드 {result['seed']}" if negative else f"시드 {result['seed']}"
        status = ('오탐 없음' if not result['visual_lift_confirmed'] and not complete else '검토 필요') if negative else ('완료' if complete else '미완료')
        start = first.get('lower', rows[0])['time'] - rows[0]['time']
        video_id = 'v' + str(len(runs))
        buttons = []
        for phase, title in PHASES.items():
            if phase in first:
                seconds = first[phase]['time'] - rows[0]['time']
                buttons.append(f'<button onclick="seek(\'{video_id}\',{seconds:.3f})">{title}</button>')
        media = (f'<video id="{video_id}" controls playsinline preload="metadata" poster="{folder}/motion-1x.png" src="{folder}/motion-1x.mp4#t={start:.2f}"></video>'
                 if (result_path.parent/'motion-1x.mp4').exists() else '<p>영상 없이 수행한 대조 실행입니다. 아래 실제 입력 영상과 로그를 확인할 수 있습니다.</p>')
        if not (result_path.parent/'motion-1x.mp4').exists():
            buttons = []
        inputs = []
        for phase in ('approach','lower','verify_lift','attachment_left','carry','verify_release'):
            if phase in first:
                row = first[phase]
                inputs.append(f'<figure><a href="{folder}/inputs/{row["step"]:04d}.jpg"><img loading="lazy" src="{folder}/inputs/{row["step"]:04d}.jpg" alt="{PHASES.get(phase,phase)} 실제 입력"/></a><figcaption>{PHASES.get(phase,phase)} · {row["time"]:.1f}초<br><code>{row["frame_sha256"][:16]}</code></figcaption></figure>')
        cards.append(f'''<section><div class="row"><h2>{label}</h2><span class="badge {'ok' if status in ('완료','오탐 없음') else 'bad'}">{status}</span></div>
<p class="metrics">최대 상승 <b>{score['max_lift_above_initial_m']*100:.1f} cm</b>　최종 이동 <b>{score['planar_displacement_m']*100:.1f} cm</b>　강제 부착 <b>{'사용' if score['active_constraints_during_episode'] else '없음'}</b></p>
{media}<div class="seek">{''.join(buttons)}</div>
<p class="reason">종료 로그: <code>{html.escape(result['reason'])}</code></p>
<details><summary>제어기에 전달된 실제 JPEG와 원본 로그</summary><div class="inputs">{''.join(inputs)}</div>
<p><a href="{folder}/control.jsonl">시각 판단·행동</a> · <a href="{folder}/commands.jsonl">모터 명령</a> · <a href="{folder}/evaluation-only.jsonl">평가 전용 위치 기록</a> · <a href="{folder}/result.json">전체 결과</a></p></details></section>''')
    positives = [r for r in runs if not r['negative']]
    successful = sum(r['completed'] for r in positives)
    hashes = sorted({r['source_hash'] for r in runs})
    summary = {'positive_completed': successful, 'positive_total': len(positives), 'source_hashes': hashes, 'runs': runs}
    (root/'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2))
    page = '''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>R1 카메라 단독 운반 검증</title>
<style>*{box-sizing:border-box}body{margin:0;background:#0d131b;color:#eaf1fa;font:16px/1.65 system-ui,-apple-system,sans-serif}main{max-width:1120px;margin:auto;padding:32px 20px 80px}h1{font-size:clamp(27px,4vw,40px);line-height:1.25;margin:8px 0 20px}h2{font-size:23px;margin:0}.eyebrow{color:#70cee7;letter-spacing:.08em;font-size:13px}.lede{color:#bac8d9;max-width:880px}.summary{padding:20px 24px;border-left:4px solid #6bd2ba;background:#152632;border-radius:8px;margin:24px 0}.summary strong{font-size:25px}section{background:#151e2a;border:1px solid #2a3a4b;border-radius:16px;padding:22px;margin:24px 0}.row{display:flex;align-items:center;justify-content:space-between;gap:12px}.badge{border-radius:30px;padding:5px 14px;font-size:14px;white-space:nowrap}.ok{background:#173f36;color:#85edc5}.bad{background:#4b2730;color:#ffc0c9}.metrics{color:#b3c1d2;font-size:15px}b{color:#f1f6fc}video{display:block;width:100%;max-height:680px;border-radius:10px;background:#06090e}.seek{display:flex;flex-wrap:wrap;gap:8px;margin:13px 0}button{background:#213548;border:1px solid #405e75;border-radius:8px;padding:8px 13px;color:#d8edfa;font:inherit;font-size:14px;cursor:pointer}button:hover{background:#345069}details{border-top:1px solid #2b3a4e;padding-top:12px}summary{cursor:pointer;color:#8ad1ee}.inputs{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:12px;margin-top:18px}figure{margin:0}img{width:100%;border-radius:8px}figcaption{font-size:13px;color:#a8b8cb}a{color:#8cd6f6}code{font-size:12px;overflow-wrap:anywhere}.reason{color:#98aabe;font-size:13px}.notes{color:#aab9ca;font-size:14px}@media(max-width:650px){main{padding:22px 12px}section{padding:14px}.inputs{grid-template-columns:repeat(2,minmax(0,1fr))}.metrics{font-size:13px}}
</style><main><div class="eyebrow">SINGLE-ROBOT EXECUTION SKILL · RGB ONLY</div><h1>카메라로 찾고, 집고, 옮기고, 확인하기</h1>
<p class="lede">R1 한 대와 상자 한 개로 수행한 단독 기술 시험입니다. 0.82m 정사각형 구역과 구역 사이의 기존 장애물은 유지했습니다. 제어기는 자기 손목 RGB와 자기 모터 명령 기록만 받으며, 영상의 외부 근접 화면과 평가용 정답 위치는 제어기에 전달되지 않습니다.</p>
''' + f'<div class="summary"><strong>단독 운반 {successful} / {len(positives)} 완료</strong><br>집기 확인 → 짧은 전진 운반 → 내려놓기 → 영상으로 놓기 확인. 영상은 1배속이며 집기 구간부터 열립니다.</div>' + '''
<p class="notes">현재 구현은 알려진 표식·상자 치수·카메라 보정을 사용하는 결정론적 시각 실행 기술입니다. LLM 호출과 로봇 간 대화는 0회입니다. 이 단계는 여러 로봇의 협력, 임의 환경 인식, 구역 간 장애물 회피 또는 최적 경로의 검증이 아닙니다.</p>
''' + ''.join(cards) + f'''<section><h2>검증 조건</h2><p>완료 판정은 제어기의 놓기 확인과 평가 전용 기록을 함께 사용합니다. 상자를 4cm 이상 들어 올리고, 최종 위치가 25cm 이상 이동했으며, 안정적으로 멈추고 강제 부착 제약을 사용하지 않아야 합니다. 집게 열림 대조 실행은 집기 성공을 선언하지 않아야 통과합니다.</p><p class="notes">접촉 해석 설정은 각 결과의 impratio·noslip 값을 기록했습니다. 마찰계수·집게 힘·로봇 및 화물 크기는 변경하지 않았습니다. <a href="https://mujoco.readthedocs.io/en/stable/modeling.html#preventing-slip">MuJoCo 접촉 미끄러짐 설명</a> · <a href="summary.json">전체 요약 JSON</a> · <a href="source-manifest.json">소스·보정 파일 기록</a></p><p class="notes">주요 소스 해시 수: {len(hashes)}<br><code>{'<br>'.join(hashes)}</code></p></section></main><script>function seek(id,t){{const v=document.getElementById(id);if(v){{v.currentTime=t;v.play().catch(()=>{{}})}}}}</script></html>'''
    (root/'index.html').write_text(page)
    print(root/'index.html')


if __name__ == '__main__':
    main()
