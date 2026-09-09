"""Preflight and run the repository's three-robot mixed cooperation episode."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--condition', choices=('rule', 'llm_peer_comm'), default='llm_peer_comm')
    parser.add_argument('--model', default='gemini-3.8-flash')
    parser.add_argument('--check-only', action='store_true')
    parser.add_argument('--timeout', type=float, default=240)
    parser.add_argument('--max-calls', type=int, default=12)
    args = parser.parse_args()
    os.chdir(ROOT)
    if sys.platform == 'win32':
        os.environ['MUJOCO_GL'] = 'glfw'
    os.environ['PYTHONPATH'] = str(ROOT)
    os.environ['PYTHONUTF8'] = '1'
    if args.timeout <= 0 or args.max_calls <= 0:
        parser.error('timeout and max-calls must be positive')
    try:
        if sys.platform == 'win32' and not shutil.which('ffmpeg'):
            packages = Path(os.environ.get('LOCALAPPDATA', '')) / 'Microsoft/WinGet/Packages'
            candidates = sorted(packages.glob('Gyan.FFmpeg_*/**/ffmpeg.exe'))
            if candidates:
                os.environ['PATH'] = str(candidates[-1].parent) + os.pathsep + os.environ.get('PATH', '')
        if not shutil.which('ffmpeg'):
            raise RuntimeError('FFmpeg missing. Install it, then reopen PowerShell: winget install --id Gyan.FFmpeg --exact')
        subprocess.run(['ffmpeg', '-version'], check=True, stdout=subprocess.DEVNULL)
        from scripts.render_warehouse_dialogue import font
        font(20)
        from sim.multi_masterpi_production import MultiMasterPiProductionV2
        world = MultiMasterPiProductionV2(warehouse_layout='mixed', render=True)
        try:
            if tuple(world.robot_ids) != ('r1', 'r2', 'r3'):
                raise RuntimeError('Expected three robots')
            for rid in world.robot_ids:
                if not world.render_jpeg(robot_id=rid):
                    raise RuntimeError('Camera failed: ' + rid)
        finally:
            world.close()
        print('PASS: three robots, cameras, font and FFmpeg', flush=True)
        if args.condition == 'llm_peer_comm':
            from harness.gemini_proxy import GeminiProxyCompleter
            print('Checking Gemini with one small API request...', flush=True)
            client = GeminiProxyCompleter(model=args.model, max_tokens=128, timeout=30)
            answer = client.complete([{'role': 'user', 'content': 'Reply OK.'}])
            if not answer.strip():
                raise RuntimeError('Gemini returned no text')
            print('PASS: Gemini model responded', flush=True)
        if args.check_only:
            return 0
        run_id = 'three-robot-' + datetime.now().strftime('%Y%m%d-%H%M%S-%f')
        output = ROOT / 'outputs' / run_id
        output.mkdir(parents=True, exist_ok=False)
        sha = subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip()
        dirty = bool(subprocess.check_output(['git', 'status', '--porcelain'], text=True).strip())
        (output / 'environment.json').write_text(json.dumps({
            'code_sha': sha, 'working_tree_dirty': dirty, 'python': sys.version,
            'platform': sys.platform, 'options': vars(args),
            'raw_storage': 'local only',
        }, indent=2), encoding='utf-8')
        episode = output / 'episode'
        command = [sys.executable, '-m', 'scripts.record_mixed_warehouse',
                   '--output', str(episode), '--condition', args.condition,
                   '--model', args.model, '--timeout', str(args.timeout),
                   '--max-calls', str(args.max_calls)]
        print('Output: ' + str(output), flush=True)
        with (output / 'console.log').open('w', encoding='utf-8') as log:
            child = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     text=True, encoding='utf-8', errors='replace')
            try:
                for line in child.stdout:
                    print(line, end='', flush=True)
                    log.write(line)
                    log.flush()
                code = child.wait()
            finally:
                if child.poll() is None:
                    if sys.platform == 'win32':
                        subprocess.run(['taskkill', '/PID', str(child.pid), '/T', '/F'],
                                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    else:
                        child.terminate()
                    child.wait()
        result_path = episode / 'result.json'
        if not result_path.exists():
            raise RuntimeError('No result.json. See ' + str(output / 'console.log'))
        result = json.loads(result_path.read_text(encoding='utf-8'))
        print('Mission success:', result.get('success'), '| Reason:', result.get('reason'))
        print('Result:', result_path)
        video = episode / 'mixed-dialogue-1x.mp4'
        if not video.exists() or video.stat().st_size == 0:
            raise RuntimeError('Dialogue video missing. See console.log')
        import cv2
        capture = cv2.VideoCapture(str(video))
        try:
            readable, _ = capture.read()
        finally:
            capture.release()
        if not readable:
            raise RuntimeError('Video cannot be decoded')
        print('Video:', video)
        return 0 if code == 0 and result.get('success') is True else 1
    except KeyboardInterrupt:
        print('Stopped by user.', file=sys.stderr)
        return 130
    except Exception as exc:
        print('FAILED: ' + str(exc), file=sys.stderr)
        return 1


if __name__ == '__main__':
    raise SystemExit(main())
