"""Headless Chrome (CDP) capture of a saved TensorBoard link with every filtered run selected.

TensorBoard deselects all runs when the logdir has more than 40 runs, so a plain
--screenshot shows empty pinned cards. This opens the saved link, ticks the run
table's select-all box (only the runs the saved runFilter shows), waits, and saves a
PNG. Local 127.0.0.1 pages only.

    python tb_capture.py URL OUT.png
"""
import asyncio
import base64
import json
import shutil
import subprocess
import sys
import tempfile
import time
import urllib.request

import websockets

CHROME = '/Applications/Google Chrome.app/Contents/MacOS/Google Chrome'
PORT = 9347


async def main(url, out):
    assert url.startswith('http://127.0.0.1:')
    prof = tempfile.mkdtemp(prefix='kiro-tb-')
    proc = subprocess.Popen([CHROME, '--headless=new', '--disable-gpu', '--no-first-run', f'--user-data-dir={prof}',
                             f'--remote-debugging-port={PORT}', '--window-size=1700,1700', 'about:blank'],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        for _ in range(50):
            try:
                pages = json.load(urllib.request.urlopen(f'http://127.0.0.1:{PORT}/json'))
                break
            except OSError:
                time.sleep(.2)
        ws_url = next(p['webSocketDebuggerUrl'] for p in pages if p['type'] == 'page')
        async with websockets.connect(ws_url, max_size=None) as ws:
            n = 0

            async def call(method, **params):
                nonlocal n
                n += 1
                await ws.send(json.dumps({'id': n, 'method': method, 'params': params}))
                while True:
                    msg = json.loads(await ws.recv())
                    if msg.get('id') == n:
                        return msg.get('result', {})
            await call('Emulation.setDeviceMetricsOverride', width=1700, height=1700, deviceScaleFactor=1, mobile=False)
            await call('Page.navigate', url=url)
            await asyncio.sleep(12)
            js = ("(() => {const b = document.querySelector('runs-selector mat-checkbox input, "
                  "tb-run-selector mat-checkbox input, .run-table-container mat-checkbox input, "
                  "mat-checkbox input'); if (!b) return 'no-box'; if (!b.checked) b.click(); return 'clicked';})()")
            res = await call('Runtime.evaluate', expression=js, returnByValue=True)
            print('select-all:', res.get('result', {}).get('value'))
            await asyncio.sleep(10)
            shot = await call('Page.captureScreenshot', format='png')
            open(out, 'wb').write(base64.b64decode(shot['data']))
    finally:
        proc.terminate()
        proc.wait(timeout=10)
        shutil.rmtree(prof, ignore_errors=True)


if __name__ == '__main__':
    asyncio.run(main(sys.argv[1], sys.argv[2]))
