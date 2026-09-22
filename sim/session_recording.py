"""Optional observer-only recording. It never changes the policy input camera."""
from __future__ import annotations
import json
from pathlib import Path
import shutil
import subprocess


class Video:
    def __init__(self, output, *, camera, fps):
        executable = shutil.which('ffmpeg')
        if executable is None:
            raise ValueError('--video requires ffmpeg on PATH')
        self.camera, self.fps = camera, fps
        self.frames = 0
        self.next_time = 0.
        self.episode = -1
        self.closed = False
        output = Path(output)
        self.log = (output/'video-encoder.log').open('wb')
        self.timeline = (output/'video-frames.jsonl').open('w')
        try:
            self.process = subprocess.Popen([executable, '-nostdin', '-n', '-loglevel', 'error',
                '-f', 'image2pipe', '-vcodec', 'mjpeg', '-r', str(fps), '-i', '-',
                '-an', '-c:v', 'libx264', '-preset', 'veryfast', '-crf', '20',
                '-vf', 'pad=ceil(iw/2)*2:ceil(ih/2)*2', '-pix_fmt', 'yuv420p',
                '-movflags', '+faststart', str(output/'motion.mp4')],
                stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=self.log)
        except BaseException:
            self.log.close(); self.timeline.close()
            raise

    def frame(self, sim):
        if sim.episode != self.episode:
            self.episode, self.next_time = sim.episode, 0.
        if sim.time + 1e-9 < self.next_time:
            return
        jpeg = sim._world.render_team_jpeg(camera=self.camera)
        self.process.stdin.write(jpeg)
        self.timeline.write(json.dumps({'frame': self.frames, 'episode': sim.episode,
                                       'sim_s': sim.time, 'camera': self.camera})+'\n')
        self.frames += 1
        self.next_time += 1 / self.fps

    def close(self):
        if not self.closed:
            self.closed = True
            try:
                try:
                    self.process.stdin.close()
                except BrokenPipeError:
                    pass
                try:
                    code = self.process.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    self.process.kill(); self.process.wait()
                    raise RuntimeError('video encoder did not stop; see video-encoder.log')
                if code or not self.frames:
                    raise RuntimeError(f'video encoder failed ({code}); see video-encoder.log')
            finally:
                self.log.close(); self.timeline.close()
        return {'file': 'motion.mp4', 'frames': self.frames, 'fps': self.fps, 'camera': self.camera,
                'time_basis': 'simulation time; pauses omitted; resets listed in video-frames.jsonl'}
