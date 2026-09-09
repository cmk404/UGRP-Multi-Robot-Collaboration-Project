"""Record the single-robot visual box skill without feeding spectator truth to it.

The warehouse camera is presentation-only.  This recorder reads R1's base pose
solely to move that camera and never calls the skill or changes an actor camera.
"""
from __future__ import annotations

import math
from pathlib import Path
import subprocess

import cv2
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont


_PHASE_KO = {
    "search": "상자 탐색",
    "approach": "상자 접근",
    "align": "집기 정렬",
    "hover": "집기 위치 접근",
    "lower": "집게 하강",
    "close": "집게 닫기",
    "lift": "들어 올리기",
    "verify_lift": "들기 확인",
    "attachment_left": "좌측 부착 확인",
    "attachment_right": "우측 부착 확인",
    "attachment_home": "중앙 부착 확인",
    "carry": "상자 운반",
    "release": "상자 내려놓기",
    "open": "집게 열기",
    "retract": "팔 회수",
    "verify_release": "놓기 확인",
    "release_ground_left": "바닥 정지 확인 · 좌측",
    "release_ground_right": "바닥 정지 확인 · 우측",
    "release_ground_home": "바닥 정지 확인 · 중앙",
    "finish": "작업 종료",
    "finished": "작업 종료",
    "failed": "작업 중단",
}


def _font(size: int) -> ImageFont.ImageFont:
    candidates = (
        "/System/Library/Fonts/AppleSDGothicNeo.ttc",
        "/System/Library/Fonts/Supplemental/AppleGothic.ttf",
        "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    )
    for name in candidates:
        try:
            return ImageFont.truetype(name, size=size)
        except OSError:
            pass
    return ImageFont.load_default()


def _letterbox(frame: np.ndarray, width: int, height: int, color=(18, 21, 26)) -> np.ndarray:
    """Fit a BGR frame without cropping or changing its aspect ratio."""
    h, w = frame.shape[:2]
    scale = min(width / w, height / h)
    resized = cv2.resize(frame, (max(1, round(w * scale)), max(1, round(h * scale))))
    result = np.full((height, width, 3), color, dtype=np.uint8)
    y = (height - resized.shape[0]) // 2
    x = (width - resized.shape[1]) // 2
    result[y:y + resized.shape[0], x:x + resized.shape[1]] = resized
    return result


def _look_quaternion(position: np.ndarray, target: np.ndarray) -> np.ndarray:
    forward = np.asarray(target, dtype=float) - np.asarray(position, dtype=float)
    norm = float(np.linalg.norm(forward))
    if norm < 1e-9:
        raise ValueError("camera position and target coincide")
    forward /= norm
    right = np.cross(forward, np.array((0.0, 0.0, 1.0)))
    right_norm = float(np.linalg.norm(right))
    if right_norm < 1e-9:
        right = np.array((1.0, 0.0, 0.0))
    else:
        right /= right_norm
    up = np.cross(right, forward)
    quaternion = np.empty(4, dtype=float)
    mujoco.mju_mat2Quat(quaternion, np.column_stack((right, up, -forward)).ravel())
    return quaternion


class SingleBoxVideo:
    """Encode a 1x, 10 fps presentation of one RGB-controlled R1 episode.

    ``skill`` is read only for its public status labels.  Camera pixels and
    camera placement are never passed back to it, so this class cannot become
    a controller truth channel.
    """

    WIDTH = 1280
    HEIGHT = 720
    FPS = 10

    def __init__(self, world, path, skill):
        self.world = world
        self.path = Path(path)
        self.skill = skill
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with world.physics_lock:
            self.start = float(world.data.time)
            self.camera_id = mujoco.mj_name2id(
                world.model, mujoco.mjtObj.mjOBJ_CAMERA, "cctv_warehouse"
            )
        if self.camera_id < 0:
            raise ValueError("cctv_warehouse camera is missing")
        self.next_frame = self.start
        self.frames = 0
        self._closed = False
        self._last_canvas = None
        self._title_font = _font(27)
        self._label_font = _font(20)
        self._small_font = _font(17)
        self.process = subprocess.Popen(
            [
                "ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo",
                "-pix_fmt", "bgr24", "-s", f"{self.WIDTH}x{self.HEIGHT}",
                "-r", str(self.FPS), "-i", "-", "-an", "-c:v", "libx264",
                "-preset", "veryfast", "-crf", "21", "-pix_fmt", "yuv420p",
                "-movflags", "+faststart", str(self.path),
            ],
            stdin=subprocess.PIPE,
        )

    def _place_spectator(self) -> float:
        """Move only the fixed spectator camera; return current simulation time."""
        with self.world.physics_lock:
            robot = self.world.robot("r1")
            base = np.asarray(robot.base_xyz(), dtype=float)
            yaw = float(robot.base_rpy()[2])
            c, s = math.cos(yaw), math.sin(yaw)
            rotation = np.array(((c, -s), (s, c)))
            offset_xy = rotation @ np.array((-0.55, -0.55))
            look_xy = rotation @ np.array((0.10, 0.0))
            position = base + np.array((offset_xy[0], offset_xy[1], 0.55))
            target = base + np.array((look_xy[0], look_xy[1], 0.12))
            self.world.model.cam_pos[self.camera_id] = position
            self.world.model.cam_quat[self.camera_id] = _look_quaternion(position, target)
            self.world.model.cam_fovy[self.camera_id] = 58.0
            # Updates derived camera/light transforms without advancing physics
            # or recomputing actor kinematics.
            mujoco.mj_camlight(self.world.model, self.world.data)
            return float(self.world.data.time)

    @staticmethod
    def _decode(jpeg: bytes) -> np.ndarray:
        frame = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            raise RuntimeError("renderer returned an invalid JPEG")
        return frame

    def _compose(self, now: float) -> np.ndarray:
        # Rendering occurs after _place_spectator releases physics_lock.  The
        # render broker may acquire the same lock on its dedicated thread.
        overview = self._decode(
            self.world.render_team_jpeg(camera="cctv_warehouse", quality=91)
        )
        wrist = self._decode(
            self.world.render_jpeg(robot_id="r1", camera="robot_cam", quality=91)
        )
        canvas = np.full((self.HEIGHT, self.WIDTH, 3), (22, 25, 30), np.uint8)
        canvas[66:690, 12:844] = _letterbox(overview, 832, 624)
        canvas[116:476, 860:1268] = _letterbox(wrist, 408, 360)
        cv2.rectangle(canvas, (858, 114), (1269, 477), (98, 204, 246), 2)

        image = Image.fromarray(cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB))
        draw = ImageDraw.Draw(image)
        draw.rectangle((0, 0, self.WIDTH, 65), fill=(18, 21, 26))
        draw.text((18, 15), "R1 단독 · 카메라 기반 집기·운반", font=self._title_font,
                  fill=(245, 247, 250))
        draw.text((870, 78), "R1 손목 카메라 · RGB", font=self._label_font,
                  fill=(120, 220, 255))
        phase = str(getattr(self.skill, "phase", "search"))
        phase_label = _PHASE_KO.get(phase, phase)
        draw.text((868, 506), f"단계  {phase_label}", font=self._label_font,
                  fill=(245, 247, 250))
        draw.text((868, 541), f"시뮬레이션 시간  {now - self.start:05.1f}초 · 1배속",
                  font=self._small_font, fill=(190, 201, 214))
        held = getattr(self.skill, "held", None)
        held_text = "확인됨" if held is True else "확인 안 됨" if held is False else "확인 중"
        draw.text((868, 575), f"집기 확인 기록  {held_text}", font=self._small_font,
                  fill=(190, 201, 214))
        draw.text((868, 615), "외부 화면은 관찰용 · R2/R3 비활성", font=self._small_font,
                  fill=(150, 160, 172))
        draw.text((868, 648), "제어: 자기 RGB·명령 기억 / LLM 호출 0",
                  font=_font(14), fill=(255, 205, 104))
        return cv2.cvtColor(np.asarray(image), cv2.COLOR_RGB2BGR)

    def capture(self, force: bool = False) -> None:
        if self._closed:
            return
        now = self._place_spectator()
        if not force and now < self.next_frame:
            return
        canvas = self._compose(now)
        self._last_canvas = canvas
        while self.next_frame <= now + 1e-9:
            assert self.process.stdin is not None
            self.process.stdin.write(canvas.tobytes())
            self.frames += 1
            self.next_frame += 1.0 / self.FPS
        if force:
            cv2.imwrite(str(self.path.with_suffix(".png")), canvas)

    def close(self) -> None:
        if self._closed:
            return
        self.capture(force=True)
        self._closed = True
        if self.process.stdin is not None:
            self.process.stdin.close()
        return_code = self.process.wait(timeout=30)
        if return_code:
            raise RuntimeError(f"video encoder failed with exit code {return_code}")

