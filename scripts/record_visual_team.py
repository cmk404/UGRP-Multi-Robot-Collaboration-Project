"""Spectator video with each robot's exact most recent RGB decision inputs."""
import base64
import textwrap
import cv2
import mujoco
import numpy as np
from PIL import Image, ImageDraw
from scripts.record_visual_box import SingleBoxVideo, _letterbox, _look_quaternion, _font, _PHASE_KO

_TEAM_STATE_KO = {
    "idle": "판단 대기", "ready_to_pick": "집기 승인 대기", "carrying": "목적지 이동",
    "grip_uncertain": "파지 재확인", "released": "배치 확인 대기", "releasing": "배치 중",
    "failure": "작업 중단", "finished": "작업 종료", "navigate": "목적지 이동",
}


class VisualTeamVideo(SingleBoxVideo):
    WIDTH=1280
    HEIGHT=960
    FPS=8

    def __init__(self,world,path,actors):
        self.actors=actors
        self.inputs={}
        super().__init__(world,path,None)

    def update_inputs(self,rid,wrist,nav,time):
        self.inputs[rid]=(self._decode(base64.b64decode(wrist['image'])),
                          self._decode(base64.b64decode(nav['image'])),time)

    def _place_spectator(self):
        with self.world.physics_lock:
            position=np.array((0.,-4.5,5.2));target=np.array((0.,0.,0.))
            self.world.model.cam_pos[self.camera_id]=position
            self.world.model.cam_quat[self.camera_id]=_look_quaternion(position,target)
            self.world.model.cam_fovy[self.camera_id]=58
            mujoco.mj_camlight(self.world.model,self.world.data)
            return float(self.world.data.time)

    def _compose(self,now):
        canvas=np.full((960,1280,3),(22,25,30),np.uint8)
        overview=self._decode(self.world.render_team_jpeg(camera='cctv_warehouse',quality=88))
        canvas[40:480,:640]=_letterbox(overview,640,440)
        panels=[(640,0),(0,480),(640,480)]
        for rid,(x,y) in zip(('r1','r2','r3'),panels):
            item=self.inputs.get(rid)
            if item:
                wrist,nav,time=item
                canvas[y+40:y+480,x:x+640]=_letterbox(nav,640,440)
                canvas[y+340:y+460,x+465:x+625]=cv2.resize(wrist,(160,120))
                cv2.rectangle(canvas,(x+464,y+339),(x+626,y+461),(255,220,100),1)
        pic=Image.fromarray(cv2.cvtColor(canvas,cv2.COLOR_BGR2RGB));draw=ImageDraw.Draw(pic)
        draw.text((12,9),f'외부 관찰 · {now-self.start:.1f}초 · 1배속',font=_font(22),fill='white')
        for rid,(x,y) in zip(('r1','r2','r3'),panels):
            actor=self.actors.get(rid)
            if actor is None:
                text=f'{rid.upper()} · 이번 단독 시험에서는 대기'
            else:
                phase=_PHASE_KO.get(actor.phase,_TEAM_STATE_KO.get(actor.phase,actor.phase))
                text=f'{rid.upper()} · {phase} · 목적지 {actor.destination_zone}'
            draw.text((x+12,y+8),text,font=_font(20),fill='white')
            sent = getattr(actor, 'last_message', None) if actor else None
            if sent and 0 <= now-sent['sent_at'] <= 15:
                content = sent['content']
                if isinstance(content, dict):
                    content = f"{content['observed']} / 의도 {content['intent']} / 요청 {content['request']}"
                draw.rectangle((x+10,y+48,x+625,y+123),fill=(18,47,65))
                for j,line in enumerate(textwrap.wrap(f"동료에게 전송: {content}",width=40)[:3]):
                    draw.text((x+16,y+52+j*22),line,font=_font(17),fill=(160,235,255))
            if actor and getattr(actor,'last_llm_reason',''):
                reason=actor.last_llm_reason
                draw.rectangle((x+10,y+355,x+455,y+436),fill=(15,20,28))
                for j,line in enumerate(textwrap.wrap('Gemini 판단: '+reason,width=30)[:3]):
                    draw.text((x+16,y+360+j*23),line,font=_font(17),fill=(225,236,250))
            if rid in self.inputs:
                age=now-self.inputs[rid][2]
                draw.rectangle((x+10,y+438,x+430,y+475),fill=(15,20,28))
                draw.text((x+16,y+443),f'실제 최근 입력 · {age:.1f}초 전 / 작은 창: 손목',font=_font(16),fill=(165,225,245))
        return cv2.cvtColor(np.asarray(pic),cv2.COLOR_RGB2BGR)
