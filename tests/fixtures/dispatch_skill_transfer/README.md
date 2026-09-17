# 실제 RGB 색상 회귀 입력

`outputs/dispatch-skills-s6/rgb/pair-210-carry-anchor-r1.jpg` 및 `pair-228-carry-r1.jpg`의 원본 바이트. 실행 SHA `47a71bc`, 모델 `dispatch-transfer-t4`, open/seed11 기록 계획 진단. 카메라/로봇/빔 변경 없음, weld OFF. 동일 빔의 황색 영역이 기존 HSV 상한24를 넘어 실제 마스크가 잘리는 오류를 재현한다. 카메라 입력 특징 검사용이며 물리적 운반 성공 증거는 아니다.

`beam-top-lit.jpg`: S7 source ca11f79, pair-233-carry-top; loaded beam under unchanged lighting, raw shared camera.

`box-top-held.jpg`: S8 source 6e349db, solo-121-top; raw shared camera with held box and cyan floor distractors.

`box-transit-274/275.jpg`: S13 source 273da1b, raw solo-274/275-top. Transition into changed illumination during actual carrying.

`box-own-158/159.jpg`: Q1 source 71900de; unchanged cargo grip while cyan floor joined old segmentation. Raw owned camera, 640x480 same-FOV resize.

`box-floor-122/162/163.jpg`: Q2 source 18474aa, raw shared RGB. At 163 the cargo mask joins the cyan floor; prior observed background separates it.

`box-floor-165.jpg`: Q3 source 2e5f20b; raw shared RGB, narrow visible cyan edge over cyan floor.

`box-edge-190/191.jpg`: Q4 source 6c7a941; thin cargo color component leaving cyan floor. Raw TOP.

`box-shadow-237/238.jpg`: Q5 source cbfc38a, raw TOP; held cargo retains silhouette while cyan edge has 14 pixels.

`box-apron-303/304.jpg`: Q6 source dea5d27, raw TOP. Background boundary changes template correlation; foreground corner flow retains 10 mutually consistent moving features.
