(() => {
  if (!window.THREE) return;
  const T = window.THREE;
  let running = false, pollTimer = null, raf = null, lastRenderAt = 0;
  let scene, workcellGroup, robot, armYaw, shoulder, elbow, wrist, grip, leftFinger, rightFinger, povCamera;
  let redBlock, blueBlock, yellowBlock;
  let radarScene, radarRobot, radarScan, radarSweep, radarObjectGroup, radarObstacleGroup, radarZoneGroup;
  let radarObservation = -1;
  const views = [];
  let target = null, current = null, workerAge = null, computeBackend = 'SIM worker', currentMode = 'sim';
  function apiPath(path){ return String(window.UGRP_API_PREFIX||'') + path; }

  const mat = {
    dark: new T.MeshStandardMaterial({color:0x22262a, roughness:.72, metalness:.15}),
    black: new T.MeshStandardMaterial({color:0x090a0c, roughness:.55, metalness:.20}),
    aluminum: new T.MeshStandardMaterial({color:0xb2b6b8, roughness:.32, metalness:.72}),
    orange: new T.MeshStandardMaterial({color:0xf35a11, roughness:.48, metalness:.12}),
    green: new T.MeshStandardMaterial({color:0x123f24, roughness:.7}),
    red: new T.MeshStandardMaterial({color:0xe51b1b, roughness:.6}),
    blue: new T.MeshStandardMaterial({color:0x245ee8, roughness:.6}),
    yellow: new T.MeshStandardMaterial({color:0xf2c020, roughness:.6}),
    floor: new T.MeshStandardMaterial({color:0x555b60, roughness:.95}),
    wood: new T.MeshStandardMaterial({color:0x624226, roughness:.9}),
  };

  function box(group, size, pos, material, name='') {
    const m = new T.Mesh(new T.BoxGeometry(size[0],size[1],size[2]), material);
    m.position.set(pos[0],pos[1],pos[2]); m.castShadow=true; m.receiveShadow=true; m.name=name; group.add(m); return m;
  }
  function cyl(group, radius, depth, pos, material, rot=[Math.PI/2,0,0]) {
    const m = new T.Mesh(new T.CylinderGeometry(radius,radius,depth,20), material);
    m.position.set(...pos); m.rotation.set(...rot); m.castShadow=true; group.add(m); return m;
  }
  function bar(group, dx, dy, z, thickness, material) {
    const len=Math.hypot(dx,dy); const m=box(group,[len,thickness,thickness],[dx/2,dy/2,z],material);
    m.rotation.z=Math.atan2(dy,dx); return m;
  }
  function mj(v) { return [v[0], v[2], -v[1]]; }

  function nums(text, fallback=[]) {
    if(!text) return fallback.slice();
    return text.trim().split(/\s+/).map(Number).filter(Number.isFinite);
  }
  function mjSize(size) { return [2*(size[0]||0), 2*(size[2]||0), 2*(size[1]||0)]; }
  function xmlMaterial(el, materialRgba) {
    let rgba=nums(el.getAttribute('rgba'));
    if(rgba.length<3) rgba=(materialRgba[el.getAttribute('material')]||[]).slice();
    if(rgba.length<3) return mat.floor;
    const alpha=rgba.length>3?rgba[3]:1;
    return new T.MeshStandardMaterial({color:new T.Color(rgba[0],rgba[1],rgba[2]),roughness:.82,metalness:.06,transparent:alpha<.999,opacity:alpha,depthWrite:alpha>.55});
  }
  async function loadWorkcellScene() {
    const r=await fetch('/sim-scene.xml',{cache:'no-store'});
    if(!r.ok) throw new Error(`scene xml ${r.status}`);
    const doc=new DOMParser().parseFromString(await r.text(),'application/xml');
    const world=doc.querySelector('worldbody');
    if(!world) throw new Error('MuJoCo worldbody missing');
    const materialRgba={};
    doc.querySelectorAll('asset > material').forEach((m)=>{ const v=nums(m.getAttribute('rgba')); if(v.length>=3) materialRgba[m.getAttribute('name')]=v; });
    clearGroup(workcellGroup);
    Array.from(world.children).filter((el)=>el.tagName==='geom').forEach((el)=>{
      const type=(el.getAttribute('type')||'sphere').toLowerCase();
      const pos=nums(el.getAttribute('pos'),[0,0,0]);
      const m=xmlMaterial(el,materialRgba);
      let mesh=null;
      if(type==='plane'){
        mesh=new T.Mesh(new T.PlaneGeometry(6,6),m); mesh.rotation.x=-Math.PI/2; mesh.position.set(...mj(pos)); mesh.receiveShadow=true;
      } else if(type==='box'){
        const size=nums(el.getAttribute('size')); if(size.length<3)return;
        mesh=new T.Mesh(new T.BoxGeometry(...mjSize(size)),m); mesh.position.set(...mj(pos)); mesh.castShadow=true; mesh.receiveShadow=true;
      }
      if(mesh){ mesh.name=el.getAttribute('name')||''; workcellGroup.add(mesh); }
    });
  }

  function buildScene() {
    scene = new T.Scene(); scene.background = new T.Color(0x22262a);
    scene.fog = new T.Fog(0x22262a, 2.8, 5.0);
    scene.add(new T.HemisphereLight(0xddeeff,0x333333,1.15));
    const sun=new T.DirectionalLight(0xffffff,1.15); sun.position.set(-.5,2.7,1.2); sun.castShadow=true; scene.add(sun);

    const grid=new T.GridHelper(6.0,60,0x777c80,0x666b70); grid.position.set(.65,.002,0); scene.add(grid);
    workcellGroup=new T.Group(); scene.add(workcellGroup);

    redBlock=box(scene,[.05,.05,.05],[.55,.025,0],mat.red,'red');
    blueBlock=box(scene,[.05,.05,.05],[.70,.025,.20],mat.blue,'blue');
    yellowBlock=box(scene,[.05,.05,.05],[.82,.025,-.23],mat.yellow,'yellow');

    robot=new T.Group(); scene.add(robot);
    box(robot,[.128,.044,.108],[0,.018,0],mat.dark);
    box(robot,[.118,.020,.096],[-.004,.052,0],mat.aluminum);
    box(robot,[.012,.044,.096],[.068,.021,0],mat.aluminum);
    box(robot,[.062,.036,.086],[-.044,.078,0],mat.aluminum);
    box(robot,[.064,.012,.076],[-.040,.066,0],mat.green);
    // ultrasonic pair
    box(robot,[.018,.030,.064],[.076,.050,0],mat.black);
    for (const zz of [-.017,.017]) { cyl(robot,.010,.012,[.086,.050,zz],mat.aluminum,[0,0,Math.PI/2]); cyl(robot,.006,.014,[.092,.050,zz],new T.MeshStandardMaterial({color:0x4aa2e9}),[0,0,Math.PI/2]); }
    // four mecanum wheels (axle along Three z)
    for (const x of [-.052,.052]) for (const z of [-.067,.067]) { cyl(robot,.034,.028,[x,0,z],mat.dark,[Math.PI/2,0,0]); cyl(robot,.029,.010,[x,0,z+(z>0?.015:-.015)],mat.orange,[Math.PI/2,0,0]); }

    armYaw=new T.Group(); armYaw.position.set(0,.064,0); robot.add(armYaw);
    cyl(armYaw,.026,.024,[0,.010,0],mat.black,[0,0,0]); box(armYaw,[.048,.050,.052],[0,.037,0],mat.orange);
    shoulder=new T.Group(); shoulder.position.set(0,.062,0); armYaw.add(shoulder);
    box(shoulder,[.042,.042,.048],[0,0,0],mat.black); bar(shoulder,.072,.060,-.025,.016,mat.orange); bar(shoulder,.072,.060,.025,.016,mat.orange);
    elbow=new T.Group(); elbow.position.set(.072,.060,0); shoulder.add(elbow);
    box(elbow,[.040,.040,.046],[0,0,0],mat.black); bar(elbow,.075,.035,-.024,.015,mat.orange); bar(elbow,.075,.035,.024,.015,mat.orange);
    wrist=new T.Group(); wrist.position.set(.075,.035,0); elbow.add(wrist);
    box(wrist,[.036,.036,.044],[0,0,0],mat.black); bar(wrist,.045,-.002,-.022,.013,mat.orange); bar(wrist,.045,-.002,.022,.013,mat.orange);
    grip=new T.Group(); grip.position.set(.045,-.002,0); wrist.add(grip);
    // camera above claw, matching current MuJoCo model
    box(grip,[.038,.012,.052],[.006,.048,0],mat.aluminum);
    box(grip,[.032,.028,.042],[.018,.054,0],mat.black);
    cyl(grip,.010,.016,[.036,.054,0],mat.black,[0,0,Math.PI/2]);
    box(grip,[.036,.024,.012],[.012,0,-.022],mat.aluminum); box(grip,[.036,.024,.012],[.012,0,.022],mat.aluminum);
    leftFinger=box(grip,[.068,.012,.012],[.047,-.025,-.035],mat.orange);
    rightFinger=box(grip,[.068,.012,.012],[.047,-.025,.035],mat.orange);

    // Browser fallback POV follows the centered provisional MuJoCo mount.
    // Main SIM first-person pixels still come from authoritative MuJoCo robot_cam.
    povCamera=new T.PerspectiveCamera(62,16/7,.012,8);
    povCamera.position.set(.067,.0136,0);
    grip.add(povCamera);
    // Three.js cameras look down local -Z. Rotate -Z toward +X, then pitch 10 deg downward.
    povCamera.rotation.order='YXZ';
    povCamera.rotation.y=-Math.PI/2;
    povCamera.rotation.x=7.45917653*Math.PI/180;
  }


  function radarBox(group,size,pos,material){
    const m=new T.Mesh(new T.BoxGeometry(size[0],size[1],size[2]),material); m.position.set(...pos); group.add(m); return m;
  }
  function makeRadarLabelColor(color){
    return ({red:0xff4c4c,blue:0x4f86ff,yellow:0xffd84d}[color]||0x9dffcf);
  }
  function radarTextSprite(text,color='#d8fff0'){
    const c=document.createElement('canvas'); c.width=384; c.height=72; const x=c.getContext('2d');
    x.clearRect(0,0,c.width,c.height); x.font='600 28px ui-monospace, SFMono-Regular, Menlo, monospace'; x.textAlign='center'; x.textBaseline='middle';
    x.fillStyle='rgba(2,9,7,.78)'; x.fillRect(2,9,380,54); x.strokeStyle='rgba(160,255,218,.28)'; x.strokeRect(2.5,9.5,379,53);
    x.fillStyle=color; x.fillText(text,192,37);
    const tex=new T.CanvasTexture(c); tex.minFilter=T.LinearFilter; tex.magFilter=T.LinearFilter;
    const sp=new T.Sprite(new T.SpriteMaterial({map:tex,transparent:true,depthTest:false})); sp.scale.set(.34,.064,1); return sp;
  }
  function clearGroup(g){ while(g && g.children.length){ const c=g.children.pop(); if(c.geometry)c.geometry.dispose(); if(c.material)c.material.dispose(); } }
  function buildRadarScene(){
    radarScene=new T.Scene(); radarScene.background=new T.Color(0x050a08);
    radarScene.add(new T.HemisphereLight(0xb7ffdd,0x07100d,.72));
    const grid=new T.GridHelper(3.2,32,0x31594a,0x173228); grid.position.set(.65,.002,0); radarScene.add(grid);
    // Range rings around the workcell origin provide the sonar/radar feel.
    for(const r of [.25,.5,.75,1.0,1.25,1.5]){
      const pts=[]; for(let i=0;i<=64;i++){const a=i/64*Math.PI*2;pts.push(new T.Vector3(r*Math.cos(a),.006,r*Math.sin(a)));}
      const geo=new T.BufferGeometry().setFromPoints(pts);
      radarScene.add(new T.Line(geo,new T.LineBasicMaterial({color:0x2a6752,transparent:true,opacity:.32})));
    }
    radarObjectGroup=new T.Group(); radarObstacleGroup=new T.Group(); radarZoneGroup=new T.Group(); radarScene.add(radarZoneGroup,radarObstacleGroup,radarObjectGroup);
    radarRobot=new T.Group(); radarScene.add(radarRobot);
    radarBox(radarRobot,[.13,.045,.10],[0,.03,0],new T.MeshStandardMaterial({color:0xdde8e4,roughness:.7}));
    const arrowGeo=new T.ConeGeometry(.05,.12,3); arrowGeo.rotateZ(-Math.PI/2); const arrow=new T.Mesh(arrowGeo,new T.MeshBasicMaterial({color:0x70ffc0})); arrow.position.set(.10,.055,0); radarRobot.add(arrow);
    // Camera FOV is a translucent sector. The thin sweep line animates inside it.
    const range=1.55, fov=74*Math.PI/180;
    const sector=new T.CircleGeometry(range,44,-fov/2,fov); sector.rotateX(-Math.PI/2);
    radarScan=new T.Mesh(sector,new T.MeshBasicMaterial({color:0x36e6a2,transparent:true,opacity:.075,depthWrite:false,side:T.DoubleSide})); radarRobot.add(radarScan);
    const lineGeo=new T.BufferGeometry().setFromPoints([new T.Vector3(0,.018,0),new T.Vector3(range,.018,0)]);
    radarSweep=new T.Line(lineGeo,new T.LineBasicMaterial({color:0x7fffd0,transparent:true,opacity:.75})); radarRobot.add(radarSweep);
  }
  function updateRadarLegend(s){
    const box=document.getElementById('radar-legend'); if(!box)return;
    const mem=s.spatial_memory||{}, sm=s.semantic_map||{}; const bits=[];
    for(const color of ['red','blue','yellow']){
      const m=mem[color]; if(!m)continue;
      bits.push(`<span class="radar-chip ${m.visible?'live':''}">${color.toUpperCase()} ${m.relation||''} · ${Math.round((m.confidence||0)*100)}%</span>`);
    }
    const n=(sm.obstacles||[]).length; if(n)bits.push(`<span class="radar-chip obstacle">OBST ${n}</span>`);
    const z=(sm.drop_zones||[]).length||(currentMode==='sim'?2:0); if(z)bits.push(`<span class="radar-chip zone">ZONE ${z}</span>`); const op=sm.obstacle_provider||{}; if(currentMode==='real'&&op.status==='unconfigured')bits.push(`<span class="radar-chip">DEPTH 준비 전</span>`);
    box.innerHTML=bits.join('');
    const st=document.getElementById('radar-status'); if(st){ const id=(sm.scan||{}).observation_id??s.memory_observation_id??0; st.textContent=`Semantic Radar · ${computeBackend} · camera memory #${id} · obstacles ${n}`; }
  }
  function rebuildRadarSemantic(s){
    if(!radarScene)return; const mem=s.spatial_memory||{}, sm=s.semantic_map||{};
    const obs=(sm.scan||{}).observation_id??s.memory_observation_id??0;
    if(obs===radarObservation && radarObstacleGroup.children.length)return;
    radarObservation=obs; clearGroup(radarObjectGroup); clearGroup(radarObstacleGroup); clearGroup(radarZoneGroup);
    const fallbackZones=[
      {name:'BLUE DELIVERY',center_xy:[.72,-.82],size_xy:[.44,.36]},
      {name:'YELLOW DELIVERY',center_xy:[.72,.82],size_xy:[.44,.36]},
    ];
    const zones=(sm.drop_zones&&sm.drop_zones.length)?sm.drop_zones:(currentMode==='sim'?fallbackZones:[]);
    for(const z of zones){
      const xy=z.center_xy||[0,0], sz=z.size_xy||[.3,.3], isBlue=(z.name||'').includes('BLUE');
      const matz=new T.MeshBasicMaterial({color:isBlue?0x3a68ff:0xffca3a,transparent:true,opacity:.13,depthWrite:false});
      radarBox(radarZoneGroup,[sz[0],.012,sz[1]],[xy[0],.008,-xy[1]],matz);
      const edge=new T.EdgesGeometry(new T.BoxGeometry(sz[0],.012,sz[1])); const lm=new T.LineSegments(edge,new T.LineBasicMaterial({color:isBlue?0x6d91ff:0xffdc70,transparent:true,opacity:.75})); lm.position.set(xy[0],.009,-xy[1]); radarZoneGroup.add(lm);
      const label=radarTextSprite(z.name||'DROP ZONE',isBlue?'#a9c0ff':'#ffe08a'); label.position.set(xy[0],.075,-xy[1]); radarZoneGroup.add(label);
    }
    for(const o of (sm.obstacles||[])){
      const xy=o.position_xy||[0,0], size=Math.max(.05,o.size_m||.08), h=Math.min(.32,Math.max(.035,(o.height_m||.1)*.35));
      const conf=Math.max(.15,Math.min(1,o.confidence||.4));
      radarBox(radarObstacleGroup,[size,h,size],[xy[0],h/2,-xy[1]],new T.MeshStandardMaterial({color:0xff8457,transparent:true,opacity:.25+.55*conf,roughness:.8}));
    }
    for(const [color,m] of Object.entries(mem)){
      const xy=m.position_xy; if(!xy)continue; const c=makeRadarLabelColor(color), live=!!m.visible;
      const pillar=radarBox(radarObjectGroup,[.045,Math.max(.05,m.height_m||.05),.045],[xy[0],Math.max(.025,(m.height_m||.05)/2),-xy[1]],new T.MeshStandardMaterial({color:c,emissive:live?c:0x000000,emissiveIntensity:live?.45:0,transparent:true,opacity:.55+.4*(m.confidence||.5)}));
      const ring=new T.Mesh(new T.RingGeometry(.055,.066,28),new T.MeshBasicMaterial({color:c,transparent:true,opacity:live?.9:.35,side:T.DoubleSide})); ring.rotation.x=-Math.PI/2; ring.position.set(xy[0],.012,-xy[1]); radarObjectGroup.add(ring);
      const label=radarTextSprite(`${color.toUpperCase()} · ${m.relation||'SEEN'}`,color==='red'?'#ff9a9a':color==='blue'?'#9bb8ff':'#ffe591'); label.position.set(xy[0],Math.max(.10,(m.height_m||.05)+.06),-xy[1]); radarObjectGroup.add(label);
    }
    updateRadarLegend(s);
  }
  function setSimOfflineVisuals(offline){
    document.querySelectorAll('.camera.observer, .camera.robot-view').forEach((el)=>el.classList.toggle('offline',!!offline));
    if(offline) updateGraspHud({});
  }
  function updateGraspHud(s){
    const el=document.getElementById('grasp-hud'); if(!el)return;
    if(currentMode!=='sim'){ el.style.display='none'; return; }
    el.style.display='block';
    const l=Number(s?.left_normal_N||0), r=Number(s?.right_normal_N||0);
    const z=Number((s?.red_xyz||[])[2]||0), bilateral=!!s?.bilateral_contact, lifted=!!s?.lifted;
    const closed=Number(s?.gripper_qpos||0)>.001;
    if(bilateral&&lifted){
      el.className='grasp-hud ok';
      el.textContent=`GRIP · BILATERAL · L ${l.toFixed(2)}N / R ${r.toFixed(2)}N · Z ${(z*100).toFixed(1)}cm`;
    } else if(l>0||r>0){
      el.className='grasp-hud warn';
      el.textContent=`GRIP · ONE-SIDED · L ${l.toFixed(2)}N / R ${r.toFixed(2)}N · Z ${(z*100).toFixed(1)}cm`;
    } else {
      el.className='grasp-hud';
      el.textContent=closed?'GRIP · CLOSED · NO CONTACT':'GRIP · OPEN';
    }
  }
  function updateRadar(s){
    if(!radarRobot||!s)return;
    const rxy=s.robot_xy||((s.semantic_map||{}).robot_xy)||[0,0];
    radarRobot.position.set(rxy[0],0,-rxy[1]);
    const scan=(s.semantic_map||{}).scan||{}; const heading=Number.isFinite(scan.heading_rad)?scan.heading_rad:((s.base_yaw||0)+((s.arm_qpos||[0])[0]||0));
    radarRobot.rotation.y=-heading;
    rebuildRadarSemantic(s);
  }
  function initViews() {
    // Observer panels use the authoritative MuJoCo GPU MJPEG streams.
    // Do not redraw them in Three.js: that creates a second, visually inconsistent world.
    const povCanvas=document.getElementById('sim-preview');
    if(povCanvas && povCamera){
      const renderer=new T.WebGLRenderer({canvas:povCanvas,antialias:true,powerPreference:'high-performance'});
      renderer.setPixelRatio(Math.min(devicePixelRatio||1,1.5));
      renderer.shadowMap.enabled=true;
      views.push({canvas:povCanvas,renderer,camera:povCamera,kind:'pov',scene});
    }
    const radarCanvas=document.getElementById('semantic-radar');
    if(radarCanvas){
      buildRadarScene();
      const renderer=new T.WebGLRenderer({canvas:radarCanvas,antialias:true,powerPreference:'high-performance'}); renderer.setPixelRatio(Math.min(devicePixelRatio||1,1.5));
      const camera=new T.PerspectiveCamera(46,1,.02,7); camera.position.set(.62,2.25,1.58); camera.lookAt(.68,0,0);
      views.push({canvas:radarCanvas,renderer,camera,kind:'radar',scene:radarScene});
    }
  }

  function copyState(s) { return JSON.parse(JSON.stringify(s||{})); }
  function angleLerp(a,b,k) { let d=((b-a+Math.PI)%(2*Math.PI))-Math.PI; if(d<-Math.PI)d+=2*Math.PI; return a+d*k; }
  function interpArray(a,b,k,n) { a=a||Array(n).fill(0); b=b||a; return Array.from({length:n},(_,i)=>(a[i]||0)+((b[i]??a[i]??0)-(a[i]||0))*k); }
  function updateModel(s) {
    if(!s||!s.robot_xy) return;
    robot.position.set(s.robot_xy[0],.040,-s.robot_xy[1]); robot.rotation.y=-(s.base_yaw||0);
    const q=s.arm_qpos||[0,0,0,0]; armYaw.rotation.y=-(q[0]||0); shoulder.rotation.z=-(q[1]||0); elbow.rotation.z=-(q[2]||0); wrist.rotation.z=-(q[3]||0);
    if(rightFinger && leftFinger) {
      const close=Math.max(0,Math.min(.012,Number(s.gripper_qpos)||0));
      if(s.gripper_symmetric || String(s.model||'').includes('dynamics_v2')) {
        const half=Math.min(.008,close);
        leftFinger.position.z=-.035+half;
        rightFinger.position.z=.035-half;
      } else {
        leftFinger.position.z=-.035;
        rightFinger.position.z=.035-close;
      }
    }
    for (const [mesh,key] of [[redBlock,'red_xyz'],[blueBlock,'blue_xyz'],[yellowBlock,'yellow_xyz']]) if(s[key]) { const p=mj(s[key]); mesh.position.set(...p); }
    updateRadar(s);
  }
  function stepCurrent() {
    if(currentMode!=='sim'||!target) return;
    if(!current) current=copyState(target);
    const k=.18;
    current.robot_xy=interpArray(current.robot_xy,target.robot_xy,k,2);
    current.base_yaw=angleLerp(current.base_yaw||0,target.base_yaw||0,k);
    current.arm_qpos=interpArray(current.arm_qpos,target.arm_qpos,k,4);
    current.gripper_qpos=(current.gripper_qpos||0)+((target.gripper_qpos||0)-(current.gripper_qpos||0))*k;
    for(const key of ['red_xyz','blue_xyz','yellow_xyz']) if(target[key]) current[key]=interpArray(current[key],target[key],k,3);
    updateModel(current);
  }
  function pingSimActivity(){
    if(!running || currentMode!=='sim' || document.visibilityState!=='visible') return;
    fetch(apiPath('/api/sim/activity'),{method:'POST',headers:{'Content-Type':'application/json'},body:'{}',keepalive:true}).catch(()=>{});
  }
  async function poll() {
    if(!running) return;
    if(document.visibilityState!=='visible'){
      pollTimer=setTimeout(poll,1000);
      return;
    }
    try {
      if(currentMode==='real'){
        const r=await fetch(apiPath('/api/status'),{cache:'no-store'}); const o=await r.json();
        const ws=o.world_state||{};
        const realState={...ws,robot_xy:[0,0],base_yaw:0};
        computeBackend='MASTERPI REAL';
        updateRadarLegend(realState);
        if(radarRobot) updateRadar(realState);
      } else {
        const r=await fetch(apiPath('/api/sim/state'),{cache:'no-store'}); const o=await r.json();
        const provider=String(o.remote_provider||'REMOTE').toUpperCase(); const machine=String(o.remote_machine||'GPU').toUpperCase();
        const recovery=o.gpu_recovery||{}; const authRequired=recovery.state==='auth_required';
        computeBackend=(o.remote_ws_connected&&o.remote_authoritative)?`${provider} ${machine}`:(authRequired?'GPU OFFLINE · LIGHTNING AUTH REQUIRED':'GPU OFFLINE');
        const physicsBadge=document.getElementById('physics-badge');
        const trainingBadge=document.getElementById('training-badge');
        const online=!!(o.remote_ws_connected&&o.remote_authoritative);
        if(typeof window.renderSimPower==='function') window.renderSimPower(online,recovery);
        const pm=String(o.physics_model||o.state?.model||'unknown');
        const isV2=pm.includes('dynamics_v2')||pm.includes('multi_v2');
        setSimOfflineVisuals(!online);
        if(physicsBadge){
          physicsBadge.textContent=!online?'GPU OFFLINE':(isV2?'DYNAMICS V2':'LEGACY PHYSICS');
          physicsBadge.className='sim-badge '+(!online?'bad':(isV2?'ok':'bad'));
        }
        if(trainingBadge){ trainingBadge.textContent=o.training_ready?'TRAINING READY':`TRAINING BLOCKED · CAL ${o.calibration_fit_trials||0}/${o.calibration_held_out_trials||0}`; trainingBadge.className='sim-badge '+(o.training_ready?'ok':'warn'); }
        const camStatus=document.getElementById('cam-status');
        if(camStatus&&currentMode==='sim') camStatus.textContent=online?`로봇 1인칭 · MuJoCo 실제 RGB · ${isV2?'Dynamics V2':'Legacy physics'} · ${provider} ${machine}`:(authRequired?'OFFLINE PREVIEW · Lightning 로그인 필요':'OFFLINE PREVIEW · 원격 MuJoCo worker 없음');
        if(o.ok&&o.state&&o.state.robot_xy){
          target=o.state; workerAge=o.worker_age_s;
          updateGraspHud(o.state);
          updateRadarLegend(o.state);
          if(radarRobot) updateRadar(o.state);
        } else {
          const st=document.getElementById('radar-status'); if(st) st.textContent='Semantic Radar · GPU OFFLINE';
        }
      }
    } catch(e) {}
    pollTimer=setTimeout(poll,currentMode==='real'?1000:((computeBackend||'').startsWith('GPU OFFLINE')?1000:250));
  }
  function render(now) {
    if(!running) return;
    // The physics/state source is independent of this presentation renderer.
    // 30 fps is ample for SIM visualization; REAL only needs a 15 fps radar.
    const minFrameMs=currentMode==='real'?66:33;
    if(document.visibilityState==='visible' && (!lastRenderAt || now-lastRenderAt>=minFrameMs)){
      lastRenderAt=now;
      stepCurrent();
      if(radarSweep){ const a=(now/900)%2; radarSweep.rotation.y=(a<1?(-.5+a):(.5-(a-1)))*74*Math.PI/180; }
      for(const v of views){
        if(currentMode==='real' && v.kind!=='radar') continue;
        const w=v.canvas.clientWidth,h=v.canvas.clientHeight;
        if(w&&h){ const rw=v.canvas.width,rh=v.canvas.height; const pr=Math.min(devicePixelRatio||1,1.0); const nw=Math.floor(w*pr),nh=Math.floor(h*pr); if(rw!==nw||rh!==nh) v.renderer.setSize(w,h,false); v.camera.aspect=w/h; v.camera.updateProjectionMatrix(); v.renderer.render(v.scene||scene,v.camera); }
      }
    }
    raf=requestAnimationFrame(render);
  }
  function setMode(mode){ currentMode=mode==='real'?'real':'sim'; computeBackend=currentMode==='real'?'MASTERPI REAL':'SIM worker'; target=null; current=null; radarObservation=-1; lastRenderAt=0; if(currentMode==='real') setSimOfflineVisuals(false); updateGraspHud({}); }
  let activityTimer=null;
  function start(){
    if(running)return;
    // The SIM dashboard now uses only two authoritative MuJoCo MJPEG feeds.
    // Keep lightweight state/health polling, but do not build or animate hidden
    // WebGL/radar scenes when no presentation canvas exists.
    running=true; poll(); pingSimActivity();
    activityTimer=setInterval(pingSimActivity,30000);
    const graphicsRequested=!!document.getElementById('sim-preview')||!!document.getElementById('semantic-radar');
    if(graphicsRequested){
      try { if(!scene){buildScene();initViews();loadWorkcellScene().catch((e)=>console.warn('UGRP scene XML load failed',e));} render(); }
      catch(e){ console.warn('UGRP optional WebGL initialization failed',e); }
    }
    document.querySelectorAll('.camera.observer .meta').forEach((el)=>{ if(!el.dataset.base) el.dataset.base=el.textContent; el.textContent=el.dataset.base; });
  }
  function stop(){ running=false; if(pollTimer)clearTimeout(pollTimer); if(activityTimer)clearInterval(activityTimer); activityTimer=null; if(raf)cancelAnimationFrame(raf); }
  window.UGRPSim3D={start,stop,setMode};
})();
