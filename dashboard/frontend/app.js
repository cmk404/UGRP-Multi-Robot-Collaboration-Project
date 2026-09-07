const $ = (s) => document.querySelector(s);
const state = { connected: false, busy: false };
const actionButtons = [...document.querySelectorAll('.motion'), $('#move-servo')];
const detailText = { ready: '컨트롤 보드 준비됨', checking: '다른 상태 확인 또는 명령 진행 중', motion_active: '주행 명령 실행 중', ssh_unreachable: 'SSH 연결 안 됨', controller_unreachable: 'I²C 컨트롤러 응답 없음', timeout: '응답 시간 초과' };
const keys = { w: 'forward', ArrowUp: 'forward', s: 'backward', ArrowDown: 'backward', a: 'left', ArrowLeft: 'left', d: 'right', ArrowRight: 'right' };

function notice(text, kind = '') {
  $('#notice').textContent = text;
  $('#notice').className = `notice ${kind}`.trim();
}

function enableActions() {
  actionButtons.forEach((button) => { button.disabled = !state.connected || state.busy; });
}

async function api(path, options = {}) {
  const response = await fetch(path, { ...options, headers: options.body ? { 'Content-Type': 'application/json' } : undefined });
  const body = await response.json();
  if (!response.ok || body.success === false) throw new Error(body?.error?.message || `요청 실패 (${response.status})`);
  return body;
}

function renderStatus(status) {
  state.connected = status.connected === true;
  $('#connection').className = `connection ${state.connected ? 'online' : 'offline'}`;
  $('#connection-label').textContent = state.connected ? '제어 가능' : '연결 안 됨';
  $('#connection-detail').textContent = detailText[status.detail] || status.detail || '상태 불명';
  $('#address').textContent = status.address || status.target || '—';
  $('#controller').textContent = status.probe?.controller || '—';
  $('#battery').textContent = Number.isFinite(status.probe?.battery_mv) ? `${(status.probe.battery_mv / 1000).toFixed(2)} V` : '—';
  enableActions();
}

async function refreshStatus(quiet = false) {
  if (!quiet) notice('Pi와 컨트롤 보드를 확인하고 있어요.');
  try {
    const status = await api('/api/status');
    renderStatus(status);
    if (!quiet) {
      const readyMessage = status.target === 'mock'
        ? '모의 전송 계층이 응답했어요.'
        : 'SSH와 I²C 컨트롤러가 모두 응답했어요.';
      notice(state.connected ? readyMessage : $('#connection-detail').textContent, state.connected ? 'success' : 'error');
    }
  } catch (error) {
    renderStatus({ connected: false, detail: 'ssh_unreachable' });
    if (!quiet) notice(error.message, 'error');
  }
}

async function refreshEvents() {
  try {
    const { events = [] } = await api('/api/events');
    const list = $('#events');
    list.replaceChildren();
    [...events].reverse().slice(0, 10).forEach((event) => {
      const li = document.createElement('li');
      const time = document.createElement('time');
      const text = document.createElement('span');
      time.textContent = new Date(event.at).toLocaleTimeString('ko-KR', { hour12: false });
      text.textContent = `${event.kind}: ${event.detail}${event.address ? ` · ${event.address}` : ''}`;
      li.append(time, text); list.append(li);
    });
    if (!list.children.length) { const li = document.createElement('li'); li.textContent = '아직 기록된 명령이 없어요.'; list.append(li); }
    $('#last-updated').textContent = `갱신 ${new Date().toLocaleTimeString('ko-KR', { hour12: false })}`;
  } catch (_) { /* connection state is shown above */ }
}

async function command(path, payload, success) {
  state.busy = true; enableActions(); notice('명령 전달 중…');
  try {
    await api(path, { method: 'POST', body: JSON.stringify(payload) });
    notice(success, 'success'); await refreshEvents();
  } catch (error) {
    notice(error.message, 'error'); await refreshStatus(true);
  } finally { state.busy = false; enableActions(); }
}

function drive(direction) {
  return command('/api/drive', { direction, speed: Number($('#speed').value), duration: Number($('#drive-duration').value) }, `${direction} 명령을 전달했어요.`);
}

async function stop() {
  notice('정지 명령 전달 중…');
  try { await api('/api/stop', { method: 'POST' }); notice('Pi가 모터 정지를 확인했어요.', 'success'); await refreshEvents(); }
  catch (error) { notice(`정지 확인 실패: ${error.message}`, 'error'); }
}

document.querySelectorAll('.motion').forEach((b) => b.addEventListener('click', () => drive(b.dataset.direction)));
document.querySelectorAll('.preset').forEach((b) => b.addEventListener('click', () => { $('#pulse').value = b.dataset.pulse; $('#pulse').dispatchEvent(new Event('input')); }));
$('#speed').addEventListener('input', () => { $('#speed-value').textContent = $('#speed').value; });
$('#drive-duration').addEventListener('input', () => { $('#drive-duration-value').textContent = Number($('#drive-duration').value).toFixed(2).replace(/0$/, ''); });
$('#pulse').addEventListener('input', () => { $('#pulse-value').textContent = $('#pulse').value; });
$('#arm-duration').addEventListener('input', () => { $('#arm-duration-value').textContent = Number($('#arm-duration').value).toFixed(1); });
$('#refresh').addEventListener('click', () => refreshStatus());
$('#stop').addEventListener('click', stop);
$('#move-servo').addEventListener('click', () => command('/api/arm', { servo: Number($('#servo').value), pulse: Number($('#pulse').value), duration: Number($('#arm-duration').value) }, `서보 ${$('#servo').value} 이동을 Pi가 확인했어요.`));
document.addEventListener('keydown', (event) => {
  if (event.repeat || event.target.matches('input,select')) return;
  if (event.code === 'Space') { event.preventDefault(); stop(); return; }
  const direction = keys[event.key] || keys[event.key.toLowerCase()];
  if (direction && state.connected && !state.busy) { event.preventDefault(); drive(direction); }
});

enableActions();
refreshStatus(); refreshEvents();
setInterval(() => { refreshStatus(true); refreshEvents(); }, 10000);
