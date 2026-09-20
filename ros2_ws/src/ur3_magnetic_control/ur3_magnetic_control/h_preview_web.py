"""Vision routes never command motion; the page embeds a separately gated arm panel."""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import queue
import threading
import time


PAGE = '''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>H 机器人 · 实时定位</title>
<style>
*{box-sizing:border-box}body{margin:0;background:#101923;color:#e8eef5;font:16px system-ui,sans-serif}
header{padding:18px 26px;background:#192734;display:flex;gap:20px;align-items:center;flex-wrap:wrap}
h1{font-size:22px;margin:0} .tag{background:#203e3b;color:#86dfb8;border-radius:6px;padding:6px 12px}
main{display:grid;grid-template-columns:minmax(0,1fr) 320px;gap:22px;padding:22px;max-width:1600px;margin:auto}
img{display:block;width:100%;border-radius:8px;background:#283640;cursor:crosshair}
aside{background:#192734;border-radius:8px;padding:22px;height:fit-content}
.value{font-size:28px;font-variant-numeric:tabular-nums;margin:10px 0 24px}.label{color:#aebecb}
button{padding:11px 15px;background:#276b8b;border:0;color:white;border-radius:6px;cursor:pointer;margin:5px 0}
button:disabled{opacity:.5;cursor:wait}#feedback{padding:10px;border:1px solid #537080;border-radius:6px;overflow-wrap:anywhere}
.note{font-size:14px;color:#b0bdc9;line-height:1.7}.ok{color:#86dfb8}.bad{color:#ffad8f}
@media(max-width:900px){main{grid-template-columns:1fr}aside{order:-1}}
</style><header><h1>H 机器人 · 实时定位</h1><span class="tag">视觉区只记录 · 电机须手动启停</span><a href="#arm-control" style="color:#ffbdad;font-weight:bold">下方：机械臂短移按钮 ↓</a></header>
<main><div><img id="view" src="/stream" alt="等待相机图像"><p class="note">绿框：选中的目标　红十字：轮廓中心　黄色线：近期轨迹。点击图中的 H 可重新选择。</p></div>
<aside><div id="state" class="value">连接中…</div><div class="label">世界坐标 / mm</div><div id="xy" class="value">—</div>
<div class="label">相对起点位移 / mm</div><div id="delta" class="value">—</div>
<div id="rate" class="note"></div><p id="spread" class="note"></p>
<p id="feedback" class="note" role="status" aria-live="polite">等待操作；本区按钮只操作视觉记录。</p>
<p id="commanderror" class="note bad" role="alert"></p>
<button id="reset-target" data-endpoint="/reset">重新锁定画面中心目标</button><button id="clear-trail" data-endpoint="/clear">清空轨迹并重设起点</button>
<p class="note">坐标使用现有相机标定，仍需用实物已知距离核验。静止抖动小不等于绝对定位准确；翻滚时轮廓中心会变化。</p>
<hr><div id="record" class="value">录像未启动</div>
<button id="start-recording" data-endpoint="/record/start" data-body='{"duration_s":120}'>开始连续录像（最多 120 秒）</button>
<button id="stop-recording" data-endpoint="/record/stop">结束并保存录像</button>
<p class="note bad">上述按钮只控制录像，绝不启停电机！录像倒计时结束也不会停止电机。目标丢失时须人工停止电机。</p>
<button id="mark-motor-started" data-endpoint="/event" data-body='{"label":"motor_started"}'>标记：电机已启动（仅记时间）</button>
<button id="mark-motor-stopped" data-endpoint="/event" data-body='{"label":"motor_stopped"}'>标记：电机已停止（仅记时间）</button>
<hr><button id="path-negative-y" data-endpoint="/path" data-body='{"kind":"line_negative_y","size_mm":10}'>显示 −Y 10 mm 参考线</button>
<button id="path-positive-y" data-endpoint="/path" data-body='{"kind":"line","size_mm":10}'>显示 +Y 10 mm 参考线</button>
<button id="path-square" data-endpoint="/path" data-body='{"kind":"square","size_mm":24}'>显示 24 mm 方形参考</button>
<button id="hide-path" data-endpoint="/path" data-body='{"kind":"clear"}'>隐藏参考路径</button>
<p id="pathinfo" class="note"></p><p id="diagnostic" class="note"></p>
<p class="note">参考路径以当前 H 为起点，仅用于观察；不代表机械臂已规划、可达或安全。点击重新选择后应重新设置参考路径。</p>
<p class="note">操作顺序：电机停止 → 放置 H 并锁定 → 开始录像 → 确认录像帧数增长 → 现场低速短时启停 → 电机停止后保存录像。</p></aside></main>
<section id="arm-control" style="max-width:1556px;margin:0 auto 30px;padding:24px;background:#302022;border:2px solid #e87865;border-radius:8px">
<h2 style="margin-top:0">真实机械臂 · 单次短移</h2>
<p>下方红色执行按钮会让 UR3 沿世界 −X 移动 3 mm；高度和姿态保持。电机仍须实体启停，页面不能停止电机。</p>
<div id="arm-unavailable"><button id="arm-waiting-button" disabled style="background:#8c3028;padding:18px;font-size:20px">执行 −X 3 mm（未准备，不能点击）</button><p id="arm-availability" role="status">等待本次机械臂准备。请保持电机关闭；准备完成后按钮会自动出现在这里。</p></div>
<iframe id="arm-panel" title="真实机械臂短移控制" style="display:none;width:100%;height:640px;border:0;background:white;border-radius:6px"></iframe>
</section>
<script>
(() => {
'use strict';
// The iframe is a separate explicitly labeled control surface. Vision routes
// and motor event markers never trigger arm motion.
const armPanelUrl = ARM_PANEL_URL;
const armFrame = document.getElementById('arm-panel');
const armUnavailable = document.getElementById('arm-unavailable');
let lastArmHeartbeat = 0, lastArmReload = 0;
window.addEventListener('message', event => {
  if (!armPanelUrl || event.origin !== new URL(armPanelUrl).origin || event.source !== armFrame.contentWindow || event.data?.kind !== 'ur3-probe-availability') return;
  if (event.data.available === true) {
    lastArmHeartbeat = Date.now(); armFrame.style.display='block'; armUnavailable.style.display='none';
  } else { lastArmHeartbeat=0; armFrame.style.display='none'; armUnavailable.style.display='block'; }
});
function refreshArmPanel() {
  if (!armPanelUrl) return;
  const now=Date.now();
  if (now-lastArmHeartbeat>2000) {
    armFrame.style.display='none';armUnavailable.style.display='block';
    if (now-lastArmReload>3000) { lastArmReload=now;armFrame.src=armPanelUrl; }
  }
}
setInterval(refreshArmPanel,500);refreshArmPanel();
// No inline handlers or implicit DOM globals: button.command is a native string property.
const elements = {};
for (const id of ['view','state','xy','delta','rate','spread','record','diagnostic','pathinfo','commanderror','feedback']) elements[id] = document.getElementById(id);
const {view,state,xy,delta,rate,spread,record,diagnostic,pathinfo,commanderror,feedback} = elements;
async function sendVisionRequest(url, body, label, button=null) {
  if (button) button.disabled = true;
  feedback.textContent = '正在提交：'+label;
  feedback.className = 'note';
  const controller = new AbortController();
  const deadline = setTimeout(() => controller.abort(), 3000);
  try {
    const response = await fetch(url, {method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body),signal:controller.signal});
    if (!response.ok) throw Error('请求失败 HTTP '+response.status);
    feedback.textContent = '已提交：'+label+'。请核对下方实际状态；录像须确认帧数增长。';
    feedback.className = 'note ok';
  } catch (error) {
    feedback.textContent = error.name==='AbortError' ? '请求超时，结果未确认；请检查实际状态，勿反复点击。' : error.message;
    feedback.className = 'note bad';
  } finally {
    clearTimeout(deadline);
    if (button) button.disabled = false;
  }
}
for (const button of document.querySelectorAll('button[data-endpoint]')) {
  button.addEventListener('click', () => sendVisionRequest(button.dataset.endpoint, JSON.parse(button.dataset.body || '{}'), button.textContent, button));
}
view.addEventListener('click', e => {
  const r = view.getBoundingClientRect();
  sendVisionRequest('/select', {x:(e.clientX-r.left)/r.width*1224,y:(e.clientY-r.top)/r.height*1024}, '选择图中目标');
});
const pathNames = {line:'+Y 直线',line_negative_y:'−Y 直线',square:'方形'};
let updating = false;
async function update(){if(updating)return;updating=true;const controller=new AbortController();const deadline=setTimeout(()=>controller.abort(),2000);try{let r=await fetch('/status',{cache:'no-store',signal:controller.signal});if(!r.ok)throw Error('状态请求失败');let s=await r.json();let valid=s.detected&&!s.stream_stale&&Date.now()-Date.parse(s.utc)<1500;
state.textContent=valid?'● 已锁定 H': '● 目标丢失 / 图像超时';state.className='value '+(valid?'ok':'bad');
xy.textContent=valid?s.world_xy_mm.map(v=>v.toFixed(2)).join(' , '):'—';delta.textContent=valid?s.displacement_from_reset_mm.map(v=>v.toFixed(2)).join(' , '):'—';
rate.textContent=(s.fps||0).toFixed(1)+' 帧/秒 · 连续检测 '+(s.consecutive_detections||0)+' 帧';
spread.textContent=s.window_std_mm?'近期坐标标准差：'+s.window_std_mm.map(v=>v.toFixed(3)).join(' , ')+' mm（仅静止时可作抖动参考）':'';
let rec=s.recording||{};record.textContent=rec.active?'● 正在录像 '+rec.frames+' 帧 / 剩余 '+Math.ceil(rec.seconds_remaining)+' 秒':(rec.error?'录像错误：'+rec.error:'录像已停止 · '+(rec.frames||0)+' 帧');
record.className='value '+(rec.active?'ok':'bad');
diagnostic.textContent='检测诊断：'+(s.tracking_reason||'等待图像');
const rejected=(s.tracking_diagnostics||{}).rejected||{};
if (!valid && rejected.shape_or_border) diagnostic.textContent+='；目标可能靠近边缘或轮廓不符合条件，请勿继续驱动。';
pathinfo.textContent=s.reference_path?'当前参考：'+(pathNames[s.reference_path.kind]||s.reference_path.kind)+'，'+s.reference_path.size_mm+' mm（仅显示）':'尚未设置参考路径';
commanderror.textContent=s.command_error?'后台未执行：'+s.command_error:'';
}catch(e){state.textContent='● 未连接';state.className='value bad';xy.textContent='—';delta.textContent='—';record.textContent='录像状态未知（连接中断）；本页面不能停止电机';record.className='value bad'}finally{clearTimeout(deadline);updating=false}}setInterval(update,500);update();
})();
</script></html>'''


class LocalPreview:
    def __init__(self, port=8767, arm_panel_url=None):
        self.status = b'{}'
        self.jpeg = None
        self.commands = queue.Queue(maxsize=32)
        owner = self
        # Isolated tests on an ephemeral port never connect to the real panel.
        panel_url = arm_panel_url if arm_panel_url is not None else ('http://127.0.0.1:8768/' if port == 8767 else '')
        page = PAGE.replace('ARM_PANEL_URL', json.dumps(panel_url))

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_):
                pass

            def reply(self, body, content_type):
                self.send_response(200)
                self.send_header('Content-Type', content_type)
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers();self.wfile.write(body)

            def do_GET(self):
                if self.path == '/':
                    self.reply(page.encode('utf-8'), 'text/html; charset=utf-8')
                elif self.path == '/status':
                    self.reply(owner.status, 'application/json')
                elif self.path == '/stream':
                    self.send_response(200)
                    self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                    self.send_header('Cache-Control', 'no-store');self.end_headers()
                    try:
                        while True:
                            frame = owner.jpeg
                            if frame is not None:
                                self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: '+str(len(frame)).encode()+b'\r\n\r\n'+frame+b'\r\n')
                                self.wfile.flush()
                            time.sleep(.2)
                    except (BrokenPipeError, ConnectionResetError):
                        pass
                else:
                    self.send_error(404)

            def do_POST(self):
                # Reject cross-origin requests to the local control UI.
                origin = f'http://127.0.0.1:{self.server.server_address[1]}'
                if self.headers.get('Origin', origin) != origin:
                    self.send_error(403);return
                if self.path not in ('/reset', '/clear', '/select', '/record/start',
                                     '/record/stop', '/event', '/path'):
                    self.send_error(404);return
                try:
                    length = int(self.headers.get('Content-Length', '0'))
                    if not 0 <= length <= 256:raise ValueError('request size')
                    body = json.loads(self.rfile.read(length) or b'{}')
                    if not isinstance(body, dict):raise ValueError('object required')
                    if self.path == '/select':
                        x, y = float(body['x']), float(body['y'])
                        if not 0 <= x < 1224 or not 0 <= y < 1024:raise ValueError('pixel range')
                        command = ('select', (x, y))
                    elif self.path == '/record/start':
                        duration = float(body.get('duration_s', 120))
                        if not math.isfinite(duration) or not 5 <= duration <= 180:raise ValueError('duration')
                        command = ('record_start', duration)
                    elif self.path == '/record/stop':
                        command = ('record_stop', None)
                    elif self.path == '/event':
                        label = body.get('label')
                        if label not in ('motor_started', 'motor_stopped'):raise ValueError('event')
                        command = ('event', label)
                    elif self.path == '/path':
                        kind, size = body.get('kind'), float(body.get('size_mm', 20))
                        if kind not in ('line', 'line_negative_y', 'square', 'clear'):raise ValueError('path')
                        if not math.isfinite(size) or not 5 <= size <= 40:raise ValueError('path size')
                        command = ('path', (kind, size))
                    else:
                        command = (self.path[1:], None)
                    owner.commands.put_nowait(command)
                    self.reply(b'{"ok":true}', 'application/json')
                except queue.Full:
                    self.send_error(429)
                except (KeyError, ValueError, TypeError):
                    self.send_error(400)

        self.server = ThreadingHTTPServer(('127.0.0.1', port), Handler)
        self.server.daemon_threads = True
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def pop_commands(self):
        while True:
            try:
                yield self.commands.get_nowait()
            except queue.Empty:
                return

    def close(self):
        self.server.shutdown();self.server.server_close()
