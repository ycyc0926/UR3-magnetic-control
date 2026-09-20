"""One-shot localhost UI whose button explicitly commands a reviewed arm segment.

This never drives the motor. It is separate from the vision-only recorder UI.
"""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import secrets
import threading
import time


class ExplicitProbeTrigger:
    def __init__(self, timeout_s=90., port=8768):
        self.token = secrets.token_urlsafe(24)
        self.deadline = time.monotonic()+timeout_s
        self.start_event = None
        self.abort_requested = False
        self.lock = threading.Lock()
        owner = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_):
                pass

            def do_GET(self):
                if self.path == '/status':
                    body = json.dumps({'available': time.monotonic() <= owner.deadline,
                        'can_start': owner.start_event is None and not owner.abort_requested and time.monotonic() <= owner.deadline,
                        'started': owner.start_event is not None, 'aborted': owner.abort_requested,
                        'seconds_remaining': max(0., owner.deadline-time.monotonic())}).encode()
                    self.send_response(200); self.send_header('Content-Type', 'application/json')
                    self.send_header('Content-Length', str(len(body))); self.send_header('Cache-Control', 'no-store')
                    self.end_headers(); self.wfile.write(body); return
                if self.path != '/':
                    self.send_error(404); return
                page = '''<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<title>UR3 单次短移：真实机械臂控制</title><body style="font:20px sans-serif;max-width:850px;margin:20px auto;padding:10px">
<h1>真实机械臂：世界 −X 3 mm</h1>
<p>仅此一次：运动 5 秒，含停留共 5.5 秒；高度和姿态保持。</p>
<p>请先确认人员撤离、电机 10 rpm、沿用刚才反转档。手动开启电机后，立即点击下面按钮；电机最多约 6 秒后必须实体关闭。</p>
<p style="color:#b00020"><b>点击按钮会使真实机械臂运动。页面不能停止电机！异常时立即实体停机；机械臂异常使用现场停止方案。</b></p>
<button id="start" style="font:inherit;padding:18px;background:#a32612;color:white" onclick="send('/start')">执行已批准的 −X 3 mm 短移</button>
<button style="font:inherit;padding:18px" onclick="send('/abort')">中止机械臂；电机仍须实体关闭</button>
<p id="status">等待执行；90 秒内未点击将取消本次准备。</p>
<script>let submitted=false;async function send(path){submitted=true;document.getElementById('start').disabled=true;
try{let r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({token:'TOKEN'})});
document.getElementById('status').textContent=r.ok?(path==='/start'?'已提交本次短移。请按时实体关闭电机，不要重复启动。':'已请求中止。请立即实体关闭电机。'):'请求未接受，请保持电机关闭。';}
catch(e){document.getElementById('status').textContent='连接中断：立即实体关闭电机，检查机械臂状态。';}}
let polling=false;async function poll(){if(polling)return;polling=true;const c=new AbortController();const t=setTimeout(()=>c.abort(),800);
try{let r=await fetch('/status',{cache:'no-store',signal:c.signal});if(!r.ok)throw Error('status');let s=await r.json();
document.getElementById('start').disabled=submitted||!s.can_start;
parent.postMessage({kind:'ur3-probe-availability',available:s.available},'*');
if(!submitted)document.getElementById('status').textContent=s.can_start?'准备完成，可执行本次短移；剩余 '+Math.ceil(s.seconds_remaining)+' 秒。':'本次不可再执行，请保持电机关闭。';}
catch(e){document.getElementById('start').disabled=true;parent.postMessage({kind:'ur3-probe-availability',available:false},'*');document.getElementById('status').textContent='控制连接已结束，请保持电机关闭。';}
finally{clearTimeout(t);polling=false;}}setInterval(poll,500);poll();
</script></body></html>'''.replace('TOKEN', owner.token).encode()
                self.send_response(200); self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.send_header('Content-Length', str(len(page))); self.send_header('Cache-Control', 'no-store')
                self.end_headers(); self.wfile.write(page)

            def do_POST(self):
                if self.headers.get('Origin') != f'http://127.0.0.1:{self.server.server_address[1]}':
                    self.send_error(403); return
                try:
                    size = int(self.headers.get('Content-Length', '0'))
                    if not 0 < size <= 256: raise ValueError('length')
                    value = json.loads(self.rfile.read(size))
                    ok = owner.accept(self.path, value.get('token'), time.monotonic(), time.time_ns())
                except (ValueError, TypeError, AttributeError):
                    ok = False
                self.send_response(202 if ok else 409); self.end_headers()

        self.server = ThreadingHTTPServer(('127.0.0.1', port), Handler)
        self.server.daemon_threads = True
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()

    def accept(self, endpoint, token, monotonic_now, wall_ns):
        with self.lock:
            if token != self.token:
                return False
            if endpoint == '/abort':
                self.abort_requested = True
                return True
            if monotonic_now > self.deadline:
                return False
            if endpoint != '/start' or self.abort_requested or self.start_event is not None:
                return False
            self.start_event = {'event': 'explicit_arm_probe_start', 'wall_time_ns': wall_ns,
                                'source': 'operator_explicit_arm_control_page_not_motor_feedback'}
            return True

    def ready(self, now_ns):
        if self.abort_requested: raise RuntimeError('Operator aborted probe')
        if time.monotonic() > self.deadline: raise RuntimeError('Explicit start window expired')
        if self.start_event is None: return False
        if not 0 <= (now_ns-self.start_event['wall_time_ns'])/1e9 <= .5:
            raise RuntimeError('Explicit start request stale or future')
        return True

    def check_during(self, now_ns):
        if self.abort_requested: raise RuntimeError('Operator requested arm stop; physically stop motor')
        if self.start_event is None or not 0 <= (now_ns-self.start_event['wall_time_ns'])/1e9 <= 6.:
            raise RuntimeError('Six-second arm probe window ended; physically stop motor')

    def close(self):
        self.server.shutdown(); self.server.server_close(); self.thread.join(1.)
