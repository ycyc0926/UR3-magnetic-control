"""Independent timestamped RTDE output recorder; no motion or input recipes."""
import json
from pathlib import Path
import socket
import threading
import time

import ur3_realtime_monitor as rtde


class OutputRecorder:
    def __init__(self,path):
        self.path=Path(path)
        self.latest=None;self.error=None;self.count=0
        self.phase='not started'
        self.first=threading.Event();self.ending=threading.Event()
        self.thread=threading.Thread(target=self._record,name='read_only_rtde_recorder',daemon=True)

    def _record(self):
        try:
            with self.path.open('x',encoding='utf-8') as stream:
                self.phase='connecting'
                with socket.create_connection((rtde.ROBOT_IP,30004),timeout=3.) as sock:
                    self.phase='negotiating outputs'
                    recipe,version=rtde.negotiate(sock,125)
                    self.version=version
                    sock.settimeout(.25)
                    self.phase='streaming outputs'
                    try:
                        while not self.ending.is_set():
                            state=rtde.parse_state(rtde.receive_command(sock,rtde.RTDE_DATA_PACKAGE),recipe)
                            entry={'host_monotonic_s':time.monotonic(),'host_time_ns':time.time_ns(),'state':state}
                            stream.write(json.dumps(entry,allow_nan=False)+'\n');stream.flush()
                            self.latest=entry;self.count+=1;self.first.set()
                    finally:
                        # Pauses this output client only, not the robot program.
                        rtde.send_packet(sock,rtde.RTDE_CONTROL_PACKAGE_PAUSE)
        except Exception as error:
            self.error=f'{self.phase}: {error}';self.first.set()

    def start(self):
        self.thread.start()
        if not self.first.wait(5.):raise RuntimeError('Direct RTDE recording did not start')
        self.check_fresh(.12)

    def check_fresh(self,max_age):
        if self.error is not None:raise RuntimeError('Direct RTDE recording failed: '+self.error)
        if self.latest is None or time.monotonic()-self.latest['host_monotonic_s']>max_age:
            raise RuntimeError('Direct RTDE recorder stale')

    def close(self):
        self.ending.set();self.thread.join(2.)
        if self.thread.is_alive():self.error='Recorder did not finish within deadline'

    def summary(self):
        return {'path':str(self.path),'samples':self.count,'error':self.error,
                'robot_writes':'RTDE output negotiation/start/pause only; no motion or inputs'}
