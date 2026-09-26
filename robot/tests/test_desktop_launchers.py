"""Launcher regression checks using fake commands; no hardware or real windows."""
import os
import fcntl
from pathlib import Path
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[2]


def test_desktop_launchers():
    with tempfile.TemporaryDirectory() as directory:
        folder = Path(directory)
        calls = folder / 'calls'
        env = dict(os.environ, PATH=f'{folder}:/usr/bin:/bin',
                   XDG_RUNTIME_DIR=directory, XDG_STATE_HOME=directory)

        def fake(name, body):
            command = folder / name
            command.write_text('#!/bin/bash\nprintf "%s\\n" "' + name
                               + ' $*" >> "' + str(calls) + '"\n' + body + '\n')
            command.chmod(0o755)

        def run(script):
            calls.write_text('')
            result = subprocess.run(['bash', str(ROOT / script)], env=env,
                                    capture_output=True, text=True, timeout=5)
            return result.returncode, calls.read_text()

        for name in ('xdg-open', 'zenity', 'lsusb', 'arv-viewer-0.8', 'wine'):
            fake(name, 'exit 0')
        fake('pgrep', 'exit 0')
        code, log = run('camera/start_flir_camera.sh')
        assert code == 0 and 'xdg-open http://127.0.0.1:8767/' in log
        assert 'arv-tool' not in log

        fake('pgrep', 'exit 1')
        fake('arv-tool-0.8', 'echo "Failed to open device: LIBUSB_ERROR_BUSY" >&2; exit 0')
        code, log = run('camera/start_flir_camera.sh')
        assert code == 1 and 'zenity --error' in log
        assert 'arv-viewer' not in log
        fake('arv-tool-0.8', 'exit 0')
        code, log = run('camera/start_flir_camera.sh')
        assert code == 0 and 'arv-viewer-0.8 --usb-mode=async' in log

        fake('xdotool', 'if [[ "$1" == search ]]; then echo 123; fi')
        code, log = run('robot/start_ze300_gui.sh')
        assert code == 0 and 'windowmap 123 windowactivate 123' in log and 'wine ' not in log
        assert '--onlyvisible' not in log  # Minimized windows must also be restored.
        ready = folder / 'ready'
        fake('xdotool', f'if [[ "$1" == search ]]; then [[ -f "{ready}" ]] || exit 1; echo 123; fi')
        fake('wine', f'touch "{ready}"; /bin/sleep 0.2')
        fake('sleep', '/bin/sleep 0.01')
        code, log = run('robot/start_ze300_gui.sh')
        assert code == 0 and 'wine ./ZE300_GUI_V3.03b.exe' in log
        assert 'windowmap 123 windowactivate 123' in log
        ready.unlink()
        fake('xdotool', 'exit 1')
        fake('wine', 'exit 1')
        code, log = run('robot/start_ze300_gui.sh')
        assert code == 1 and 'zenity --error' in log
        fake('wine', 'exec /bin/sleep 60')
        code, log = run('robot/start_ze300_gui.sh')
        assert code == 1 and 'zenity --error' in log  # Hung startup is stopped, not left locked.
        lock_path = folder / f'ze300-gui-{os.getuid()}.lock'
        with lock_path.open('w') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            code, log = run('robot/start_ze300_gui.sh')
            assert code == 0 and 'zenity --info' in log and 'wine ' not in log


if __name__ == '__main__':
    test_desktop_launchers()
