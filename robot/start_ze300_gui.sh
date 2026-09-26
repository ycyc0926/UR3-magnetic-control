#!/usr/bin/env bash
set -euo pipefail

# Include minimized windows so another double-click restores the existing GUI.
show_window() {
  local window
  window=$(xdotool search --name '^ZE300_GUI_V3\.03b$' 2>/dev/null | head -n 1) || true
  [[ -n "$window" ]] || return 1
  xdotool windowmap "$window" windowactivate "$window" || true
}
show_window && exit 0
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/ze300-gui-${UID}.lock"
if ! flock -n 9; then
  zenity --info --timeout=5 --title='ZE300 电机控制' \
    --text='ZE300 正在启动，请稍候。若 30 秒内没有显示窗口，会提示启动失败。' 2>/dev/null || true
  exit 0
fi
show_window && exit 0

log_dir="${XDG_STATE_HOME:-$HOME/.local/state}/ur3"
mkdir -p "$log_dir"
cd '/home/yc/UR3/motor and magnet/ZE300用户资料3.03_20260416/5.上位机/V3.03x'
WINEDEBUG=fixme-all wine ./ZE300_GUI_V3.03b.exe >"$log_dir/ze300-gui.log" 2>&1 9>&- &
gui_pid=$!
for ((attempt=0; attempt<30; attempt++)); do
  if show_window; then
    flock -u 9
    if wait "$gui_pid"; then exit 0; fi
    break
  fi
  kill -0 "$gui_pid" 2>/dev/null || break
  sleep 1
done
# A live Wine process without a window is not a successful launch. Stop only
# this launch, release its lock, and let the next double-click try again.
kill -TERM "$gui_pid" 2>/dev/null || true
wait "$gui_pid" 2>/dev/null || true
flock -u 9
zenity --error --title='ZE300 电机控制' \
  --text="ZE300 未能显示窗口或异常退出，已清理本次启动进程。请重新双击图标。日志：$log_dir/ze300-gui.log" 2>/dev/null || \
  cat "$log_dir/ze300-gui.log" >&2
exit 1
