#!/usr/bin/env bash
set -euo pipefail

camera_id="1e10:4000"

# The tracking service already owns the USB camera; reuse its live preview.
if pgrep -f '^/usr/bin/python3 -m ur3_magnetic_control.h_tracker_node( |$)' >/dev/null; then
  exec xdg-open 'http://127.0.0.1:8767/'
fi

if ! lsusb -d "${camera_id}" >/dev/null 2>&1; then
  zenity --error --title="FLIR 相机" \
    --text="没有检测到 Blackfly S 相机。请确认相机连接在 USB 3.x 接口。" 2>/dev/null || \
    echo "没有检测到 Blackfly S 相机。请确认 USB 连接。" >&2
  exit 1
fi

if pgrep -x SpinView_QT >/dev/null 2>&1; then
  zenity --warning --title="FLIR 相机" \
    --text="SpinView 正在占用相机。请先关闭 SpinView，再启动全分辨率查看器。" 2>/dev/null || \
    echo "SpinView 正在占用相机，请先关闭它。" >&2
  exit 1
fi

# Restore the camera's full sensor area and free-running acquisition. These
# settings are volatile; no firmware or saved UserSet is modified.
if ! camera_error=$(arv-tool-0.8 control \
  TriggerMode=Off \
  Width=8 Height=6 \
  OffsetX=0 OffsetY=0 \
  Width=2448 Height=2048 \
  PixelFormat=BayerRG8 2>&1) || [[ "$camera_error" == *"Failed"* ]]; then
  # Aravis 0.8 can print a USB-open failure while still returning exit code 0.
  zenity --error --title="FLIR 相机" \
    --text="相机初始化失败。请先关闭其他占用相机的程序。\n${camera_error}" 2>/dev/null || \
    printf '%s\n' "$camera_error" >&2
  exit 1
fi

exec arv-viewer-0.8 --usb-mode=async
