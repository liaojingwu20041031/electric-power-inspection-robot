#!/usr/bin/env bash
set -euo pipefail

iface="${1:-can1}"
bitrate="${2:-500000}"
protocol_dir="/home/nvidia/ros2_DL/官方通信协议"

echo "Setting up ${iface} for ZLAC8015D CANopen at ${bitrate} bps"

sudo modprobe can || true
sudo modprobe can_raw || true

if ip link show "${iface}" >/dev/null 2>&1; then
  sudo ip link set "${iface}" down || true
fi

sudo ip link set "${iface}" type can bitrate "${bitrate}" berr-reporting on restart-ms 100
sudo ip link set "${iface}" up

echo
ip -details link show "${iface}"
echo
echo "Protocol documents are local-only and ignored by git:"
echo "  ${protocol_dir}"
echo
echo "Quick checks:"
echo "  candump -tz ${iface}"
echo "  cansend ${iface} 000#0101"
