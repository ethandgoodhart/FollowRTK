#!/bin/sh
# pi_gps_bridge.sh — run ON the Raspberry Pi that owns the u-blox UART.
#
# The Jetson sees this Pi as USB CDC gadget /dev/gps-bridge. NMEA already
# comes up that tty. RTCM has to go BACK DOWN the same tty onto UART1, which
# only happens if something on the Pi is reading /dev/ttyGS0 and writing
# /dev/ttyAMA0. A one-way `cat UART > GS0` is why NTRIP showed
# "receiver downlink is read-only".
#
# Install (on the Pi, once):
#   sudo cp pi_gps_bridge.sh /usr/local/sbin/pi_gps_bridge.sh
#   sudo chmod +x /usr/local/sbin/pi_gps_bridge.sh
#   sudo cp pi-gps-bridge.service /etc/systemd/system/
#   sudo systemctl enable --now pi-gps-bridge.service
#
# Needs: socat, dwc2 + g_cdc (or equivalent ACM gadget) already loaded.

set -eu
UART="${UART:-/dev/serial0}"
[ -e "$UART" ] || UART=/dev/ttyAMA0
GADGET=/dev/ttyGS0
BAUD="${BAUD:-38400}"

for i in 1 2 3 4 5 6 7 8 9 10; do
  [ -e "$GADGET" ] && [ -e "$UART" ] && break
  sleep 1
done

stty -F "$UART" "$BAUD" raw -echo -echoctl -echoke
stty -F "$GADGET" raw -echo -echoctl -echoke

exec socat -dd \
  "$UART,raw,echo=0,b${BAUD},crnl=0" \
  "$GADGET,raw,echo=0,crnl=0"
