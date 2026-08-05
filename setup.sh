#!/bin/bash
# Set up the Pure Pursuit Dashboard + service on a stock Neoracer.
#
# From your laptop, on the same network as the car:
#   scp -r neoracer_pursuit racecar@<car-ip>:~/pursuit
#   ssh racecar@<car-ip> 'bash ~/pursuit/setup.sh'
#
# Installs neoracer-pursuit.service but leaves it OFF: it does not start now
# and does not start at boot, so it takes no port, power, or compute until
# someone turns it on with:
#   bash ~/pursuit/setup.sh enable
# Turn it back off (also permanent across boots) with:
#   bash ~/pursuit/setup.sh disable
#
# Dashboard, when enabled: http://<car-ip>:8084. Idempotent: safe to re-run.
# Needs the stock image (ROS Humble, /home/racecar/ros2_ws) and, for cars
# mode, the driver's inference node publishing on /edgetpu/inference.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEST=/home/racecar/pursuit
SVC=neoracer-pursuit.service

if [[ "${1:-}" == "disable" ]]; then
    sudo systemctl disable --now "$SVC"
    echo "$SVC stopped and disabled. It will not start at boot."
    echo "Re-enable with: bash setup.sh enable"
    exit 0
elif [[ "${1:-}" == "enable" ]]; then
    sudo systemctl enable --now "$SVC"
    echo "$SVC enabled and started: http://$(hostname -I | awk '{print $1}'):8084"
    exit 0
fi

# The service unit expects the files at $DEST. If the script runs from
# somewhere else, copy them over. Keep an existing tuning yaml.
if [[ "$SCRIPT_DIR" != "$DEST" ]]; then
    mkdir -p "$DEST"
    cp "$SCRIPT_DIR/pursuit.py" "$SCRIPT_DIR/pursuit.html" "$SCRIPT_DIR/$SVC" \
       "$SCRIPT_DIR/setup.sh" "$DEST/"
    [[ -f "$DEST/pursuit.yaml" ]] || cp "$SCRIPT_DIR/pursuit.yaml" "$DEST/"
fi

if ! cmp -s "$DEST/$SVC" "/etc/systemd/system/$SVC" 2>/dev/null; then
    sudo install -m 0644 "$DEST/$SVC" "/etc/systemd/system/$SVC"
    sudo systemctl daemon-reload
    echo "  $SVC: installed"
fi

# Off by default. If it is already enabled (a previous `setup.sh enable`),
# an update re-run keeps it enabled and restarts it with the new code.
if systemctl is-enabled --quiet "$SVC" 2>/dev/null; then
    sudo systemctl restart "$SVC"
    echo "$SVC updated and restarted (was enabled)."
else
    echo "$SVC installed but OFF. Start it with: bash ~/pursuit/setup.sh enable"
fi
