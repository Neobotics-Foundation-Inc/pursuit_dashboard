# Pure Pursuit Dashboard

Target-following as a service for the Neoracer, on port 8083. The car chases whatever the driver's YOLO inference node detects: PD steering on the target's horizontal offset, throttle that brakes as the target's bounding box grows toward a stop size.

## Contents

- [Install](#install)
- [Service control](#service-control)
- [Detection and control](#detection-and-control)
- [Tuning](#tuning)
- [Known conflicts on the car](#known-conflicts-on-the-car)
- [Safety](#safety)

## Install

On the car:

```
git clone https://github.com/Neobotics-Foundation-Inc/pursuit_dashboard.git
bash pursuit_dashboard/setup.sh
```

setup.sh points neoracer-pursuit.service at this checkout wherever it sits and copies nothing, so the repository can live anywhere the racecar user can read. A first install leaves the service stopped and disabled; start it with `bash setup.sh enable`. Dashboard: `http://<car-ip>:8083`.

Re-running setup.sh updates the unit and leaves the enable state alone: a running service restarts on the new code, a stopped one stays stopped.

## Service control

Run on the car, from the checkout:

| Command | Effect |
| --- | --- |
| `bash setup.sh` | install or update the unit; a first install does not start it |
| `bash setup.sh enable` | start now and at every boot |
| `bash setup.sh disable` | stop now and keep off across boots |
| `bash setup.sh restart` | restart; takes port 8083 back first |
| `bash setup.sh remove` | stop, disable, and uninstall the unit; files here are kept |

Enable, restart, and an update of a running service clear port 8083 first. A dashboard left over from an earlier install under a different unit name or directory, or any other service on 8083, is stopped through systemd; a `pursuit.py` started by hand is signalled directly. Without this the new instance would fail to bind and loop on `Restart=on-failure`.

After `remove`, the checkout and its tuned pursuit.yaml stay in place; `bash setup.sh` reinstalls.

## Detection and control

Detections come from the driver inference node (vision_msgs/Detection2DArray on /edgetpu/inference). Detections below score_threshold are dropped, the best-scoring max_detections remain as candidates, and the largest box is the target. The model being served and the camera frame rate show in the top bar; the model is whatever model_path in the driver's inference.yaml points to.

If no object is found the car KEEPS THE LAST ACTUATED COMMAND, speed and angle both, republished continuously. It does not stop on its own except by the safety_area brake or the speed slider at zero; the transmitter's SWB switch is the physical gate.

## Tuning

All on the dashboard, live: speed (throttle cap, drag slider, applies while dragging; it moves only with a sideways drag, never with a tap or a scroll that starts on it, and always shows the speed the car has), speed_kp (stopping gain), safety_area (bbox area fraction where the car fully stops; the yellow dashed outline on the camera view is this size), angle_kp, angle_kd, score_threshold, max_detections. Save writes pursuit.yaml, Reset re-reads it.

The page works with a mouse, a touch screen or the keyboard, on a laptop, a tablet or a phone, in a light or a dark theme (the switch in the top bar). The number fields have − and + buttons that repeat while held.

## Known conflicts on the car

Anything else publishing /drive will fight this service at the mux and the car will sit still or stutter:

- neoracer-autonomy runs a twist bridge that idles at zero on /drive. Disable it while using pursuit: `sudo systemctl disable --now neoracer-autonomy`
- A leftover Jupyter kernel that ever created a racecar object keeps publishing /drive. Restart the jupyter service to clear them.

Check with: `ros2 topic info -v /drive` (there should be exactly one publisher: pursuit).

## Safety

The neoracer mux forwards /drive with no software deadman; the SWB switch is the physical autonomy gate. The shipped yaml has speed 0.0, so the car cannot drive until the slider is raised. Speed is hard capped at 1.0. Remember the keep-last rule: a car that loses its target mid-turn keeps turning until the switch is flipped or the box reappears.
