# Pure Pursuit Dashboard

Target-following as a service for the Neoracer, on port 8083. The car chases whatever the driver's YOLO inference node detects: PD steering on the target's horizontal offset, throttle that brakes as the target's bounding box grows toward a stop size.

## Install

From a laptop on the car's network:

```
scp -r pursuit_dashboard racecar@<car-ip>:~/pursuit
ssh racecar@<car-ip> 'bash ~/pursuit/setup.sh'
```

Setup installs, enables at boot, and starts the service in one step. Dashboard: `http://<car-ip>:8083`.

```
bash ~/pursuit/setup.sh disable    # stop now and stay off across boots
bash ~/pursuit/setup.sh enable     # turn it back on
```

## How it works

Detections come from the driver inference node (vision_msgs/Detection2DArray on /edgetpu/inference). Detections below score_threshold are dropped, the best-scoring max_detections remain as candidates, and the largest box is the target. The model being served and the camera frame rate show in the top bar; the model is whatever model_path in the driver's inference.yaml points to.

If no object is found the car KEEPS THE LAST ACTUATED COMMAND, speed and angle both, republished continuously. It does not stop on its own except by the safety_area brake or the speed slider at zero; the transmitter's SWB switch is the physical gate.

## Tuning

All on the dashboard, live: speed (throttle cap, drag slider, applies while dragging), speed_kp (stopping gain), safety_area (bbox area fraction where the car fully stops; the yellow dashed outline on the camera view is this size), angle_kp, angle_kd, score_threshold, max_detections. Save writes pursuit.yaml, Reset re-reads it.

## Known conflicts on the car

Anything else publishing /drive will fight this service at the mux and the car will sit still or stutter:

- neoracer-autonomy runs a twist bridge that idles at zero on /drive. Disable it while using pursuit: `sudo systemctl disable --now neoracer-autonomy`
- A leftover Jupyter kernel that ever created a racecar object keeps publishing /drive. Restart the jupyter service to clear them.

Check with: `ros2 topic info -v /drive` (there should be exactly one publisher: pursuit).

## Safety

The neoracer mux forwards /drive with no software deadman; the SWB switch is the physical autonomy gate. The shipped yaml has speed 0.0, so the car cannot drive until the slider is raised. Speed is hard capped at 1.0. Remember the keep-last rule: a car that loses its target mid-turn keeps turning until the switch is flipped or the box reappears.
