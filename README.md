# Pure Pursuit Dashboard

Target-following as a service for the Neoracer, on port 8084. The car chases AR tags or other cars: PD steering on the target's horizontal offset, throttle that brakes as the target's bounding box grows toward a stop size.

## Install

From a laptop on the car's network:

```
scp -r neoracer_pursuit racecar@<car-ip>:~/pursuit
ssh racecar@<car-ip> 'bash ~/pursuit/setup.sh'
```

The service installs OFF: no port, power, or compute used until enabled.

```
bash ~/pursuit/setup.sh enable     # start now and at every boot
bash ~/pursuit/setup.sh disable    # stop now and stay off across boots
```

Dashboard, when enabled: `http://<car-ip>:8084`.

## Modes

`mode` in pursuit.yaml selects the target source: 0 = AR tags, 1 = cars. Switched only by editing the file, then pressing Reset or Load. There is no UI toggle.

- AR tags: ArUco DICT_6X6_250 detected straight from the camera with cv2, the same dictionary the neoracer library uses. No model involved.
- Cars: reads the driver inference node's YOLO detections (vision_msgs/Detection2DArray on /edgetpu/inference). The model decides what is detectable; use class_filter in the driver's inference.yaml to restrict what gets chased.

The largest detection above score_threshold is the target. No target means stop (with a half-second grace so a missed frame or two does not reset the throttle ramp).

## Tuning

All on the dashboard, live: speed (throttle cap, drag slider), speed_kp (stopping gain), safety_area (bbox area fraction where the car fully stops; the yellow dashed outline on the camera view is this size), angle_kp, angle_kd, score_threshold. Save writes pursuit.yaml, Reset re-reads it.

## Known conflicts on the car

Anything else publishing /drive will fight this service at the mux and the car will sit still or stutter:

- neoracer-autonomy runs a twist bridge that idles at zero on /drive. Disable it while using pursuit: `sudo systemctl disable --now neoracer-autonomy`
- A leftover Jupyter kernel that ever created a racecar object keeps publishing /drive. Restart the jupyter service to clear them.

Check with: `ros2 topic info -v /drive` (there should be exactly one publisher: pursuit).

## Safety

The neoracer mux forwards /drive with no software deadman; the transmitter's SWB switch is the physical autonomy gate. The shipped yaml has speed 0.0, so the car cannot drive until the slider is raised. Speed is hard capped at 1.0. Losing the target stops the car.
