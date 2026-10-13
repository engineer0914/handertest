# RB5-850 Eye-in-Hand

```
python main.py --robot-ip 192.168.0.100 --tcp-cam "TCP T_CAM.JSON" --cam-checkerboard CAM_T_CHECKERBOARD.JSON --json-unit m --dh-end-is-tcp
```

## Transform convention

Column vectors: p_A = T_A_B @ p_B.
T_A_B represents the pose of frame B expressed in frame A.

T_BASE_CAM = T_BASE_TCP @ T_TCP_CAM
T_TCP_CHECKERBOARD = T_TCP_CAM @ T_CAM_CHECKERBOARD
T_BASE_CHECKERBOARD = T_BASE_TCP @ T_TCP_CHECKERBOARD

DH uses all 9 supplied rows, including fixed rows 3, 5, 7.
DH d/a are in mm; joint angles and DH angular offsets are in degrees.
ZYX rotation: R = Rz(rz) @ Ry(ry) @ Rx(rx).

## Inputs

Both calibration JSON files contain raw 4x4 arrays.
--json-unit m/mm is required.
Mixed units: override --tcp-cam-unit or --cam-checkerboard-unit.
All loaded translations, plotted positions and printed matrices use mm.

TCP T_CAM.JSON contains the supplied matrix.
Demo commands assume its translation is in metres.
CAM_T_CHECKERBOARD.JSON is a DEMO: identity rotation, z=0.5 m.
Replace that file with your measured pose.

Choose --dh-end-is-tcp only if the last DH frame coincides with TCP.
Otherwise supply --dh-end-to-tcp X_MM Y_MM Z_MM RX_DEG RY_DEG RZ_DEG.
This is the TCP pose expressed relative to the last DH frame.

## Run

python -m pip install -r requirements.txt
python -m unittest -v
python main.py --joints 0 0 0 0 0 0 --json-unit m --dh-end-is-tcp

For hardware, install requirements-robot.txt and supply --robot-ip,
or set ROBOT_IP at the top of main.py.

A run reads one snapshot of measured jnt_ang via port 5001.
No robot motion commands are sent.
--timeout applies to request_data; SDK connection behavior is separate.
Use joint angles corresponding to the checkerboard image capture pose.

## Visualization

Mouse rotates/zooms the Matplotlib graph.
--show-dh-frames displays all 9 DH frames.
--save scene.png saves an image.
--no-show enables headless execution.

Default checkerboard: 10 x 7 squares, 25 mm each.
Default origin is the first internal corner.
Use --board-squares COLS ROWS --square-mm VALUE
and --board-origin inner/corner to match your estimator.

The camera frame must match the calibration/pose estimator frame.
The camera frustum is schematic, not derived from camera intrinsics.
The program visualizes supplied calibrations; it does not estimate them.

[Official SDK](https://github.com/RainbowRobotics/rbpodo)
[Official data structure](https://rainbowrobotics.github.io/rb_cobot_docs/technical_docs/data_structure)
