(sam6d3) recl3090@recl3090-Precision-3680:~/Downloads/handertest$ python main.py --robot-ip 192.168.0.100 --tcp-cam "TCP T_CAM.JSON" --cam-checkerboard CAM_T_CHECKERBOARD.JSON --json-unit m --dh-end-is-tcp
Joint source: measured jnt_ang at 192.168.0.100:5001
q [deg] = [ -1.8576684  -36.13392258 127.66570282  -9.26593399  56.11946869 -70.08493805]; DH transforms = 9
Convention: p_A = T_A_B @ p_B
All translations below are in mm.

T_DH_END_TCP
[[1. 0. 0. 0.]
 [0. 1. 0. 0.]
 [0. 0. 1. 0.]
 [0. 0. 0. 1.]]

T_BASE_TCP
[[   0.96586418   -0.0935961     0.24154949  256.2715039 ]
 [   0.25161096    0.56079186   -0.78879935 -173.00509036]
 [  -0.06163044    0.82264954    0.56519857  437.31611574]
 [   0.            0.            0.            1.        ]]

T_BASE_CAM
[[  -0.95939375   -0.26293786    0.10211416  301.87182534]
 [  -0.27444129    0.78650569   -0.55325472 -230.63311859]
 [   0.06515825   -0.55881346   -0.82672965  443.86585673]
 [   0.            0.            0.            1.        ]]

T_TCP_CHECKERBOARD (checkerboard pose in TCP)
[[  -0.95793601    0.28166213    0.05500033  133.80480108]
 [   0.11483835    0.55185862   -0.82599286 -569.27470329]
 [  -0.26300332   -0.78493217   -0.56099086  118.00923104]
 [   0.            0.            0.            1.        ]]

T_CHECKERBOARD_TCP (TCP points -> checkerboard coordinates)
[[  -0.95793601    0.11483835   -0.26300332  224.58782295]
 [   0.28166213    0.55185862   -0.78493217  369.10064923]
 [   0.05500033   -0.82599286   -0.56099086 -411.37404744]
 [   0.            0.            0.            1.        ]]

T_BASE_CHECKERBOARD
[[  -0.99951282    0.03079558   -0.00507449  467.29573273]
 [   0.03083005    0.99950109   -0.00686216 -551.66855832]
 [   0.00486063   -0.00701526   -0.99996357   27.45474395]
 [   0.            0.            0.            1.        ]]
(sam6d3) recl3090@recl3090-Precision-3680:~/Downloads/handertest$ python3 get_realworld_checkerboard_matrix.py 
========== Camera Intrinsic ==========
Size  : 640 x 480
fx/fy : 606.8292236328125, 606.9563598632812
cx/cy : 327.4695129394531, 240.5584716796875
Model : distortion.inverse_brown_conrady
Dist  : [0.0, 0.0, 0.0, 0.0, 0.0]
======================================
Checkerboard: (7, 5)
Square size: 0.03 m
Camera axes: X right, Y down, Z forward
p_cam = T_CAM_CHECKERBOARD @ p_checkerboard
[SPACE] print current matrix
[Q / ESC] quit
Save JSON on SPACE: False
Click the video window before pressing keys.

T_CAM_CHECKERBOARD [translation: m] | frame=251
[
  [0.73671703, 0.67487193, 0.0423781, -0.14024524],
  [-0.63955597, 0.67507158, 0.36775879, 0.02948104],
  [0.21958183, -0.29803733, 0.9289551, 0.53110947],
  [0.0, 0.0, 0.0, 1.0]
]
(sam6d3) recl3090@recl3090-Precision-3680:~/Downloads/handertest$ ^C
(sam6d3) recl3090@recl3090-Precision-3680:~/Downloads/handertest$ python main.py --robot-ip 192.168.0.100 --tcp-cam "TCP T_CAM.JSON" --cam-checkerboard CAM_T_CHECKERBOARD.JSON --json-unit m --dh-end-is-tcp
Joint source: measured jnt_ang at 192.168.0.100:5001
q [deg] = [  -7.89188194  -16.53496742   91.66187286   26.02846146   72.08641052 -105.67237854]; DH transforms = 9
Convention: p_A = T_A_B @ p_B
All translations below are in mm.

T_DH_END_TCP
[[1. 0. 0. 0.]
 [0. 1. 0. 0.]
 [0. 0. 1. 0.]
 [0. 0. 0. 1.]]

T_BASE_TCP
[[   0.9163143     0.22457979   -0.33155999  326.13219419]
 [  -0.38651502    0.27939271   -0.87894588 -186.99355974]
 [  -0.10475804    0.93354359    0.342815    565.55199042]
 [   0.            0.            0.            1.        ]]

T_BASE_CAM
[[  -0.92556038    0.31299802   -0.21300285  325.8763698 ]
 [   0.36451354    0.88875199   -0.27793843 -259.86228761]
 [   0.10231254   -0.33489122   -0.93668565  554.00378219]
 [   0.            0.            0.            1.        ]]

T_TCP_CHECKERBOARD (checkerboard pose in TCP)
[[  -0.72039394   -0.69237096   -0.04068205  174.2178005 ]
 [  -0.2311232     0.29495405   -0.92713762 -560.61845229]
 [   0.6539225    -0.65850176   -0.3725061    24.38592679]
 [   0.            0.            0.            1.        ]]

T_CHECKERBOARD_TCP (TCP points -> checkerboard coordinates)
[[  -0.72039394   -0.2311232     0.6539225   -20.01298842]
 [  -0.69237096    0.29495405   -0.65850176  302.03820511]
 [  -0.04068205   -0.92713762   -0.3725061  -503.59901628]
 [   0.            0.            0.            1.        ]]

T_BASE_CHECKERBOARD
[[  -0.9288274    -0.34985585   -0.1219858   351.78148263]
 [  -0.36089354    0.9288072     0.08410143 -432.39797436]
 [   0.08387792    0.12213961   -0.98896229   32.29937507]
 [   0.            0.            0.            1.        ]]

