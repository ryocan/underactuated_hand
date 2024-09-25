## 自分用のメモ

Realsense D405での深度画像の取得
```
roslaunch realsense2_camera rs_aligned_depth.launch
```

Dobotの起動
```
roslaunch mg400_bringup mg400_bringup.launch robot_ip:=192.168.1.6

```

変形検出や制御
```
roslaunch img_proc fb_control.launch

```
