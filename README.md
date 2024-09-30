## 概要
劣駆動ハンドで物体把持を行う手法のプログラム．
制御プログラムは**img_proc**にある．

## システム起動時コマンド

Realsense D405での深度画像の取得
```
roslaunch realsense2_camera rs_aligned_depth.launch
```

Dobotの起動(PCとLANケーブル接続)
```
roslaunch mg400_bringup mg400_bringup.launch robot_ip:=192.168.1.6

```

画像処理
```
roslaunch img_proc fb_control.launch

```
