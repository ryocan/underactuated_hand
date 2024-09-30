## ファイル概要説明
### src
メインで動くのは*main.cpp*である．  
制御に関するプログラムがまとめてあるのが*fb_control.cpp*，画像処理の一般的な処理についてまとめてあるのが*common_proc.cpp*．  
fb_control.cppでは提案手法の全フローを網羅しているため，関数を全部書いてしまうと見づらくなると考えた．
そこでfb_control.cppでは主にフローのみ記述することとし，関数の詳細はファイルを分けた．  
下記がfb_control.cppで参照しているファイルの詳細である．  

| 処理内容  | cppファイル |
| ------------- | ------------- |
| 背景差分による把持物体検出  | object_detection.cpp  |
| 物体とハンドの接触検出  | contact_detection.cpp  |
| 物体を把持するときの状態検出(潰れ検出など)  | object_state_detection.cpp  |
| 物体の落下検出  | object_detection.cpp  |
| DynamixelやDobotとの通信のための関数  | service_client.cpp  |
| Dobot制御のための関数  | dobot_control_sub.cpp  |

DOBOTの制御については，サービス通信で行われている．  
サービス通信は，responsがあって初めて次の処理が実行される．  
自分のコードの書き方が良くなかったというのがあるかもだが，アームを動かしている最中に画像処理できなかったため，fb_control.cppからトピックでアームの制御位置司令を送ったあと，
dobot_control_sub.cppでトピック受信およびアームとのサービス通信，またサービス通信完了のタイミングをfb_control.cppにpublishするようにした．  
あんまり参考にしないでほしい.  

### include
*fb_control.hpp*はfb_control.cppの，*common_proc*はcommon_proc.cpp用のファイル

### config
*param.yaml*では，閾値の調整が可能である．毎回buildしなくても，yamlファイルの数字を書き換えるだけで値が更新される