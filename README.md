# ROS
CIT 2017年度 ロボットシステム学 課題2 ROS

### シリアル通信のKondoサーボをROSと繋げて、動かす。
* パッケットの作成部分の製作
* ROSでサーボOnと動作のノードを作成

### 使用方法
 `$ cd ~/catkin_ws/src`   
 `$ git clone https://github.com/RikiHayashi/b3m_ros.git`  
 `$ cd ../`   
 `$ catkin_make`   
 `$ roscore &`    
 `$ rosrun b3m_leg b3m_leg &` servoのトルクが入る。    
 `$ rosrun b3m_leg b3m_send.py`　一定の周期で動作。

### 動画
* [b3m_servo_ros](https://youtu.be/RlHcSc8ZGGo)

