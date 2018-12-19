# シミュレータ上のラズパイマウスを動かす方法 Part5

## ラズパイマウスを動かすまでの流れ

1. [距離センサの値の読み取り方](how_to_control_raspimouse_on_sim_1.md)
2. [モータの動かし方](how_to_control_raspimouse_on_sim_2.md)
3. [キーボードを用いたラズパイマウスの動かし方](how_to_control_raspimouse_on_sim_3.md)
4. [コントローラを用いたラズパイマウスの動かし方](how_to_control_raspimouse_on_sim_4.md)
5. [距離センサの値を利用したラズパイマウスの動かし方](how_to_control_raspimouse_on_sim_5.md)←今ここ
6. [測域センサ\(URG\)を用いたSLAMの行い方](how_to_control_raspimouse_on_sim_6.md)

## はじめに

シミュレータのチュートリアルPart5,Part6では実際にプログラムでラズパイマウスを動かしていみたいと思います。そのため少し難しいかもしれません。

プログラムは以下のリポジトリで公開しています。

GitHub:[raspimouse_sim_tutorial_program](https://github.com/yukixx6/raspimouse_sim_tutorial_program)

フォークかクローンしてみましょう。
フォークは上記のリポジトリに飛び、ページの右上にある`Fork`のボタンを押せば出来ます。Forkすることで本家から派生して自分のリポジトリを作成することが出来ます。また自分のリポジトリとなっているので、加えた変更をコミットしておくことも出来ます。

実際にクローンしましょう。
まず`catkin_ws/src`に移動します。

```
cd ~/catkin_ws/src
```

フォークした場合、クローンするときは以下のコマンドになります。

```
git clone https://github.com/自分のGitHubのユーザー名/raspimouse_sim_tutorial_program.git
```

フォークをしていない場合、クローンする時は以下のコマンドになります。

```
git clone https://github.com/yukixx6/raspimouse_sim_tutorial_program.git
```
<br>

このPart5では左手法という迷路の解析手法を利用して、ラズパイマウスをスタート地点からゴールまで動かしてみようと思います。

## 左手法について

左手法とは、優先的に左側を向き進み、常に左側の壁に沿って動くことでゴールを目指す手法のことです。 マイクロマウス競技などでも使用されています。

| 次に取る行動 | 条件1 | 条件2 | 条件3 | 条件4 |
| :--- | :--- | :--- | :--- | :--- |
| 左を向く | 左、右、正面が空いている | 左、右が空いている | 左、正面が空いている | 左のみが空いている |
| 前進する | 正面、右が空いている | 正面のみが空いている | なし | なし |
| 右を向く | 右のみが空いている | なし | なし | なし |
| Uターンする | すべて塞がっている | なし | なし | なし |

## サンプルプログラム

今回もPythonで書きます。 名前は`left_hand.py`とします。

```text
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from raspimouse_ros_2.msg import *
from std_srvs.srv import Trigger, TriggerResponse, Empty

class LeftHand():
    def __init__(self):
        # 光センサのサブスクライバー
        rospy.Subscriber('/lightsensors', LightSensorValues, self.sensor_callback)
        # モータに周波数を入力するためのパブリッシャー
        self.motor_raw_pub = rospy.Publisher('/motor_raw', MotorFreqs, queue_size = 10)
        # Raspberry Pi Mouseの光センサのメッセージオブジェクト
        self.data = LightSensorValues()

        # 実行時にシミュレータを初期状態にする
        self.modeSimReset = True

        self.ls_count = 0
        self.rs_count = 0

    def sensor_callback(self, msg):
        # クラス変数のメッセージオブジェクトに受信したデータをセット
        self.data = msg

    def motor_cont(self, left_hz, right_hz):
        if not rospy.is_shutdown():
            d = MotorFreqs()
            # 両輪の周波数を設定
            d.left_hz = left_hz
            d.right_hz = right_hz
            # パブリッシュ
            self.motor_raw_pub.publish(d)

    def turn_move(self, m):
        if m == "LEFT": self.motor_cont(-200, 200)
        if m == "RIGHT": self.motor_cont(200, -200)

    def moveFeedback(self, offset, speed, k, mode):
        # left_sideが1500より大きい時は、右回り旋回
        if self.data.left_side > 1500:
            self.turn_move("RIGHT")
            return
        
        # right_sideが1500より大きい時は、右回り旋回
        if self.data.right_side > 1500:
            self.turn_move("LEFT")
            return

        # 壁沿いを追従走行するための計算
        # (基準値 - 現在のleft_side) * ゲイン
        if mode == "LEFT":
            diff = (offset - self.data.left_side) * k
            # 計算した値をモータに出力
            self.motor_cont(speed - diff, speed + diff)
        if mode == "RIGHT":
            diff = (offset - self.data.right_side) * k
            # 計算した値をモータに出力
            self.motor_cont(speed + diff, speed - diff)

    def stopMove(self):
        # 終了時にモータを止める
        self.motor_cont(0, 0)

    def checker(self):
        # 壁無し判定
        if self.data.left_side < 100:
            print("--RS_COUNT:", self.data.left_side)
            self.rs_count += 1
        if self.data.right_side < 150:
            print("--LS_COUNT:", self.data.right_side)
            self.ls_count += 1

    def motion(self):
        # 左側に壁がある確率が高くて、目の前に壁がなさそうなとき
        if self.data.left_forward < 300 or self.data.right_forward < 300:
            print("Move: STRAIGHT")
            for time in range(12):
                self.checker()
                if self.data.left_side > self.data.right_side:
                    self.moveFeedback(500, 500, 0.2, "LEFT")
                else:
                    self.moveFeedback(500, 500, 0.2, "RIGHT")
                self.rate.sleep()
            self.stopMove()
            
            # 目の前に壁がなくて、右側に壁がない場合
            if self.data.left_forward < 300 or self.data.right_forward < 300:
                if self.rs_count > 0:
                    print("Move: MID LEFT TURN")
                    for time in range(10):
                        self.turn_move("LEFT")
                        self.rate.sleep()
                    self.stopMove()
            # 直進した後に、目の前に壁があったとき
            elif self.data.left_forward > 300 and self.data.right_forward > 300:
                # 左右の壁がない場合
                if self.ls_count > 0 and self.rs_count > 0:
                    print("Move: LEFT TURN_2")
                    for time in range(10):
                        self.turn_move("LEFT")
                        self.rate.sleep()
                    self.stopMove()
                # 右の壁がない場合
                elif self.ls_count > 0:
                    print("Move: RIGHT TURN")
                    for time in range(10):
                        self.turn_move("RIGHT")
                        self.rate.sleep()
                    self.stopMove()
                # 左の壁がない場合
                elif self.rs_count > 0:
                    print("Move: LEFT TURN")
                    for time in range(10):
                        self.turn_move("LEFT")
                        self.rate.sleep()
                    self.stopMove()          
            self.ls_count = 0
            self.rs_count = 0
            return
        
        # 左右関係なく、目の前に壁があるとき
        if self.data.left_forward > 2000 and self.data.right_forward > 2000:
            print("Move: DEAD END")
            for time in range(20):
                self.turn_move("LEFT")
                self.rate.sleep()
            self.stopMove()
            self.ls_count = 0
            self.rs_count = 0
            return
        if self.data.left_side > self.data.right_side:
            self.moveFeedback(500, 500, 0.2, "LEFT")
        else:
            self.moveFeedback(500, 500, 0.2, "RIGHT")

    def init(self):
        if self.modeSimReset:
            rospy.wait_for_service('/gazebo/reset_world')
            try: rospy.ServiceProxy('/gazebo/reset_world', Empty).call()
            except rospy.ServiceException, e: print "Service call failed: %s"%e
        rospy.wait_for_service('/motor_on')
        try: rospy.ServiceProxy('/motor_on', Trigger).call()
        except rospy.ServiceException, e: print "Service call failed: %s"%e
        
    def run(self):
        self.rate = rospy.Rate(10)
        self.init()
        rospy.on_shutdown(self.stopMove)
        while self.data.left_side == 0 and self.data.right_side == 0:
            self.rate.sleep()
        while not rospy.is_shutdown():
            self.motion()
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('LeftHand')
    LeftHand().run()
```

`# -*- coding: utf-8 -*-`はプログラム内に日本語のコメント書くときに必要で、今回はコードが長くなったのでコメントはプログラム内に書きました。

## 実行結果

![](../.gitbook/assets/left_hand.gif)

今回は判別の閾値を決め打ちで設定しています。
そのため正常に動いているように見えますが、時折変な挙動を示したり壁にぶつかったりしてしまっています。

動作がより良くなるように判別の閾値を変更してみましょう。
判別の閾値は主に`motion`関数のif文の値になります。