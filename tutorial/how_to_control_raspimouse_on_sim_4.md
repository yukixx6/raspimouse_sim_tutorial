# シミュレータ上のラズパイマウスを動かす方法 Part4

## ラズパイマウスを動かすまでの流れ

1. [距離センサの値の読み取り方](how_to_control_raspimouse_on_sim_1.md)
2. [モータを動かし方](how_to_control_raspimouse_on_sim_2.md)
3. [キーボードを用いたラズパイマウスの動かし方](how_to_control_raspimouse_on_sim_3.md)
4. [コントローラを用いたラズパイマウスの動かし方](how_to_control_raspimouse_on_sim_4.md) ← 今ここ
5. [距離センサの値を利用したラズパイマウスの動かし方](how_to_control_raspimouse_on_sim_5.md)
6. 測域センサ\(URG\)を用いたSLAMの行い方

Part4ではコントローラを用いてシミュレータ上のラズパイマウスを動かす方法を紹介します。

コントローラを用いることにより、キーボードで操作するより自由に操作することができます。

## 前準備

コントローラで動かすために、[Ryo Okazaki](https://github.com/zaki0929)さんの[`raspimouse_game_controller`](https://github.com/zaki0929/raspimouse_game_controller)を使用します。

```text
git clone https://github.com/zaki0929/raspimouse_game_controller.git
cd ~/catkin_ws
catkin_make
```

次にコントローラを扱うために必要な`joy_node`をインストールします。すでに`joy_node`をインストールしている場合、この操作を行う必要はありません。

```text
sudo apt install ros-kinetic-joy
```

以上で準備完了です。

## コントローラでラズパイマウスを動かす

今回は**Logicool Wireless Gamepad F710**を使用します。

詳しくは[`raspimouse_game_controller`](https://github.com/zaki0929/raspimouse_game_controller)の[README](https://github.com/zaki0929/raspimouse_game_controller/blob/master/README.md)をご覧下さい。

まず、前回と同様に[Raspberry Pi Mouse Simulatorの動作確認](../setup/how_to_use_raspimouse_sim.md)に従い、Gazeboを起動します。

別のターミナルで、以下のコマンドを実行します。

このコマンドがコントローラでラズパイマウスを動かすときに使用するコマンドです。実行すると自動的にモータが通電します。

```text
roslaunch raspimouse_gamepad_control run_with_base_nodes.launch
```

次に、コントローラの上のスイッチが**D**の方にセットされ、MODEのランプが**付いている**ことを確認します。

![](../.gitbook/assets/logicool_controller_mode_sw.jpg)

操作するときは**X**を押しながら、左の**十字キー**を押します。 

![](../.gitbook/assets/logicool_how_to_use.jpg)

## コントローラを使用している様子：

![](../.gitbook/assets/raspimouse_sim_demo.gif)

## Trubleshooting

### joy\_nodeについてのエラー

```text
[ERROR] [1526893382.851884948]: Couldn't open joystick /dev/input/js0. Will retry every second.
```

上記のエラーが出て、コントローラが反応しないことがあります。

コントローラを接続し直してみて下さい。

