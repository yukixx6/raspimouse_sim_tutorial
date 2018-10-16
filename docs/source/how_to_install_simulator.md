# Raspberry Pi Mouse Simulatorのインストール方法

GazeboでRaspberry Pi Mouseをシミュレーションするためのセットアップ方法

## 概要

Gazebo上でRaspberry Pi Mouseをシミュレートできるようにするため、以下のソフトウェアをインストールします。

* [rt-net/raspimouse_sim](https://github.com/rt-net/raspimouse_sim)

### 動作確認済み環境

* Ubuntu Desktop 14.04 LTS 64bit
  * ROS Indigo
  * Gazebo 2.x (ROSとともにインストール)
* Ubuntu Desktop 16.04 LTS 64bit
  * ROS Kinetic
  * Gazebo 7.x (ROSとともにインストール)
* Ubuntu Desktop 18.04 LTS 64bit
  * ROS Melodic
  * Gazebo 9.x (ROSとともにインストール)

ROSのインストールが完了していない場合は[ROSのインストール方法](how_to_install_ros_kinetic.html)をご覧ください。

## 手順

[ryuichiueda/raspimouse_sim_installer](https://github.com/ryuichiueda/raspimouse_sim_installer)を使用して[rt-net/raspimouse_sim](https://github.com/rt-net/raspimouse_sim)のインストール作業を進めていきます。

必要なコマンドをまとめると、以下のようになります。
これらのコマンドについてこの後細かく説明していきます。

```
sudo apt-get update
sudo apt-get install -y curl
bash -exv -c "$(curl -SsfL https://git.io/raspimouse-sim-installer)"
source ~/.bashrc
```

## コマンド解説

### [rt-net/raspimouse_sim](https://github.com/rt-net/raspimouse_sim)のインストール

インストーラを使用するに当たり、まずは以下のコマンドでパッケージマネージャ(apt)のパッケージリストを最新版にし、`curl`
をインストールします。1行ずつ実行してください。

```
sudo apt-get update
sudo apt-get install -y curl
```

次にインストーラを実行します。

```
bash -exv -c "$(curl -SsfL https://git.io/raspimouse-sim-installer)"
```

最終的に以下のログが出ていればインストール成功です。

```
###HOW TO VERIFY###
# roslaunch raspimouse_gazebo raspimouse_with_samplemaze.launch
# rosrun raspimouse_control controller_vel_publisher.py
```

このインストーラでは[rt-net/raspimouse_sim](https://github.com/rt-net/raspimouse_sim.git)をダウンロードし、ビルドおよび実行に必要な追加パッケージをインストールしています。

最後に設定を再読み込みしてインストール完了です。

```
source ~/.bashrc
```

## Trubleshooting

### catkin_makeに失敗する

```
$ catkin_make
Base path: /home/ubuntu/catkin_ws
Source space: /home/ubuntu/catkin_ws/src
Build space: /home/ubuntu/catkin_ws/build
Devel space: /home/ubuntu/catkin_ws/devel
Install space: /home/ubuntu/catkin_ws/install
####
#### Running command: "make cmake_check_build_system" in "/home/ubuntu/catkin_ws/build"
####
####
#### Running command: "make -j8 -l8" in "/home/ubuntu/catkin_ws/build"
####
```
上記のようになにもビルドせず、目的のパッケージのcatkin_makeに失敗する場合があります。

`setup.bash`を再読み込みしてからcatkin_makeしてみてください。

詳しくは[ROSトラブルシューティング](troubleshooting#catkin_make-failed)をご覧ください。
