# ROS Indigoのインストール方法\(Ubuntu Trusty\)

**Ubuntu 16.04を使用している場合は**[**ROSのインストール方法\(Ubuntu Xenial\)**](how_to_install_ros_kinetic.md)**をご覧ください。**  
**Ubuntu 18.04を使用している場合は**[**ROSのインストール方法\(Ubuntu Bionic\)**](how_to_install_ros_melodic.md)**をご覧ください。**

## 概要

Gazebo上でRaspberry Pi Mouseをシミュレートできるようにするため、以下のソフトウェアをインストールします。

* ROS Indigo
* Gazebo 2.x \(ROSとともにインストール\)

ROSがすでにインストール済みの場合はこのページを飛ばして[Raspberry Pi Mouse Simulatorのインストール](how_to_install_simulator.md)を行ってください。

### 動作確認済み環境

* Ubuntu Desktop 14.04 LTS 64bit

## 手順

[Tiryoh/ros\_setup\_scripts\_Ubuntu14.04\_desktop](https://github.com/Tiryoh/ros_setup_scripts_Ubuntu14.04_desktop)を使用してインストール作業を進めていきます。

1. ROSをインストール
2. `catkin_ws`を準備

必要なコマンドをまとめると、以下のようになります。  
これらのコマンドについてこの後細かく説明していきます。

```bash
sudo apt-get update
sudo apt-get install -y curl
bash -c "$(curl -SsfL https://git.io/ros-indigo-desktop)"
source ~/.bashrc
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_init_workspace src
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
catkin_make && source ~/catkin_ws/devel/setup.bash
```

## コマンド解説

### ROSのインストール

今回はROSのインストールに[Tiryoh/ros\_setup\_scripts\_Ubuntu14.04\_desktop](https://github.com/Tiryoh/ros_setup_scripts_Ubuntu14.04_desktop)を使用します。

インストーラを使用するに当たり、まずは以下のコマンドでパッケージマネージャ\(apt\)のパッケージリストを最新版にし、curlをインストールします。1行ずつ実行してください。

```bash
sudo apt-get update
sudo apt-get install -y curl
```

次にインストーラを実行します。

```bash
bash -c "$(curl -SsfL https://git.io/ros-indigo-desktop)"
```

このインストーラを使ってapt-getでインストールするパッケージは以下の通りです。

* ros-indigo-desktop-full

最終的に以下のログが出ていればインストール成功です。

```text
***INSTRUCTION*****************
* do the following command    *
* $ source ~/.bashrc          *
* after that, try             *
* $ LANG=C roscore            *
*******************************
```

ROSのインストールについて詳しく調べたい場合は[ROS Wiki](http://wiki.ros.org/indigo/Installation/Ubuntu)をご覧ください。

### `catkin_ws`の準備

ROSのパッケージを保存するためのワークスペースを用意します。1行ずつ実行してください。

```bash
source ~/.bashrc
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_init_workspace src
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
catkin_make && source ~/catkin_ws/devel/setup.bash
```

## Trubleshooting

### catkin\_makeに失敗する

```bash
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

上記のようになにもビルドせず、目的のパッケージのcatkin\_makeに失敗する場合があります。

`setup.bash`を再読み込みしてからcatkin\_makeしてみてください。

詳しくは[ROSトラブルシューティング](../troubleshooting.md#catkin_make-failed)をご覧ください。

