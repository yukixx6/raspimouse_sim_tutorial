# Raspberry Pi Mouse Simulator's Tutorial

## 概要

このサイトはRaspberry Pi Mouseのシミュレータを使いながらROSの使い方を学んでいけるオンラインチュートリアルです。

Raspberry Pi MouseやROSに親しみがない人も興味を持って取り組めるような構成にしていきます。

本チュートリアルは現在執筆中につき、まだ手順書のような構成にはなっていません。初めての方は[はじめに](introduction/)からお読みいただくとスムーズに読み進めていただけるかと思います。

このオンラインチュートリアルは千葉工業大学上田研究室にて研究の一環で開発されたものです。株式会社アールティによって提供されるものではありません。

## Raspberry Pi Mouse Simulator

Raspberry Pi Mouse SimulatorはメインボードにRaspberry Piを使った左右独立二輪方式の小型移動プラットフォームロボット、Raspberry Pi Mouseのシミュレータです。

![Raspberry Pi Mouse on sample maze](.gitbook/assets/raspimouse_samplemaze.png)

### 関連ページ

* Raspberry Pi Mouse 公式メーカーページ
  * [Raspberry Pi Mouse \| 製品情報 \| 株式会社ア－ルティ](https://www.rt-net.jp/products/raspimouse2/)
* ROS wiki
  * [raspimouse\_sim - ROS Wiki](https://wiki.ros.org/raspimouse_sim)
  * [raspimouse\_ros - ROS Wiki](https://wiki.ros.org/raspimouse_sim)
* GitHub repository
  * [rt-net/raspimouse\_sim - GitHub](https://github.com/rt-net/raspimouse_sim)

## 目次

* [はじめに](introduction/)
* [ROSとは](introduction/ros.md)

### セットアップ

#### ROSのインストール

* [ROS Indigoのインストール方法\(Ubuntu Trusty\)](setup/how_to_install_ros_indigo.md)
* [ROS Kineticのインストール方法\(Ubuntu Xenial\)](setup/how_to_install_ros_kinetic.md)
* [ROS Melodicのインストール方法\(Ubuntu Bionic\)](setup/how_to_install_ros_melodic.md)

#### シミュレータのインストール

* [Raspberry Pi Mouse Simulatorのインストール方法](setup/how_to_install_simulator.md)
* [Raspberry Pi Mouse Simulatorの動作確認](setup/how_to_use_raspimouse_sim.md)

### チュートリアル

#### ROSのチュートリアル

1. [ROSパッケージの作り方](ros_tutorial/how_to_create_pkg.md)
2. [トピックの書き方](ros_tutorial/how_to_write_topic.md)
3. [独自のメッセージファイルの作り方](ros_tutorial/how_to_create_msg.md)
4. [まとめて起動するやり方](ros_tutorial/how_to_use_launch.md)
5. [サービスを書き方](ros_tutorial/how_to_write_service.md)
6. [独自のサービスファイルの作り方](ros_tutorial/how_to_create_srv.md)
7. [付録](ros_tutorial/appendix/)
   * [ROSでよく使用する用語](ros_tutorial/appendix/ros_word.md)
   * [よく使用するROSコマンド](ros_tutorial/appendix/ros_comand.md)

#### シミュレータのチュートリアル

1. [シミュレータ上のラズパイマウスを動かす方法 Part1](tutorial/how_to_control_raspimouse_on_sim_1.md)
2. [シミュレータ上のラズパイマウスを動かす方法 Part2](tutorial/how_to_control_raspimouse_on_sim_2.md)
3. [シミュレータ上のラズパイマウスを動かす方法 Part3](tutorial/how_to_control_raspimouse_on_sim_3.md)
4. [シミュレータ上のラズパイマウスを動かす方法 Part4](tutorial/how_to_control_raspimouse_on_sim_4.md)
5. [シミュレータ上のラズパイマウスを動かす方法 Part5](tutorial/how_to_control_raspimouse_on_sim_5.md)
6. [シミュレータ上のラズパイマウスを動かす方法 Part6](tutorial/how_to_control_raspimouse_on_sim_6.md)

### トラブルシューティング

* [ROSのトラブルシューティング](troubleshooting.md)

## 終わりに <a id="afterword"></a>

本チュートリアルは千葉工業大学上田研究室にて研究の一環で開発されました。  
ぜひアンケートにご協力よろしくお願いします。 

[利用者アンケート](https://docs.google.com/forms/d/1MS4NzNuZ98quCpD-EPRLXg-ltk9QZ5SK46gBR9cLhfg/edit)\(Google Forms\)

