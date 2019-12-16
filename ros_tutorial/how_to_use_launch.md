# まとめて起動するやり方

## ROSチュートリアルの流れ

1. [ROSパッケージの作り方](how_to_create_pkg.md)
2. [トピックの書き方](how_to_write_topic.md)
3. [独自のメッセージファイルの作り方](how_to_create_msg.md)
4. [まとめて起動するやり方](how_to_use_launch.md) ←今ここ
5. [サービスの書き方](how_to_write_service.md)
6. [独自のサービスファイルの作り方](how_to_create_srv.md)

## はじめに

[独自のメッセージファイルの作り方](how_to_create_msg.md)では、作成したプログラムを起動するときに`roscore`を立ち上げて、`rosrun ros_tutorial time_pub2.py` 、`rosrun ros_tutorial timesub2.py`とそれぞれ起動していました。

しかし毎回これらのプログラムを立ち上げるのは手間です。 そこでROSにはまとめて起動することができる`roslaunch`というコマンドがあります。

この章では、`roslaunch`コマンドを使用して、まとめて複数のプログラムを起動するやり方について説明します。

詳しい説明や、他のROSコマンドについては[よく使用するROSコマンド](appendix/ros_comand.md)を御覧ください。

## launchファイルを書く

まず`roslaunch`コマンドで使用するlaunchファイルを書きましょう。 このlaunchファイルで起動するプログラムを指定することが出来ます。今回は`time_pub2.py`と`time_sub2.pyを`指定します。

launchファイルは`launch`ディレクトリの中に置きます。ない場合は作成しましょう。

```text
roscd ros_tutorial
mkdir launch
```

ファイル名は`date.launch`とします。

```text
vim launch/date.launch
```

{% code title="date.launch" %}
```text
<launch>
    <node pkg="ros_tutorial" name="time_pub" type="time_pub2.py" />
    <node pkg="ros_tutorial" name="time_sub" type="time_sub2.py" />
</launch>
```
{% endcode %}

### コード解説

```text
<launch>
    ︙
</launch>
```

起動するnodeはこの間に書きます。

```text
    <node pkg="ros_tutorial" name="time_pub" type="time_pub2.py" />
    <node pkg="ros_tutorial" name="time_sub" type="time_sub2.py" />
```

`pkg`は起動するパッケージ名、`name`はノード名、`type`はプログラム名を書きます。

## 実行

`roslaunch`コマンドで実行しましょう。

```text
roslaunch ros_tutorial date.launch
```

何もエラーを吐かなければ正常に立ち上がっています。

### 実行結果

`roslaunch`コマンドで立ち上げた場合、プログラム内の`print`は表示されません。 そのため`rostopic`コマンドでトピックが動いているか確認しましょう。

```text
rostopic list
```

```text
/Date_and_Time
/rosout
/rosout_agg
```

```text
rostopic echo /Date_and_Time
```

```text
date: 20181120
time: 172953.60466
---
date: 20181120
time: 172953.7045
---

        ︙
```

またプログラム内の`print`で表示できる`--screen`という引数があります。

```text
roslaunch ros_tutorial date.launch --screen
```

```text
date : 20181120 , time : 173715.503780
date : 20181120 , time : 173715.603760
date : 20181120 , time : 173715.701480
date : 20181120 , time : 173715.803500
date : 20181120 , time : 173715.902740
date : 20181120 , time : 173716.001930
date : 20181120 , time : 173716.101760
date : 20181120 , time : 173716.204750

                ︙
```

