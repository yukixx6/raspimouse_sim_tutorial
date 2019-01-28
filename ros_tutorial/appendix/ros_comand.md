# よく使用するROSコマンド

ここではよく使用するROSコマンドについて紹介しようと思います。

コマンドを入力する時、Tab補完という機能を利用すると簡単に入力できます。 Tabキーを押した時、入力していた文字を補完してくれる機能です。 他に同じ文字の使用している対象がいなければすべて補完し、同じ文字を使用している対象がいれば重複している文字までを補完してくれます。

## 一覧

* [実行コマンド](ros_comand.md#zikkou)
  * [roscore](ros_comand.md#roscore)
  * [rosrun](ros_comand.md#rosrun)
  * [roslaunch](ros_comand.md#roslaunch)
* [環境コマンド](ros_comand.md#kankyou)
  * [rosdep](ros_comand.md#rosdep)
* [通信情報コマンド](ros_comand.md#tusin)
  * [rosnode](ros_comand.md#rosnode)
  * [rostopic](ros_comand.md#rostopic)
  * [rossrv](ros_comand.md#rossrv)
  * [rosservice](ros_comand.md#rosservice)
  * [rosparam](ros_comand.md#rosparam)
  * [rosmsg](ros_comand.md#rosmsg)
  * [rosbag](ros_comand.md#rosbag)
* [その他コマンド](ros_comand.md#sonota)
  * [roscd](ros_comand.md#roscd)
  * [rosed](ros_comand.md#rosed)

## 実行コマンド  <a id="zikkou"></a>

### roscore: ROSで通信するときに使用  <a id="roscore"></a>

roscoreはROS Master、ROS Parameter Server、rosoutログ用のnodeを立ち上げている。

```text
roscore
```

### rosrun: プログラムを実行するときに使用  <a id="rosrun"></a>

```text
rosrun <パッケージ名> <プログラム名>
rosrun ros_tutorial time_pub.py
```

**注 : roscoreを起動しておく必要がある**

### roslaunch: `roscore`と複数の`rosrun`を同時に実行  <a id="roslaunch"></a>

```text
roslaunch <パッケージ名> <launchファイル名>
roslaunch sample_program sample.launch
```

## 環境コマンド  <a id="kankyou"></a>

### rosdep: パッケージの依存しているファイルに関する\(操作する\)コマンド  <a id="rosdep"></a>

rosdepを使用する時、一番最初に実行するコマンドで、 rosdepを初期化し、最新のもの更新する。

```text
rosdep init
rosdep update
```

パッケージの依存しているファイルを確認するコマンド

```text
rosdep check <パッケージ名>
rosdep check raspimouse_sim
```

不足しているファイルをインストールするコマンド

```text
rosdep install <ファイル名>
```

## 通信情報コマンド  <a id="tusin"></a>

### rosnode: ノードに関するコマンド  <a id="rosnode"></a>

立ち上がっているノードを確認する。

```text
rosnode list
```

### rostopic: トピックに関するコマンド  <a id="rostopic"></a>

動いているトピック名一覧を表示する。

```text
rostopic list
```

トピックから配信されている値を確認する。

```text
rostopic echo /<トピック名>
rostopic echo /lightsensors
```

トピックにデータを送る。

```text
rostopic pub /<トピック名>　<メッセージの名前/メッセージの型> "<型の名前>: <データ>"
rostopic pub /UnixTime std_msgs/Float64 "data: 3.0"
```

### rossrv: サービスに関するコマンド1  <a id="rossrv"></a>

使用できるサービス全てを表示する。

```text
rossrv list
```

サービスの型を確認する。

```text
rossrv show ros_tutorial/DateTrigger
```

### rosservice: サービスに関するコマンド2  <a id="rosservice"></a>

動いているサービスの一覧を表示する。

```text
rosservice list
```

サービスを起動する。

```text
rosservice call /<サービス名>
rosservice call /motor_on
```

### rosparam : パラメータに関するコマンド  <a id="rosparam"></a>

パラメータを設定できる変数の一覧を表示する。

```text
rosparam list
```

変数にパラメータを設定する。

```text
rosparam set /<変数名>
rosparam set /lightsensors/frequency
```

### rosmsg: メッセージに関するコマンド  <a id="rosmsg"></a>

メッセージの一覧を表示する。

```text
rosmsg list
```

メッセージの型を確認する。

```text
rosmsg show <メッセージ名>
rosmsg show std_msgs/int16
```

### rosbag: 現在の状況を記録したり、再現したりするコマンド  <a id="rosbag"></a>

動いているトピックを記録する。記録したものをバグファイルと呼ぶ。

記録するトピックは一つ一つ指定できるが、引数`-a`を取ればすべてのトピックを記録できる。

バグファイル名を指定しない場合は、その時の日付と時刻がファイル名になる。

```text
rosbag record <トピック名> <トピック名> ... <トピック名>
rosbag record -a
```

バグファイル名を指定する場合は引数に`-O`を取る。バグファイルの拡張子は`.bag`になる。

```text
rosbag record -O <バグファイル名> <トピック名> ... <トピック名>
rosbag record -O sample.bag -a
```

記録したバグファイルを再生する。

```text
rosbag play <バグファイル名>
```

**注 : roscoreを起動しておく必要がある**

## その他のコマンド  <a id="sonota"></a>

### roscd: 移動コマンド  <a id="roscd"></a>

指定したディレクトリまで移動する。

```text
roscd <パッケージ名>
roscd ros_torial
```

### rosed: ファイル編集コマンド  <a id="rosed"></a>

指定したファイルを編集する。

```text
rosed <パッケージ名>　<ファイル名>
rosed ros_tutorial time_pub.py
```

