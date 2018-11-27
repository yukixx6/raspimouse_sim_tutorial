# よく使用されるROSコマンド
ここではよく使用するROSコマンドについて紹介しようと思います。

コマンドを入力する時、Tab補完という機能を利用すると簡単に入力できます。
Tabキーを押した時、入力していた文字を補完してくれる機能です。
他に同じ文字の使用している対象がいなければすべて補完し、同じ文字を使用している対象がいれば重複している文字までを補完してくれます。

## 一覧

* 実行コマンド
	* roscore
	* rosrun 
	* roslaunch

* 環境コマンド
	* rosdep

* 通信情報コマンド
	* rosnode
	* rostopic
	* rosservice
	* rosparam
	* rosmsg
	* rosbag
	
* その他コマンド
	* roscd
	* rosed


## 実行コマンド

### roscore : ROSを使用するときに使用 

```
roscore 
```
<br>

### rosrun : プログラムを実行するときに使用 

```
rosrun <パッケージ名> <プログラム名>
rosrun ros_tutorial time_pub.py
```
**注 : roscoreを起動しておく必要がある**

<br>

### roslaunch : `roscore`と複数の`rosrun`を同時に実行

```
roslaunch <パッケージ名> <launchファイル名>
roslaunch sample_program sample.launch
```
<br>

## 環境コマンド

### rosdep : パッケージの依存しているファイルに関する(操作する)コマンド

rosdepを使用する時、一番最初に実行するコマンドで、
rosdepを初期化し、最新のもの更新する。

```
rosdep init
rosdep update
```

パッケージの依存しているファイルを確認するコマンド

```
rosdep check <パッケージ名>
rosdep check raspimouse_sim
```

不足しているファイルをインストールするコマンド

```
rosdep install <ファイル名>
```
<br>

## 通信情報コマンド

### rosnode : ノードに関するコマンド

立ち上がっているノードを確認する。

```
rosnode list
```
<br>

### rostopic : トピックに関するコマンド

動いているトピック名一覧を表示する。

```
rostopic list
```

トピックから配信されている値を確認する。

```
rostopic echo /<トピック名>
rostopic echo /lightsensors
```
<br>

### rosservice : サービスに関するコマンド

動いているサービス一覧を表示する。

```
rosservice list
```

サービスを起動する。

```
rosservice call /<サービス名>
rosservice call /motor_on
```
<br>

### rosparam : パラメータに関するコマンド

パラメータを設定できる変数の一覧を表示する。

```
rosparam list
```

変数にパラメータを設定する。

```
rosparam set /<変数名>
rosparam set /lightsensors/frequency
```
<br>

### rosmsg : メッセージに関するコマンド

動いているメッセージの一覧を表示する。

```
rosmsg list
```

メッセージの型を確認する。

```
rosmsg show <メッセージ名>
rosmsg show std_msgs/int16
```
<br>

### rosbag : 現在の状況を記録したり、再現したりするコマンド

動いているトピックを記録する。記録したものをバグファイルと呼ぶ。

記録するトピックは一つ一つ指定できるが、引数`-a`を取ればすべてのトピックを記録できる。

バグファイル名を指定しない場合は、その時の日付と時刻がファイル名になる。

```
rosbag record <トピック名> <トピック名> ... <トピック名>
rosbag record -a
```

バグファイル名を指定する場合は引数に`-O`を取る。バグファイルの拡張子は`.bag`になる。

```
rosbag record -O <バグファイル名> <トピック名> ... <トピック名>
rosbag record -O sample.bag -a
```

記録したバグファイルを再生する。

```
robag play <バグファイル名>
```

**注 : roscoreを起動しておく必要がある**


<br>

## その他のコマンド

### roscd : 移動コマンド

指定したディレクトリまで移動する。

```
roscd <パッケージ名>
roscd ros_torial
```
<br>

### rosed : ファイル編集コマンド

指定したファイルを編集する。

```
rosed <パッケージ名>　<ファイル名>
rosed ros_tutorial time_pub.py
```

<br>

