# サービスを書き方

## ROSチュートリアルの流れ

1. [ROSパッケージの作り方](how_to_create_pkg.md)
2. [トピックの書き方](how_to_write_topic.md)
3. [独自のメッセージファイルの作り方](how_to_create_msg.md)
4. [まとめて起動するやり方](how_to_use_launch.md)
5. [サービスを書き方](how_to_write_service.md) ←今ここ
6. [独自のサービスファイルの作り方](how_to_create_srv.md)

## はじめに

この章では、サービスの書き方について説明したいと思います。

サービスにはデータを要求する「クライアント」と応答する「サーバ」があります。

これはトピックの「サブスクライバ」と「パブリッシャ」の関係に似ていますが、サービスは更に応答の**成否**を知ることができます。

またトピックではメッセージ\(.msg\)を使用しましたが、サービスではサービスのファイル\(`.srv`\)を使用します。

詳しくは[ROSでよく使用する用語](appendix/ros_word.md#service)を御覧ください。

## サービスファイル

サービスでは`.srv`という拡張子になっているファイルを使用します。 まず標準のサービス`std_srvs`の`SetBool.srv`を見てみましょう。

```text
rossrv show std_srvs/SetBool
```

以下のように表示されるはずです。

```text
bool data
---
bool success
string message
```

`---`の上が入力、下が出力になります。

## サービスサーバ

最初にサーバ側から書いていきます。

`server.py`を作成します。

```text
roscd ros_tutorial/
vim scripts/server.py
```

```text
#!/usr/bin/env python                                                           
import rospy
from std_srvs.srv import SetBool, SetBoolResponse

def callback_srv(data):
    resp = SetBoolResponse()
    resp.success = data.data
    if data.data == True:
        resp.message = "called"
    else:
        resp.message = "ready"
    print(resp.message)
    return resp

if __name__ == "__main__":
    rospy.init_node("srv_server")
    srv = rospy.Service('service_call', SetBool, callback_srv)
    rospy.spin()
```

実行権限を与えます。

```text
chmod +x scripts/server.py
```

### コード解説

```text
#!/usr/bin/env python                                                           

import rospy
```

ここまで今までと同じです。

```text
from std_srvs.srv import SetBool, SetBoolResponse
```

標準のサービスの`std_srvs`の中にある`SetBool`とその出力に関する`SetBoolResponse`をインポートしています。

先にメイン関数について説明します。

```text
if __name__ == "__main__":
    rospy.init_node("srv_server")
```

`srv_server`というノードの名前にしています。

```text
    srv = rospy.Service('service_call', SetBool, callback_srv)
```

ここでサービスをインスタンスしています。`service_call`がサービス名、`SetBool`がサービスの型、`callback_srv`がサービスの引数を返すコールバック関数名になります。

```text
    rospy.spin()
```

プログラムを終了させないようにしています。

最後にコールバック関数について解説します。

```text
def callback_srv(data):
```

`callback_srv`という関数名で、受け取った値を`data`という名前にしてます。

```text
resp = SetBoolResponse()
```

`SetBoolResponse`を`resp`という名前でインスタンス化しています。

```text
    resp.success = data.data
```

サービスファイルの出力の`success`に入力の`data`を入れています。 両方共同じbool型のため、そのまま与えています。

```text
    if data.data == True:
        resp.message = "called"
    else:
        resp.message = "ready"
```

Trueが入力された場合called、Falseが入力された場合readyを`message`に書き込んでいます。

```text
    print(resp.message)
    return resp
```

確認のため、`message`を表示しています。

最後にサービスの型を返す必要があり、`SetBoolResponse`をインスタンス化した`resp`を返しています。

## サービスクライアント

次にクライアント側を書いていきます。

`client.py`を作成します。

```text
roscd ros_tutorial/
vim scripts/client.py
```

```text
#!/usr/bin/env python
import rospy
from std_srvs.srv import SetBool

if __name__ == "__main__":
    rospy.wait_for_service('service_call')
    try:
        service_call = rospy.ServiceProxy('service_call', SetBool)
        service_call(True)
    except rospy.ServiceException, e:
        print ("Service call failed: %s" % e)
```

実行権限を与えます。

```text
chmod +x scripts/client.py
```

### コード解説

```text
#!/usr/bin/env python                                                           
import rospy
```

ここまで同じです。

```text
std_srvs.srv import SetBool
```

`SetBool`型をインポートしています。

```text
    rospy.wait_for_service('service_call')
```

ここで`service_call`というサービスが立ち上がるのを待ちます。

トピックを違い、サービスは一対一通信のため相手がいないとエラーを吐きます。 そのためサービスの立ち上がりを待っています。

```text
    try:
        service_client = rospy.ServiceProxy('service_call', SetBool)
        service_client(True)
    except rospy.ServiceException, e:
        print ("Service call failed: %s" % e)
```

`sarvice_call`という名前の`SetBool`型のサービスプロキシ`service_client`を作成しています。

`service_client`にTrueを与えています。

## 実行方法

それぞれ別のターミナルで実行してください。

```text
roscore
```

```text
rosrun ros_tutorial server.py
```

```text
rosrun ros_tutorial client.py
```

## 実行結果

```text
rosrun ros_tutorial client.py
```

を実行した時、`rosrun ros_tutorial server.py`を実行しているターミナルで、

```text
called
```

と表示されたら、正常に動作しています。

## 別の方法

```text
roscore
```

```text
rosrun ros_tutorial server.py
```

ここまで同じですが、こちらでは`rosservice`というコマンドを使用します。 このコマンドでもサーバに要求できます。

```text
rosservice call /service_call "data: true"
```

```text
rosservice call /service_call "data: false"
```

