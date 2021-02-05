# Sample DB（）

Sample DB用のリポジトリ（ディレクトリ）です。

# Rasberry Pi の　OS準備
適当なサイトで調べてRaspberryPiにOSをインストールしてください  
例：https://raspida.com/release-raspberry-pi-imager  
執筆者は Raspbianの  
Linux raspberrypi 4.19.115-v7l+ #1305 SMP Fri Apr 17 11:59:28 BST 2020 armv7l GNU/Linux  
をインストールしました。  

# Dockerのinstall
まずは下記にしたがってDockerをラズパイ環境にinstallしてください
```
コマンドライン

$ sudo apt update && sudo apt upgrade -y
$ sudo apt-get install apt-transport-https ca-certificates curl gnupg2 software-properties-common -y
$ curl -sSL https://get.docker.com | sh
$ docker -v
Docker version 19.03.7, build 7141c19 (Versionは特に気にしなくてよいはず。installできたことの確認)
$ sudo usermod -aG docker $(whoami)

```
ユーザがグループdockerに入ったのを反映するため、一旦ログアウトして、再度ログインします。（再起動でもよい）

# gitのinstallと該当レポジトリのclone

任意のディレクトリにgitレポジトリをcloneしてください
```
$ git clone https://github.com/arcosaka/arcosaka

```

# docker buildの実行
cloneしたリポジトリでdocer_wsに移動して、docker buildを実行します。
* sqlite3がinstallされるようにDockerFileが更新されています。
```
$ cd arcosaka/2020/docker_ws/
$ docker build -t arc_ros_sqlite .

```

# コンテナの起動
dockerコンテナを起動します。

```
$ docker run --name arc_ros_sqlite --net host --privileged -u root -v /sys:/sys -v /dev/mem:/dev/mem -v /home/pi/.ros/:/root/.ros/ -v /home/pi/arcosaka/2020/arc_ws/:/root/catkin_ws/ -it arc_ros_sqlite:latest
```
/home/pi/arcosaka/2020/arc_ws/はgitリポジトリをcloneしたディレクトリとあわせてください


# コンテナ内での作業開始 rosのbuild
コンテナの中のbashに入ったのでmake、make installを実施します。

```bash
# catkin_make
# catkin_make install
# source devel/setup.sh
```
#追加したコード  
git log で確認してください。  
scriptの下にsample_dbを追加しています。  

# サンプルの実行
1.サンプルを実行します。末尾に"&"がついているのは次のコマンドを続行して実行するためです。  
適当なところでEnterを入力するとコマンド受付状態になります。
```bash
# roslaunch arc2020 db_test.launch & 
```

2. messageをrostopicコマンドでpublishします。  
   コマンドの詳細は rostopic pub でぐぐってください。  
   簡単に説明すると、  
   pub             :publish、   
   -1              :1回だけ、  
   /arc2020_db/sql :というtopicに  
   arc2020/sql     :というmessageを  
   'seed' 'bean'   :messageの中身  
```bash
# rostopic pub -1 /arc2020_db/sql arc2020/sql   -- 'seed' 'bean' 
```

3. 送信が完了したらDBに登録されたか確認します。  
   今回TEST.dbを/root/catkin_ws/src/arc2020/に作成するようにしているので
   移動します。
```bash
# cd /root/catkin_ws/src/arc2020 
```

4. sqlite構文でTEST.dbにtable:seed,name:beanが登録されていることを確認します。
```bash
# sqlite3 TEST.db
SQLite version 3.22.0 2018-01-22 18:45:57
Enter ".help" for usage hints.
sqlite> .table                # .tableコマンドでtable一覧を表示 
seed                          # seed tableが登録されていることを確認
sqlite> SELECT * from seed;   # SELECTコマンドで seed tableの要素一覧を取得
1|bean|                       # beanが登録されていることを確認
sqlite> .exit                 # .exitiで終了 
```

5. rosnode list で 立ち上がっているnodeを確認して  
   rosnode kill で 終了する  
   * rosoutは終了しなくてよいです。
 
```bash
# rosnode list         
/arc2020_db/db
/rosout
# rosnode kill /arc2020_db/db
```

# Author

* 作成者
* 所属
* E-mail

