# ARC2020（）

2020年用のリポジトリ（ディレクトリ）です。

# Rasberry Pi の　OS準備
適当なサイトで調べてRaspberryPiにOSをインストールしてください  
例：https://raspida.com/release-raspberry-pi-imager  
執筆者は Raspbianの  
Linux raspberrypi 4.19.115-v7l+ #1305 SMP Fri Apr 17 11:59:28 BST 2020 armv7l GNU/Linux  
をインストールしました。  

# Dockerのinstall
- for raspberry-pi  
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

- docker desktop for mac(windows)  
下記からDLしてInstallしてください  
https://www.docker.com/products/docker-desktop

# gitのinstallと該当レポジトリのclone

任意のディレクトリにgitレポジトリをcloneしてください
```
$ git clone https://github.com/arcosaka/arcosaka

```

# docker buildの実行
cloneしたリポジトリでdocer_wsに移動して、docker buildを実行します。
```
$ cd arcosaka/2020/docker_ws/
$ docker build -t arc2020ros .

```
arc2020ros: このあと作成するコンテナの元になるイメージの名前です。
            任意の名前に変更可能です。

# コンテナの起動
dockerコンテナを起動します。

```
#Raspberry-Pi版
$ docker run --name arc2020cont --net host --privileged -u root -v /sys:/sys -v /dev/mem:/dev/mem -v /home/pi/.ros/:/root/.ros/ -v /home/pi/arcosaka/2020/arc_ws/:/root/catkin_ws/ -it arc2020ros:latest
#Docker-Desktop版
$ docker run --name arc2020cont -p 8085:8085 -p 9090:9090 --privileged -u root -v /sys:/sys -v /dev/mem:/dev/mem -v /home/pi/.ros/:/root/.ros/ -v /home/pi/arcosaka/2020/arc_ws/:/root/catkin_ws/ -it arc2020ros:latest
```
- 引数の簡単な意味
  - arc2020cont   
              コンテナの名前です。
              任意の名前に変更可能です。
  - -p 8085:8085  
  - -p 9090:9090  
               port8085,9090をhost os と dockerコンテナで共有します。
               roswwwで8085,9090を使用するためこのように設定します。
  - --privileged 
  - -u root
  - -v /sys:/sys  
                  host os とのディレクトリ共有です。  
                  /host_osディレクトリ:/dockerコンテナディレクトリ
  - -v /dev/mem:/dev/mem  
                  host os とのディレクトリ共有です。
  - -v /home/pi/.ros/:/root/.ros/  
                  host os とのディレクトリ共有です。
  - -v /home/pi/arcosaka/2020/arc_ws/:/root/catkin_ws/  
                  host os とのディレクトリ共有です。  
                  /home/pi/arcosaka/2020/arc_ws/はgitリポジトリをcloneしたディレクトリとあわせてください
  - -it  
                  bash起動の指定です。
  - arc2020ros:latest  
                  arc2020rosイメージの最新版を使用してコンテナを作成するという意味になります。

##### dockerのコマンドあれこれ
- コンテナから抜けるとき
  - Ctrl-p,Ctrl-qで抜けます。
- 再度コンテナに入る時
```
$ docker attach arc2020cont
```
- コンテナを停止する時
```
$ docker stop arc2020cont
```
- 停止したコンテナを起動してコンテナ内に入る時
```
$ docker start -i arc2020cont
```
- 現在存在するコンテナを確認する時
```
$ docker ps -a
```
- コンテナを削除する時
```
$ docker rm (コンテナ名)
```

# コンテナ内での作業開始
## rosのbuild
初回起動時及び  
msgなどを追加した場合は下記コマンドでbuildしなおしてください。
```bash
# catkin_make
# catkin_make install
```

## PATHの設定 
コンテナの中のbashに入ったのでrosのpath設定を実施します。
おそらくdocker startするたびに必要なはず
```bash
# source devel/setup.sh
```



# サンプルの実行
2019のWebGUIをサンプル実行します

```bash
$ roslaunch arc2020 client.launch
```
同じネットワーク内にあるデバイスから  
http://(ip-addr):8085/arc2020  
にアクセスすることでGUIを表示できます。  
note: ラズパイの8085ポートを通信可能なようにしておくこと  
      かんたんに実現するにはufwをinstallして下記コマンドを実行
      *コンテナ側ではなくhost側で実施する
```bash
$ sudo apt-get install ufw
$ sudo ufw allow 8085
$ sudo ufw reload
$ sudo ufw status
```

# Author

* 作成者
* 所属
* E-mail
