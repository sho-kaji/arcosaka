# ARC2020（）

2020年用のリポジトリ（ディレクトリ）です。

# Dockerのinstall
まずは下記にしたがってDockerをラズパイ環境にinstallしてください
```
コマンドライン

$ sudo apt update && sudo apt upgrade -y
$ sudo apt-get install apt-transport-https ca-certificates curl gnupg2 software-properties-common -y
$ curl -sSL https://get.docker.com | sh
$ docker -v
Docker version 19.03.7, build 7141c19
$ sudo usermod -aG docker pi

```
# gitのinstallと該当レポジトリのclone

任意のディレクトリにgitレポジトリをcloneしてください
```
$ git clone https://github.com/arcosaka/arcosaka

```

# dockar buildの実行
cloneしたリポジトリでdocer_wsに移動して、docker buildを実行します。
```
$ cd arcosaka/2020/docker_ws/
$ docker build -t arc2020ros .

```
arc2020rosは任意の名前に変更可能。

# コンテナの起動
dockerコンテナを起動します。

```
$ docker run --name arc2020cont --net host --privileged -u root -v /sys:/sys -v /dev/mem:/dev/mem -v /home/pi/.ros/:/root/.ros/ -v /home/pi/arcosaka/2020/arc_ws/:/root/catkin_ws/ -it arc2020ros:latest
```
/home/pi/arcosaka/2020/arc_ws/はgitリポジトリをcloneしたディレクトリとあわせてください


# コンテナ内での作業開始 rosのbuild
コンテナの中のbashに入ったのでmake、make installを実施します。

```bash
$ catkin_make
$ catkin_make install
$ source /ros_entrypoint.sh
$ source devel/setup.sh
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

