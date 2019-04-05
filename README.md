# ros_kinect
## Overview
キネクトの情報をOpenNI&NiTE2経由で取得するROSパッケージ。
このパッケージは、freenectとは違い、デプスとカラー画像がキャリブレーションされた状態で扱えるようになっている。

## Setup
OpenNIとNiTE2のインストールが必須。  
パッケージ内の`install.sh`を実行すれば導入できる（ﾊｽﾞ）


## Usage

```
rosrun ros_kinect ros_kinect
```

## Node
**`name` ros_kinect**

### Publish Topic

* **`/ros_kinect/color`** キャリブレーションされたカラー画像を出力( sensor_msgs/Image -bgr8 )

* **`/ros_kinect/depth`** キャリブレーションされたデプス情報を出力( sensor_msgs/Image -mono16 )

