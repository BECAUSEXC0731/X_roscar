# 扫地机器人雷达 ROS2 驱动

## 介绍视频
Bilibili: [【开源】超低成本的激光雷达导航方案](https://www.bilibili.com/video/BV15Li4B9EDW/)  
Youtube: [【开源】超低成本的激光雷达导航方案](https://www.youtube.com/watch?v=J-uE7xHYDxY)

## 特别感谢
感谢智元机器人的大力支持，可扫描如下二维码获取更多开发资料。
<div align="center">
  <img src="./media/D1_EDU.jpg" width="400">
</div><br>

## 教材书籍
《机器人操作系统（ROS2）入门与实践》
<div align="center">
  <img src="./media/book_1.jpg" width="300">
</div><br>

淘宝链接：[《机器人操作系统（ROS2）入门与实践》](https://world.taobao.com/item/820988259242.htm)

## 系统版本

- ROS2 Humble (Ubuntu 22.04)

## 使用说明
1. 获取源码：
  ```
  cd ~/ros2_ws/src/
  git clone https://github.com/zhangwanjie/lidar_pkg_ros2.git
  ```
  备用地址：
  ```
  cd ~/catkin_ws/src/
  git clone https://gitee.com/s-robot/lidar_pkg_ros2.git
  ```
2. 插上雷达，执行如下指令：  
  ```
  ls /dev/ttyUSB* 
  ```
  或者
  ```
  ls /dev/ttyACM* 
  ```
  拔掉雷达，再执行上述指令。消失的那个设备就是雷达设备。  
  3. 设置访问权限
  ```
  sudo usermod -a -G dialout $USER 
  ```
4. 修改设备参数：  
  修改文件 /config/lidar_params.yaml  
  ```
  lidar_node:
  ros__parameters:
    port_name: "/dev/ttyUSB*"  #修改为具体的设备名称
    frame_id: "laser"
  ```
5. 编译
  ```
  cd ~/ros2_ws
  colcon build --symlink-install
  ```
6. 运行测试：
  ```
  ros2 launch lidar_pkg monitor.launch.py 
  ```
7. 对于没有外接显示设备的情况，使用如下指令启动雷达：
  ```
  ros2 launch lidar_pkg lidar.launch.py 
  ```
  
## 其他
1. 雷达底座的3D打印模型：[models/lidar_mount.stl](./models/lidar_mount.stl)

## 分享&交流
关注我，后续解锁更多玩法 o(*≧▽≦)ツ
<div align="center">
  <img src="./media/AJQR.jpg" width="400">
</div><br>


