# 环境(ubuntu20.04):  

请根据ubuntu版本自行修改
- ros-noetic
```
wget http://fishros.com/install -O fishros && fishros
```
- camera_calibration
```
sudo apt-get install ros-noetic-camera-calibration
```

# 编译:

```
catkin_make
```

# 设置参数:
- 使用大恒相机:

    于`image_publisher/config/daheng.yaml`中更改图像尺寸，曝光时间等参数

- 使用视频:
  
    于`image_publisher/launch/video.launch`中更改视频路径

# 运行:

- source
```
source devel/setup.bash
```
- 发布图像

    - 使用大恒相机:
    ```
    roslaunch image_publisher daheng.launch
    ```
    - 使用视频:
    ```
    roslaunch image_publisher video.launch
    ```

- 标定

`--size` 标定板角点个数（格子数-1）

`--square` 标定板每个格子的长度（单位：m)

`image:=` 图像消息的名称
```
rosrun camera_calibration cameracalibrator.py --size 11x8 --square 0.03 image:=/daheng_camera/image_raw camera:=/daheng_camera --no-service-check
```

![image](标定.png)
