# fisher-stereo-calibration

-----

## 文件说明
* fisheye-stereo-calibration: plain cmake工程，标定计算程序，根据imgs中的图像计算生成yml文件(dependency: `sudo apt install libpopt-dev`)；
* src/capture_image: ros工程，采集 标定图像 到fisheye-stereo-calibration/imgs文件夹；
* src/stereo_fisheye_rectify: ros工程，根据标定计算出的yml文件，对raw图像矫正生成rect图像；

## 执行顺序
* 启动摄像头: 启动双目摄像头ros wrapper程序 `roslaunch mynt_eye_ros_wrapper mynteye.launch ir_control:=0`

* 采集图像
  - 根据实际情况修改capture_image/src/capture_image.cpp中102行和103行的路径，重新编译；
  - 启动程序
    - `cd ~/catkin_calib/src/fisheye-stereo-calibration`
    - `source devel/setup.bash`
    - rm imgs/*.*  ; ls -l imgs/
    - `roslaunch capture_image capture_image.launch`，开始采集标定图像；
  - 鼠标焦点定位到图象预览窗口，按w键会保存图像；

* 计算标定参数
  - 根据标定板实际情况，修改fisheye-stereo-calibration文件夹calib.sh中的参数；
  - 运行脚本`./calib.sh`进行标定计算，生成yml文件，将该文件放到stereo_fisheye_rectify/params中；
  - scp cam_stereo.yml firefly@10.0.0.24:/home/firefly/sweeper/src/sweeper-camera/stereo_fisheye_rectify/params/cam_stereo.yml
  - ssh firefly@10.0.0.24  && cd /home/firefly/sweeper/src/sweeper-location-core/VINS-Mono/config/mynteye

* calib mynteye 240x376
  - roslaunch mynt_eye_ros_wrapper mynteye_calib.launch ir_control:=0  (on firefly)
  - cd /root/catkin_ws/src/sweeper/src/sweeper-camera/mynt-eye-sdk-2/wrappers/ros
  - source devel/setup.bash
  - roslaunch mynteye_calibration mynteye_calibration.launch    #- rosrun camera_calibration cameracalibrator.py --size 11x8 --square 0.03 --no-service-check --approximat=0.1 right:=/mynteye/left/image_raw  left:=/mynteye/left/image_raw
  - scp /root/catkin_ws/src/sweeper/src/sweeper-camera/mynt-eye-sdk-2/wrappers/ros/src/mynt-eye-calibration/results/SN0610303800090720.conf   firefly@192.168.125.117:~/.ros/
  - cd /home/firefly/sweeper/src/sweeper-camera/mynt-eye-sdk-2/tools/_output/bin/writer  && ./img_params_writer ~/.ros/SN0610303800090720.conf


