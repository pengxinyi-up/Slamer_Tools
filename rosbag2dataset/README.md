# rosbag2dataSet
A ros package converts rosbag to a dataset data format, such as Euroc

* Thanks for the work: [bag2kitti_tool](https://github.com/xiaxin2000/bag2kitti_tool)

## Instalation
This instalation process is for catkin, Assuming that your catkin workspace is under ~/catkin_ws, if not replace ~/catkin_ws with appropriate location. 

```bash
cd ~/catkin_ws/
git clone https://github.com/pengxinyi-up/rosbag2dataSet
catkin_make
```

## 配置launch文件参数
```bash
    <param name = "b_viewImage" type = "bool" value = "true" /> <!--是否显示图像，false 可以提高转换速度-->
    <param name = "b_printData" type = "bool" value = "true" /> <!--stamp，imu_data,建议开启-->

    <param name = "path_bagfile" type = "string" value = "/home/pxy/data/2022-06-27-18-00-13.bag" />
    <param name = "pathOut_image_left" type = "string" value = "/home/pxy/data/DataSet_RK3/data1/image_left/" />
    <param name = "pathOut_image_right" type = "string" value = "/home/pxy/data/DataSet_RK3/data1/image_right/" />
    <param name = "pathOut_imu_raw" type = "string" value = "/home/pxy/data/DataSet_RK3/data1/imu/imu_data.csv" />
    
    <param name = "topic_image_left" type = "string" value = "/mynteye/left/image_mono" />
    <param name = "topic_image_right" type = "string" value = "/mynteye/right/image_mono" />
    <param name = "topic_imu_raw" type = "string" value = "/mynteye/imu/data_raw" />
    <param name = "topic_imu_raw_processed" type = "string" value = "/mynteye/imu/data_raw_processed" />
    <param name = "topic_wheel_odom" type = "string" value = "/StereoInertial_node/wheel_odom" />
    <param name = "topic_scan" type = "string" value = "/scan" />

```

## 启动
```bash
roslaunch rosbag2dataset rosbag2dataSet.launch
```
