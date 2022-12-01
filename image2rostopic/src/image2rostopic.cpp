#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
 
#include <stdio.h>
#include <iostream>
#include <iomanip>

using namespace std;

int n=0;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image", 1);
  ros::Rate loop_rate(10);//image rostopic FPS
  while (nh.ok()) 
  {
    // cout<<"n:"<<n<<endl;
    ostringstream os;
    os<<"/media/user/Pxy_HardDisk/DataSET/rgbd-scenes-v2_imgs/rgbd-scenes-v2/imgs/scene_14/"//path
    << setw(5) << setfill('0') //format
    <<n 
    <<"-color.png";
    cout<<"read image at:  "<<os.str()<<endl;
    cv::Mat image = cv::imread(os.str(), CV_LOAD_IMAGE_COLOR);
    n++;
    if(image.empty()){
      printf("open error! or read finished!\n");
      return 0;
    }
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
      pub.publish(msg);
      ros::spinOnce();
      loop_rate.sleep();
  }
}
