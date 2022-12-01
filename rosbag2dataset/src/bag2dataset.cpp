#include <iostream>
#include <stdio.h>
#include <cstdio>
#include <time.h>
#include <math.h>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <exception>
#include <vector>
#include <string>
#include <boost/foreach.hpp>
//#include <sys/io.h> //Linux下检查文件夹是否存在
#include <sys/uio.h> //ARM下检查文件夹是否存在
#include <unistd.h>
#include <dirent.h>
#include <sys/stat.h>
#include <stdlib.h>

#include <load_txt.h>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosgraph_msgs/Clock.h>
#include <rosbag/view.h>

#include <opencv2/imgproc/imgproc.hpp>      //图像处理
#include <opencv2/highgui/highgui.hpp>       //opencv GUI
#include <image_transport/image_transport.h>   //image_transport
#include <cv_bridge_34/cv_bridge.h>              //cv_bridge

#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>    //图像编码格式
#include <geometry_msgs/PoseStamped.h>
// #include <gps_common/GPSFix.h>
// #include <sensor_msgs/NavSatFix.h>

// #include <pcl/io/pcd_io.h>
// #include <pcl_ros/point_cloud.h>
// #include <pcl/io/pcd_io.h>      
// #include <pcl/point_types.h>     

using namespace std;

// #define foreach BOOST_FOREACH

bool Imu_dataWrite(string path, vector<string>& imu_stamp, vector<vector<double>>& imu_data);
bool Odom_dataWrite(string path, vector<string>& odom_stamp, vector<vector<double>>& odom_data);
bool Scan_dataWrite(string path, vector<Scan>& scan_data);
bool Scan_dataRead(string path, vector<Scan>& scan_data);

int createDirectory(std::string path);
bool rmdir(string &dirName);
bool dir_check(string path);
bool file_check(string path);
template<class T>
string toString(T data);

int main(int argc, char** argv)
{
    bool b_viewImage; //false 可以提高转换速度
    bool b_printData; //stamp，imu_data,建议开启

    string path_bagfile;
    string pathOut_rootPath;
    string pathOut_image_left;
    string pathOut_image_right;
    string pathOut_imu_raw;
    string pathOut_wheel_odom;
    string pathOut_scan;

    string topic_image_left;
    string topic_image_right;
    string topic_imu_raw;
    string topic_imu_raw_processed;
    string topic_wheel_odom;
    string topic_scan;

    ros::init(argc,argv,"rosbag2dataSet");
    ros::NodeHandle nh;

    nh.getParam("b_viewImage", b_viewImage);
    nh.getParam("b_printData", b_printData);

    nh.getParam("path_bagfile", path_bagfile);
    nh.getParam("pathOut_rootPath", pathOut_rootPath);
    nh.getParam("pathOut_image_left", pathOut_image_left);
    nh.getParam("pathOut_image_right", pathOut_image_right);
    nh.getParam("pathOut_imu_raw", pathOut_imu_raw);
    nh.getParam("pathOut_wheel_odom", pathOut_wheel_odom);
    nh.getParam("pathOut_scan", pathOut_scan);

    nh.getParam("topic_image_left", topic_image_left);
    nh.getParam("topic_image_right", topic_image_right);
    nh.getParam("topic_imu_raw", topic_imu_raw);
    nh.getParam("topic_imu_raw_processed", topic_imu_raw_processed);
    nh.getParam("topic_wheel_odom", topic_wheel_odom);
    nh.getParam("topic_scan", topic_scan);

    pathOut_image_left = pathOut_rootPath + pathOut_image_left;
    pathOut_image_right = pathOut_rootPath + pathOut_image_right;
    pathOut_imu_raw = pathOut_rootPath + pathOut_imu_raw;
    pathOut_wheel_odom = pathOut_rootPath + pathOut_wheel_odom;
    pathOut_scan = pathOut_rootPath + pathOut_scan;

    // if(nh.getParam("noise", noise))
    //     ROS_INFO("noise is %f", noise);
    // else
    //     ROS_WARN("didn't find parameter noise");

    // vector<Scan> test_data;
    // if(Scan_dataRead("/home/pxy/data/euroc/jinditianyi/data/scan.dat", test_data))
    // {cout<<"读取成功"<<endl<<test_data.size()<<endl;
    // }else{cout<<"读取失败"<<endl;}
    // for(auto da:test_data)
    // {
    //     cout<<da.stamp<<endl;
    //     cout<<da.angle_min<<endl;
    //     cout<<da.range_max<<endl;
    //     cout<<da.ranges.size()<<endl;
    //     cout<<da.intensities.size()<<endl;
    //     for(auto a:da.ranges) cout<<a<<", ";
    //     cout<<endl;
    //     for(auto a:da.intensities) cout<<a<<", ";
    //     cout<<endl;
    // }
    // return 0;

    cout<<endl<<"数据转换前，请务必确认路径设置！"<<endl;
    cout<<"path_bagfile: "<<path_bagfile<<endl;
    cout<<"pathOut_image_left: "<<pathOut_image_left<<endl;
    cout<<"pathOut_image_right: "<<pathOut_image_right<<endl;
    cout<<"pathOut_imu_raw: "<<pathOut_imu_raw<<endl;
    cout<<"pathOut_wheel_odom: "<<pathOut_wheel_odom<<endl;
    cout<<"pathOut_scan: "<<pathOut_scan<<endl;

    if(dir_check(pathOut_image_left) &&
        dir_check(pathOut_image_right) &&
        dir_check(pathOut_imu_raw) &&
        dir_check(pathOut_wheel_odom) &&
        dir_check(pathOut_scan)) {}
    else{
        cout<<"检查路径设置！ "<<endl;
        return -1;
    }

    pathOut_imu_raw += "imu_data.csv";
    pathOut_wheel_odom += "wheel_odom.csv";
    pathOut_scan += "scan.bin";
    if(file_check(pathOut_imu_raw) &&
        file_check(pathOut_wheel_odom) &&
        file_check(pathOut_scan) ){}
    else{
        cout<<"检查文件设置！ "<<endl;
        return -1;
    } 


    cout<<endl<<"数据转换前，请务必确认RosTopic设置！"<<endl;
    vector<string> topics;
    topics.push_back(topic_image_left);
    topics.push_back(topic_image_right);
    topics.push_back(topic_imu_raw);
    topics.push_back(topic_imu_raw_processed);
    topics.push_back(topic_wheel_odom);
    topics.push_back(topic_scan);
       
    cout<<"待转换话题如下: "<<endl;
    for(string &topic:topics)
    {
        cout<<topic<<endl;
    }
    if(b_viewImage)
    {
        cout<<endl<<"图像显示已打开，在roslaunch参数面板关闭后可提高转换速度！"<<endl<<endl;
    }
    else{
        cout<<endl<<"图像显示已关闭，在roslaunch参数面板打开后可显示左目图像！"<<endl<<endl;
    }
    cout<<"数据加载中，根据rosbag大小需要等待几秒到数分钟，请稍候！"<<endl<<endl;
    
    rosbag::Bag bag_input;

    bag_input.open(path_bagfile, rosbag::bagmode::Read); 

    rosbag::View view_data(bag_input, rosbag::TopicQuery(topics));

    double duration=view_data.getEndTime().toSec()-view_data.getBeginTime().toSec();
    cout<<"duration: "<<duration<<" 秒"<<endl<<endl;


    vector<string> imu_stamp;
    vector<vector<double>> imu_data;
    vector<string> odom_stamp;
    vector<vector<double>> odom_data;
    vector<Scan> scan_data;

    const char *lable = "|/-\\";
    int  z=0;
    
    for (rosbag::View::iterator it=view_data.begin(); it !=view_data.end(); ++it)
    {
        auto m=*it;

        std::string topic=m.getTopic();
        if(topic == topic_image_left)
        {
            sensor_msgs::Image::Ptr Image_msg = m.instantiate<sensor_msgs::Image>();

            cv_bridge::CvImagePtr cv_ptr;
            try{
                cv_ptr = cv_bridge::toCvCopy(Image_msg, sensor_msgs::image_encodings::BGR8);
            }
            catch(cv_bridge::Exception& e){
                ROS_ERROR("Cv_bridge Exception: %s", e.what());
                return -1;
            }

            // std::cout<<"left image stamp : "<<cv_ptr->header.stamp.toSec()<<std::endl; //double
            if(b_printData) std::cout<<"left image stamp : "<<cv_ptr->header.stamp<<std::endl;
            
            
            double progress=((cv_ptr->header.stamp.toSec()-view_data.getBeginTime().toSec())*100.0) / duration;
            printf("转换进度：[%c   %.3f%%   %c]\r",lable[z%4],progress,lable[z%4]); if(++z>100) z=0;
            fflush(stdout);

            cv::Mat image = cv_ptr->image;
            string image_name=toString(cv_ptr->header.stamp);
            image_name.erase(std::remove(image_name.begin(), image_name.end(), '.'), image_name.end());
            imwrite(pathOut_image_left+image_name+".png",image);

            if(b_viewImage)
            {
                cv::imshow("image_left", image);  
                cv::waitKey(1); //ms延时 ;1：20
            }
        }

        if(topic == topic_image_right)
        {
            sensor_msgs::Image::Ptr Image_msg = m.instantiate<sensor_msgs::Image>();

            cv_bridge::CvImagePtr cv_ptr;
            try{
                cv_ptr = cv_bridge::toCvCopy(Image_msg, sensor_msgs::image_encodings::BGR8);
            }
            catch(cv_bridge::Exception& e){
                ROS_ERROR("Cv_bridge Exception: %s", e.what());
                return -1;
            }

            // std::cout<<"right image stamp : "<<cv_ptr->header.stamp.toSec()<<std::endl; //double
            if(b_printData) std::cout<<"right image stamp : "<<cv_ptr->header.stamp<<std::endl;

            cv::Mat image = cv_ptr->image;
            string image_name=toString(cv_ptr->header.stamp);
            image_name.erase(std::remove(image_name.begin(), image_name.end(), '.'), image_name.end());
            imwrite(pathOut_image_right+image_name+".png",image);

            if(b_viewImage)
            {
                cv::imshow("image_right", image);  
                cv::waitKey(1); //ms延时 ;1：20
            }
        }

        if(topic == topic_imu_raw)
        {
            sensor_msgs::Imu::ConstPtr imu = m.instantiate<sensor_msgs::Imu>();
            if(b_printData) cout<<"imu stamp : "<<imu->header.stamp<<":  ["
            <<imu->angular_velocity.x<<", "<<imu->angular_velocity.y<<", "<<imu->angular_velocity.z<<"];  ["
            <<imu->linear_acceleration.x<<", "<<imu->linear_acceleration.y<<", "<<imu->linear_acceleration.z<<"] "<<endl;
            // cout<<imu->orientation.x<<", "<<imu->orientation.y<<", "<<imu->orientation.z<<", "
            // <<imu->orientation.w<<", "<<endl;

            string imu_s=toString(imu->header.stamp);
            imu_s.erase(std::remove(imu_s.begin(), imu_s.end(), '.'), imu_s.end());
            imu_stamp.push_back(imu_s);
            vector<double> imu_data_line{
                imu->angular_velocity.x,
                imu->angular_velocity.y,
                imu->angular_velocity.z,
                imu->linear_acceleration.x,
                imu->linear_acceleration.y,
                imu->linear_acceleration.z
            };
            imu_data.push_back(imu_data_line);
        }

        if(topic == topic_wheel_odom)
        {
            nav_msgs::Odometry::ConstPtr odom = m.instantiate<nav_msgs::Odometry>();
            if(b_printData) cout<<"odom stamp : "<<odom->header.stamp<<":  ["
            <<odom->pose.pose.position.x<<", "<<odom->pose.pose.position.y<<", "<<odom->pose.pose.position.z<<"];  ["
            <<odom->pose.pose.orientation.x<<", "<<odom->pose.pose.orientation.y<<", "<<odom->pose.pose.orientation.z<<", "<<odom->pose.pose.orientation.w<<"] "<<endl;

            string odom_s=toString(odom->header.stamp);
            odom_s.erase(std::remove(odom_s.begin(), odom_s.end(), '.'), odom_s.end());
            odom_stamp.push_back(odom_s);
            vector<double> odom_data_line{
                odom->pose.pose.position.x,
                odom->pose.pose.position.y,
                odom->pose.pose.position.z,
                odom->pose.pose.orientation.x,
                odom->pose.pose.orientation.y,
                odom->pose.pose.orientation.z,
                odom->pose.pose.orientation.w,
                odom->twist.twist.linear.x,
                odom->twist.twist.linear.y,
                odom->twist.twist.linear.z,
                odom->twist.twist.angular.x,
                odom->twist.twist.angular.y,
                odom->twist.twist.angular.z
            };
            odom_data.push_back(odom_data_line);
        }


        if(topic == topic_scan)
        {
            sensor_msgs::LaserScan::ConstPtr scan = m.instantiate<sensor_msgs::LaserScan>();
            if(b_printData) cout<<"scan stamp : "<<scan->header.stamp<<endl;
            // <<":  ["<<scan->angle_min<<","<<scan->ranges[0]<<endl;

            Scan scan_line;
            string scan_s=toString(scan->header.stamp);
            scan_s.erase(std::remove(scan_s.begin(), scan_s.end(), '.'), scan_s.end());

            scan_line.stamp=scan_s;
            scan_line.angle_min=scan->angle_min;
            scan_line.angle_max=scan->angle_max;
            scan_line.angle_increment=scan->angle_increment;
            scan_line.time_increment=scan->time_increment;
            scan_line.scan_time=scan->scan_time;
            scan_line.range_min=scan->range_min;
            scan_line.range_max=scan->range_max;

            for(auto range:scan->ranges) scan_line.ranges.push_back(range);
            for(auto intensitie:scan->intensities) scan_line.intensities.push_back(intensitie);

            scan_data.push_back(scan_line);
        }
        
        // if(topic == topic_new)
        // {
        // }
    }

    
    bag_input.close();

    bool bImuW=Imu_dataWrite(pathOut_imu_raw, imu_stamp, imu_data);
    bool bOdomW=Odom_dataWrite(pathOut_wheel_odom, odom_stamp, odom_data);
    bool bScanW=Scan_dataWrite(pathOut_scan, scan_data);

    cout<<endl<<endl<<"Image数据转换完成！"<<endl<<endl;
    if(bImuW) cout<<endl<<"IMU数据转换完成！"<<endl<<endl;
    if(bOdomW) cout<<endl<<"Odom数据转换完成！"<<endl<<endl;
    if(bScanW) cout<<endl<<"Scan数据转换完成！"<<endl<<endl;

    if(bImuW && bOdomW && bScanW)
    {
        cout<<endl<<"数据全部转换完成！"<<endl<<endl;
    }
    else
    {
        if(!bImuW) cout<<endl<<"IMU数据转换失败，请检查！"<<endl<<endl;
        if(!bOdomW) cout<<endl<<"Wheel_odom数据转换失败，请检查！"<<endl<<endl;
        if(!bScanW) cout<<endl<<"Scan数据转换失败，请检查！"<<endl<<endl;
        return -1;
    }

    // ros::spin();
    ros::spinOnce();
    // ros::shutdown();
    return 0;
}

bool Imu_dataWrite(string path, vector<string>& imu_stamp, vector<vector<double>>& imu_data)
{
    ofstream file(path);
    if (file)
    {
        for(size_t i=0;i<imu_data.size()&&i<imu_stamp.size();i++)
        {
            file << imu_stamp[i] << "," << imu_data[i][0] << "," << imu_data[i][1] << "," << imu_data[i][2]
            << ","<< imu_data[i][3] << "," << imu_data[i][4] << "," << imu_data[i][5] << "\n";
        }
        file.close();
    }
    else
    {
        cout<<"Imu_dataWrite ERROR !! "<<path<<endl;
        return false;
    }
    return true;
}

bool Odom_dataWrite(string path, vector<string>& odom_stamp, vector<vector<double>>& odom_data)
{
    ofstream file(path);
    if (file)
    {
        for(size_t i=0;i<odom_data.size()&&i<odom_stamp.size();i++)
        {
            file << odom_stamp[i] << "," << odom_data[i][0] << "," << odom_data[i][1] << "," << odom_data[i][2]
            << ","<< odom_data[i][3] << "," << odom_data[i][4] << "," << odom_data[i][5]<< "," << odom_data[i][6] 
            << "," << odom_data[i][7] << "," << odom_data[i][8] << "," << odom_data[i][9]
            << "," << odom_data[i][10] << "," << odom_data[i][11] << "," << odom_data[i][12]<< "\n";
        }
        file.close();
    }
    else
    {
        cout<<"Odom_dataWrite ERROR !! "<<path<<endl;
        return false;
    }
    return true;
}

bool Scan_dataWrite(string path, vector<Scan>& scan_data)
{
    ofstream outFile(path, ios::out | ios::binary);
    if(outFile)
    {
        for (Scan &scan : scan_data)
        {
            int L = scan.stamp.size();
            outFile.write((char*)&L, sizeof(L));
            for (char c : scan.stamp)
            {
                outFile.write((char*)&c, sizeof(c));
            }
            outFile.write((char*)&scan.angle_min, sizeof(scan.angle_min));
            outFile.write((char*)&scan.angle_max, sizeof(scan.angle_max));
            outFile.write((char*)&scan.angle_increment, sizeof(scan.angle_increment));
            outFile.write((char*)&scan.time_increment, sizeof(scan.time_increment));
            outFile.write((char*)&scan.scan_time, sizeof(scan.scan_time));
            outFile.write((char*)&scan.range_min, sizeof(scan.range_min));
            outFile.write((char*)&scan.range_max, sizeof(scan.range_max));
            L = scan.ranges.size();
            outFile.write((char*)&L, sizeof(L));
            for (double &da : scan.ranges)
            {
                outFile.write((char*)&da, sizeof(da));
            }
            L = scan.intensities.size();
            outFile.write((char*)&L, sizeof(L));
            for (double &da : scan.intensities)
            {
                outFile.write((char*)&da, sizeof(da));
            }
        }
        outFile.close();
    }
    else
    {
        cout<<"Scan_dataWrite ERROR !! "<<path<<endl;
        return false;
    }
    return true;
}

bool Scan_dataRead(string path, vector<Scan>& scan_data)
{
    ifstream inFile(path,ios::in|ios::binary); //二进制读方式打开
    if(!inFile) {
        cout<<"Scan_dataRead ERROR !! "<<path<<endl;
        return false;
    }
    Scan scan;
   while(!inFile.eof())
	{
		Scan scan;
		int Ls = 0; inFile.read((char *)&Ls, sizeof(Ls));
		scan.stamp.resize(Ls);
		for (int i = 0; i < Ls; i++)
		{
			inFile.read((char *)&scan.stamp[i], sizeof(scan.stamp[i]));
		}
		inFile.read((char *)&scan.angle_min, sizeof(scan.angle_min));
		inFile.read((char *)&scan.angle_max, sizeof(scan.angle_max));
		inFile.read((char *)&scan.angle_increment, sizeof(scan.angle_increment));
		inFile.read((char *)&scan.time_increment, sizeof(scan.time_increment));
		inFile.read((char *)&scan.scan_time, sizeof(scan.scan_time));
		inFile.read((char *)&scan.range_min, sizeof(scan.range_min));
		inFile.read((char *)&scan.range_max, sizeof(scan.range_max));
		int Lr = 0; inFile.read((char *)&Lr, sizeof(Lr));
		scan.ranges.resize(Lr);
		for (int i = 0; i < Lr; i++)
		{
			inFile.read((char *)&scan.ranges[i], sizeof(scan.ranges[i]));
		}
		int Li = 0; inFile.read((char *)&Li, sizeof(Li));
		scan.intensities.resize(Li);
		for (int i = 0; i < Li; i++)
		{
			inFile.read((char *)&scan.intensities[i], sizeof(scan.intensities[i]));
		}
		scan_data.push_back(scan);
	}
    inFile.close();

    return true;
}

bool file_check(string path)
{
    fstream fs;
	fs.open(path, ios::in);
    if (!fs)
    {
        cout << "不存在该文件," ;
        //创建文件
		ofstream fout(path);
		if (fout)
		{
            cout<<"创建文件： "<<path<<endl;
			// 执行完操作后关闭文件句柄
			fout.close();
		}
        else
        {
            cout<<"请检查路径： "<<path<<endl;
        }
    }
    else
    {
        cout<<"文件： "<<path<<"已存在，进行删除！"<<endl;
        if(remove(path.c_str())==0)
        {
            cout<<"删除成功后重新创建！"<<endl;
        }
        else
        {
            cout<<"删除失败！"<<endl;
            return false;
        }
    }
    return true;
}

bool dir_check(string path)
{
    if(rmdir(path))
    {
        cout<<"路径："<<path<<"已经存在，删除后准备重新创建！"<<endl;
        int ret=createDirectory(path);
        if (ret == -1) 
        {
            cout<<"路径："<<path<<"创建失败！"<<endl;
            return false;
        }
        else
        {
            cout<<"路径："<<path<<"创建成功！"<<endl;
        }
    }
    else
    {
        int ret=createDirectory(path);
        if (ret == -1) 
        {
            cout<<"路径："<<path<<"创建失败！"<<endl;
            return false;
        }
        else
        {
            cout<<"路径："<<path<<"创建成功！"<<endl;
        }
    }
    return true;
}


int createDirectory(std::string path)
{
	int len = path.length();
	char tmpDirPath[256] = { 0 };
	for (int i = 0; i < len; i++)
	{
		tmpDirPath[i] = path[i];
		if (tmpDirPath[i] == '\\' || tmpDirPath[i] == '/')
		{
			if (access(tmpDirPath, 0) == -1)
			{
				int ret = mkdir(tmpDirPath,S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
				if (ret == -1) return ret;
			}
		}
	}
	return 0;
}

// 递归删除
bool rmdir(string &dirName)
{
    DIR *dir;
    struct dirent *dirinfo;
    struct stat statbuf;
    lstat(dirName.c_str(), &statbuf);
 
 
    if (S_ISREG(statbuf.st_mode)) // 是否是文件
    {
        remove(dirName.c_str());
    }
    else if (S_ISDIR(statbuf.st_mode)) // 是否是目录
    {
        if ((dir = opendir(dirName.c_str())) == NULL)
        {
            return false;
        }   
 
        while ((dirinfo = ::readdir(dir)) != NULL)
        {
            if (strcmp(dirinfo->d_name, ".") == 0 || strcmp(dirinfo->d_name, "..") == 0) // 是否是特殊目录
                continue;
            
            string absPath = dirName + "/" + string(dirinfo->d_name);
            rmdir(absPath);
        }
        closedir(dir);
 
        rmdir(dirName.c_str());
    }
    else
    {
        return false;
    }
 
    return true;
}

template<class T>
string toString(T data) //会存在精度问题，慎用
{
    ostringstream os; 
    os << data; 
    return os.str();
}

// {
    // if (topic == lidar_topic)
        // {
        //     sensor_msgs::PointCloud2::ConstPtr msgPtr=m.instantiate<sensor_msgs::PointCloud2>();
        //     if (msgPtr !=NULL  && msgPtr->header.stamp.sec>1)
        //     {

        //         std::stringstream ss;
        //         std::cout << "j = " << j << std::endl;
        //         j++;
        //         ss << j;
        //         std::string tmp_str = ss.str()+ ".bin";
        //         std::string bin_file = bin_path + tmp_str;

        //         pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);

        //         sensor_msgs::PointCloud2 pcd_frame=*msgPtr;
        //         ros::Time lidar_time=pcd_frame.header.stamp;
        //         std::string time_stamp_str=stampToString(lidar_time);

        //         std::cout<<"lidar_time.sec = " <<lidar_time.sec <<" lidar_time.nsec = "<< lidar_time.nsec <<std::endl;
        //         std::cout<<"converted time = "<<time_stamp_str<<std::endl;

        //         pcl::fromROSMsg(pcd_frame, *cloud);
        //         pcd2bin( cloud, bin_file);
        //         time_stamp_file<<time_stamp_str <<endl;
        //     }
        // }
// }

//  std::string stampToString(const ros::Time& stamp, const std::string format="%Y-%m-%d %H:%M:%S")
//  {
//     const int output_size = 100;
//     char output[output_size];
//     std::time_t raw_time = static_cast<time_t>(stamp.sec);
//     struct tm* timeinfo = localtime(&raw_time);
//     std::strftime(output, output_size, format.c_str(), timeinfo);
//     std::stringstream ss; 
//     ss << std::setw(9) << std::setfill('0') << stamp.nsec;  
//     const size_t fractional_second_digits = 4;
//     return std::string(output) + "." + ss.str().substr(0, fractional_second_digits);
// }

// //Transform PCD 2 BIN
// void pcd2bin (pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, string& out_file)
// { 

//   ofstream bin_file(out_file.c_str(),ios::out|ios::binary|ios::app);
//   if(!bin_file.good()) cout<<"Couldn't open "<<out_file<<endl;  

//   for (size_t i = 0; i < cloud->points.size (); ++i)
//   {
//     bin_file.write((char*)&cloud->points[i].x,3*sizeof(float)); 
//     bin_file.write((char*)&cloud->points[i].intensity,sizeof(float));
//     //cout<<    cloud->points[i]<<endl;
//   }
    
//   bin_file.close();
// }
// static std::vector<std::string> file_lists;

// //Read the file lists
// void read_filelists(const std::string& dir_path,std::vector<std::string>& out_filelsits,std::string type)
// {
//     struct dirent *ptr;
//     DIR *dir;
//     dir = opendir(dir_path.c_str());
//     out_filelsits.clear();
//     while ((ptr = readdir(dir)) != NULL){
//         std::string tmp_file = ptr->d_name;
//         if (tmp_file[0] == '.')continue;
//         if (type.size() <= 0){
//             out_filelsits.push_back(ptr->d_name);
//         }else{
//             if (tmp_file.size() < type.size())continue;
//             std::string tmp_cut_type = tmp_file.substr(tmp_file.size() - type.size(),type.size());
//             if (tmp_cut_type == type){
//                 out_filelsits.push_back(ptr->d_name);
//             }
//         }
//     }
// }
