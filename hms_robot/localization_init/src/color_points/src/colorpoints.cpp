#include "ros/ros.h"
#include <iostream>
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/Image.h"
#include <deque>
#include <queue>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <vector>
#include <pcl/io/ply_io.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "my_custom_msgs/CustomMessage.h"

bool flag= false;
std::deque<sensor_msgs::PointCloud> lidarBuffer;
std::deque<double>  time_buffer;
double last_timestamp_lidar;
sensor_msgs::PointCloud radarData; 
pcl::PointCloud<pcl::PointXYZRGB>::Ptr accumulated_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

void lidarCallback(const sensor_msgs::PointCloud::ConstPtr& msg)
{
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidarBuffer.clear();
    }

    time_buffer.push_back(msg->header.stamp.toSec());
    last_timestamp_lidar = msg->header.stamp.toSec();
   
    lidarBuffer.push_back(*msg);
}




// ros::Time target_time;  // 给定的目标时间
bool odom_received = false;  // 是否接收到里程计消息
my_custom_msgs::CustomMessage closest_odom;  // 距离目标时间最近的里程计消息
std::queue<my_custom_msgs::CustomMessage> odom_buffer; 

Eigen::Matrix3d R1;
Eigen::Matrix3d R2;
Eigen::Vector3d T1;
Eigen::Vector3d T2;

void odomCallback(const my_custom_msgs::CustomMessage::ConstPtr& msg)
{
    // 从odom消息中获取时间戳
    odom_buffer.push(*msg);

    // 计算当前odom消息的时间与目标时间的差值
    // double time_diff = last_timestamp_lidar - odom_time;
    // if (time_diff < 0) time_diff = -time_diff;  // 取绝对值

    // 如果是第一条odom消息，或者当前消息距离目标时间更近，则更新closest_odom
   
}



std::queue<cv::Mat> imageBuffer;

cv::Mat the_image;
std::deque<double>  image_timebuffer;
double last_image_time;
double lidar_image_dif;
int row, col;
std::vector<std::vector<int>> color_vector;
std::shared_ptr<sensor_msgs::Image const> imagePtr;

cv_bridge::CvImagePtr cv_ptr;

void cameraCallback(const sensor_msgs::Image::ConstPtr& msg)
{

    // ROS_INFO("Received image with encoding: %s", msg->encoding.c_str());
    ROS_INFO("Received image with encoding: %s", msg->encoding.c_str());

    // 将ROS图像消息转换为OpenCV中的Mat类型
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    } 
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat image = cv_ptr->image;
    //printf("!!!!!!image row is :%d\n", image.rows);

    imageBuffer.push(cv_ptr->image);
    // 获取时间戳
    //imageQueue.push(msg);
    double timestamp = msg->header.stamp.toSec();
    image_timebuffer.push_back(msg->header.stamp.toSec());


}




// void getUV(const cv::Mat &matrix_in, const cv::Mat &matrix_out, float x, float y, float z, float* UV) {
//     double matrix3[4][1] = {x, y, z, 1};
//     cv::Mat coordinate(4, 1, CV_64F, matrix3);
    
//     // calculate the result of u and v
//     cv::Mat result = matrix_in*matrix_out*coordinate;
//     float u = result.at<double>(0, 0);
//     float v = result.at<double>(1, 0);
//     float depth = result.at<double>(2, 0);
    
//     UV[0] = u / depth;
//     UV[1] = v / depth;
// }

int in=0;

void getUV(const cv::Mat &cameraMatrix, const cv::Mat &rotation, const cv::Mat &translation, float x, float y, float z, float* UV) {
    // 将三维点 (x, y, z) 转换成齐次坐标表示
    cv::Mat point3d = (cv::Mat_<double>(3,1) << x, y, z);

    // 将三维点从世界坐标系转换到相机坐标系
    cv::Mat pointInCamera = rotation * point3d + translation;

    // 将点投影到相机平面上
    cv::Mat point2d = cameraMatrix * pointInCamera;
    point2d /= point2d.at<double>(2);  // 归一化得到(u/v, 1)

    // 提取归一化坐标
    float u = static_cast<float>(point2d.at<double>(0));
    float v = static_cast<float>(point2d.at<double>(1));

    // 保存结果
    UV[0] = u;
    UV[1] = v;
}

// get RGB value of the lidar point
void getColor(const cv::Mat &matrix_in, const cv::Mat &rotation, const cv::Mat &translation, float x, float y, float z, int row, int col, const std::vector<std::vector<int>> &color_vector, int* RGB) {
    float UV[2] = {0, 0}; 
    getUV(matrix_in, rotation, translation, x, y, z, UV);  // get U and V from the x,y,z
    
    int u = int(UV[0]);
    int v = int(UV[1]);
    if(u<col&&v<row)
    {
        in++; //
    }

    int32_t index = v*col + u;
    if (index < row*col && index >= 0) {
        RGB[0] = color_vector[index][0];
        RGB[1] = color_vector[index][1];
        RGB[2] = color_vector[index][2];
    }
}


typedef pcl::PointXYZRGB PointType;
double matrix1[3][3] = {{1736.19, 0.0, 1310.4}, {0.0, 1739.6, 1015.85}, {0.0, 0.0, 1.0}}; 
double matrix2[3][4] = {{0, -1, 0, 0}, {0, 0, -1, 0}, {1, 0, 0, 0.1}};
        // transform into the opencv matrix
    cv::Mat rotation = (cv::Mat_<double>(3, 3) << 
                        0, -1, 0,
                        0, 0, -1,
                        1, 0, 0);
    cv::Mat translation = (cv::Mat_<double>(3, 1) << 0, 0, 0.1);


sensor_msgs::PointCloud2 output;

Eigen::Matrix3d R ;
Eigen::Vector3d T ;

void make_color_points() //进行点云着色
{
            cv::Mat matrix_in(3, 3, CV_64F, matrix1); // 内参矩阵
            //cv::Mat matrix_out(3, 4, CV_64F, matrix2); // 外参增广矩阵 [R|t]
            //获取像素
            int count = 0;
            int row = the_image.rows; // height
            int col = the_image.cols; // width
            std::cout << "row is:" << row << std::endl;
            std::cout << "col is:" << col << std::endl;

            // 获取像素的RGB
            color_vector.resize(row*col);
            for (unsigned int i = 0; i < color_vector.size(); ++i) {
                color_vector[i].resize(3);
            }
            
            // read photo and get all RGB information into color_vector
            ROS_INFO("Start to read the photo ");
            for (int v = 0; v < row; ++v) { // height
                for (int u = 0; u < col; ++u) { // width
                    // opencv图片 the 3 channels are BGR
                    color_vector[v*col + u][0] = the_image.at<cv::Vec3b>(v, u)[0];
                    color_vector[v*col + u][1] = the_image.at<cv::Vec3b>(v, u)[1];
                    color_vector[v*col + u][2] = the_image.at<cv::Vec3b>(v, u)[2];
                }
            }

            //获取颜色；
            pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>); // RGB点云
            cloud->is_dense = false;
            cloud->height = 1;
            cloud->width = radarData.points.size(); // get the point number of lidar data
            cloud->points.resize(cloud->width);
            cloud->header.stamp =  ros::Time::now().toSec();
            for(uint64_t i = 0; i < radarData.points.size(); ++i) {
                float x = radarData.points[i].x;
                float y = radarData.points[i].y;
                float z = radarData.points[i].z;
                
                // ignore the invalid point
                if(x == 0 && y == 0 && z == 0) {  
                    continue;

                }
                
                // set coordinate for the cloud point

                // set the RGB for the cloud point  
                int RGB[3] = {0, 0, 0}; 
                getColor(matrix_in, rotation,translation, x, y, z, row, col, color_vector, RGB);  // 获取对应像素的RGB
                // ignore the unexisting point
                if (RGB[0] == 0 && RGB[1] == 0 && RGB[2] == 0) {  
                    count++;                                      
                    continue;
                
                }
                Eigen::Vector3d Point(x, y, z);
                Eigen::Vector3d Point_world;

                Point_world = R1*Point + T1;

                cloud->points[i].x = Point_world.x();
                cloud->points[i].y = Point_world.y();
                cloud->points[i].z = Point_world.z();
                cloud->points[i].r = RGB[0]; 
                cloud->points[i].g = RGB[1]; 
                cloud->points[i].b = RGB[2];

            }
            printf("unexisting points number is :%d\n",count);
            printf("existing points number is :%d\n",in);
            // once lidar_datas receive something new, it will transform it into a ROS cloud type
            pcl::toROSMsg(*cloud, output);
            *accumulated_cloud += *cloud;
            flag = true;
            printf("success!!\n");

}



bool sync_data()  //时间对齐，并进行点云着色
{
    double radarTimestamp;

    if (!lidarBuffer.empty()) { 
        
        radarData = lidarBuffer.front();
        printf("lidar queue size is %d\n",lidarBuffer.size());
            // 获取雷达数据的时间戳
        radarTimestamp = radarData.header.stamp.toSec();
        printf("radar timestamp is %f\n",radarTimestamp);

        my_custom_msgs::CustomMessage odom = odom_buffer.front();
        double odom_time = odom.header.stamp.toSec();
        printf("odom_time is %f\n",odom_time);
                closest_odom = odom;
                odom_received = true;

                for (int i = 0; i < 3; i++) {
                    for (int j = 0; j < 3; j++) {
                    R1(i, j) = closest_odom.rotation_matrix1[i * 3 + j];
                    }
                }

                for (int i = 0; i < 3; i++) {
                    for (int j = 0; j < 3; j++) {
                    R2(i, j) = closest_odom.rotation_matrix1[i * 3 + j];
                    }
                }

                T1 << closest_odom.translation_matrix1[0], closest_odom.translation_matrix1[1], closest_odom.translation_matrix1[2];
                T2 << closest_odom.translation_matrix2[0], closest_odom.translation_matrix2[1], closest_odom.translation_matrix2[2];
                
                printf("odom_received!!!!!");
        
    }else {
        printf("lidar queue is empty\n");
        return false;
    }


    //获取位姿信息的时间戳
    // double odom_time;
    // if ( !odom_buffer.empty() )
    // {
    //     my_custom_msgs::CustomMessage odom = odom_buffer.front();
    //     odom_time = odom.header.stamp.toSec();
    //     printf("odom_time is %f\n",odom_time);

    //     if (odom_time <= radarTimestamp)
    //     {
    //         if (abs(odom_time - radarTimestamp) <0.05)
    //         {
    //             closest_odom = odom;
    //             odom_received = true;

    //             for (int i = 0; i < 3; i++) {
    //                 for (int j = 0; j < 3; j++) {
    //                 R1(i, j) = closest_odom.rotation_matrix1[i * 3 + j];
    //                 }
    //             }

    //             for (int i = 0; i < 3; i++) {
    //                 for (int j = 0; j < 3; j++) {
    //                 R2(i, j) = closest_odom.rotation_matrix1[i * 3 + j];
    //                 }
    //             }

    //             T1 << closest_odom.translation_matrix1[0], closest_odom.translation_matrix1[1], closest_odom.translation_matrix1[2];
    //             T2 << closest_odom.translation_matrix2[0], closest_odom.translation_matrix2[1], closest_odom.translation_matrix2[2];
                
    //             odom_buffer.pop();
    //             printf("odom_received!!!!!");
    //         }
    //         else
    //         {
    //             odom_buffer.pop();
    //         }
            
    //     }
    //     else{

    //         if (!lidarBuffer.empty())
    //         {
    //             lidarBuffer.pop_back(); 
    //         }

    //         printf("odom is not sync!!\n");
    //         return false;
    //     }
    // }
    // else
    // {
    //     printf("odom queue is empty!!\n");
    //     return false;
    // }


    if (!imageBuffer.empty()) {
        printf("imageBuffer size is:%d\n",imageBuffer.size());
        
        the_image = imageBuffer.front();

        double imageTimestamp = image_timebuffer.front();



        if (radarTimestamp < imageTimestamp)
        {
            if (!lidarBuffer.empty())
            {
                odom_buffer.pop();
                lidarBuffer.pop_front(); 
            }
            ROS_WARN("time is not sync!\n");
            return false;
        }
        else
        {
            
            imageBuffer.pop();
            image_timebuffer.pop_front();

            while((radarTimestamp-imageTimestamp)>0.1)
            {

                if(imageBuffer.empty())
                {
                    ROS_WARN("time is not sync222!\n");
                    return false;
                }
                the_image = imageBuffer.front();
                imageTimestamp = image_timebuffer.front();
                      // 使用get()函数将智能指针转换为普通指针
                // sensor_msgs::Image const* imagePtr = imageData.get();
                // cv::Mat imageMat(imagePtr->height, imagePtr->width, CV_8UC3, const_cast<uint8_t*>(imagePtr->data.data()), imagePtr->step);

                imageBuffer.pop(); 
                image_timebuffer.pop_front();

    
            }
            // std::string filename = "/home/gk/mon_livox_ws/src/1.jpg"; // 保存的文件路径和文件名
            // cv::imwrite(filename, the_image);
            // cv::imshow("显示图像", image); // 显示图像
            // cv::waitKey(0); // 等待按键
            make_color_points();
            if (flag)
            {
                lidarBuffer.pop_front(); 
                odom_buffer.pop();
                return true; 
            }
            

        }

        
    }
    else
    {
        printf("image queue is empty!!\n");
        return false;
    }

}








int main(int argc, char **argv)
{
  ros::init(argc, argv, "lidar_camera_subscriber_node");
  ros::NodeHandle n;

  // 订阅Lidar消息（PointCloud2类型）
  ros::Subscriber lidar_sub = n.subscribe("add_points", 1000, lidarCallback);


  ros::Subscriber odom_sub = n.subscribe("custom_message_topic", 1000, odomCallback);


  // 订阅相机消息
  ros::Subscriber camera_sub = n.subscribe("/blackflys/image_raw", 1000, cameraCallback);

  ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("color_lidar", 10);
//   ros::Publisher pub1 = n.advertise<sensor_msgs::PointCloud>("idar", 10);


  ros::Rate loop_rate(10); // frequence 20 Hz

  while (ros::ok())
  {
        ros::spinOnce();
        
        if(sync_data())
        {
            output.header.stamp = ros::Time::now();
            output.header.frame_id = "camera_init"; 
            pub.publish(output); // publish the cloud point to rviz
            loop_rate.sleep();
        }else{
            loop_rate.sleep();
        }

        // radarData = lidarBuffer.front();
        // if (!lidarBuffer.empty()) 
        // { lidarBuffer.pop_back(); }
        // printf("points number is :%d\n",radarData.points.size());
        // radarData.header.frame_id = "camera_init"; 
        // pub1.publish(radarData); // publish the cloud point to rviz
        // loop_rate.sleep();

  }

    pcl::io::savePLYFileBinary("/home/gk/mon_livox_ws/src/diaoche3.ply", *accumulated_cloud);

    ROS_INFO("Accumulated point cloud saved as PLY file");
  

  return 0;
}