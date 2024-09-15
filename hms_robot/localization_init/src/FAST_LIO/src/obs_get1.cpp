#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <Python.h>
#include <so3_math.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include "IMU_Processing.hpp"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <livox_ros_driver2/CustomMsg.h>
#include "preprocess.h"
#include <ikd-Tree/ikd_Tree.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nlohmann/json.hpp>

#include "std_msgs/Bool.h"
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>

#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)

/*** Time Log Variables ***/
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
double match_time = 0, solve_time = 0, solve_const_H_time = 0;
int    kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
bool   runtime_pos_log = false, pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true, path_en = true;
/**************************/

float res_last[100000] = {0.0};
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;
double time_diff_lidar_to_imu = 0.0;

mutex mtx_buffer;
condition_variable sig_buffer;

string root_dir = ROOT_DIR;
string map_file_path, lid_topic, imu_topic , json_path;
string map_input_path;

double res_mean_last = 0.05, total_residual = 0.0;
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
int    effct_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
int    iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, pcd_save_interval = -1, pcd_index = 0;
bool   point_selected_surf[100000] = {0};
bool   lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
bool   scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;

vector<vector<int>>  pointSearchInd_surf; 
vector<BoxPointType> cub_needrm;
vector<PointVector>  Nearest_Points; 
vector<bool> add_flag;
vector<double>       extrinT(3, 0.0);
vector<double>       extrinR(9, 0.0);
deque<double>                     time_buffer;

deque<PointCloudXYZI::Ptr>        lidar_buffer;

deque<pcl::PointCloud<pcl::PointXYZI>::Ptr>    obstacle_buffer;

deque<sensor_msgs::Imu::ConstPtr> imu_buffer;

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr _featsArray;

PointCloudXYZI::Ptr relocal_map(new PointCloudXYZI());


pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

KD_TREE<PointType> ikdtree;

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;
V3D position_last(Zero3d);
V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);

/*** EKF inputs and output ***/
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom state_point;
vect3 pos_lid;

nav_msgs::Path path;
nav_msgs::Odometry odomAftMapped;
nav_msgs::Odometry tran_odom;
geometry_msgs::Quaternion geoQuat;
geometry_msgs::PoseStamped msg_body_pose;

shared_ptr<Preprocess> p_pre(new Preprocess());
shared_ptr<ImuProcess> p_imu(new ImuProcess());


// 定义一个结构体来存储目标点
struct Point {
    double x;
    double y;
    std::string tag;  // 新增tag字段，用于区分普通航迹点和任务点
};



void SigHandle(int sig)
{
    flg_exit = true;
    ROS_WARN("catch sig %d", sig);
    sig_buffer.notify_all();
}



//////

double x_g , x_g_r , y_g_r , z_g_r;
double y_g ;
double z_g ;
int grav_cnt = 0;
Eigen::Quaternionf rotation_quaternion_grav;

float rotation_angle_x;
float rotation_angle_y;
Eigen::Quaternionf get_gravity_align_rotation(Eigen::Vector3f grav_world) 
{
    if (grav_world.z() < 0) {
        grav_world = -grav_world;
    }
    //grav_world.normalize();
    float x_grav = grav_world(0);
    Eigen::Vector3f x_gravity(x_grav, 0.0f, 0.0f);
    float y_grav = grav_world(1);
    Eigen::Vector3f y_gravity(0.0f, y_grav, 0.0f);

    Eigen::Vector3f grav_rm_x = grav_world - x_gravity;
    Eigen::Vector3f grav_rm_y = grav_world - y_gravity;
    Eigen::Vector3f z_axis(0, 0, 1);
    Eigen::Vector3f x_axis(1, 0, 0);
    Eigen::Vector3f y_axis(0, 1, 0);


    float cos_theta_x = grav_rm_x.dot(z_axis) / (grav_rm_x.norm() * z_axis.norm());
    float sin_theta_x = std::sqrt(1.0f - cos_theta_x*cos_theta_x);
    rotation_angle_x = std::atan2(sin_theta_x, cos_theta_x);
    //  std::cout << "Rotation angle:\n"
    //           << rotation_angle_x << std::endl;

    float cos_theta_y = grav_rm_y.dot(z_axis) / (grav_rm_y.norm() * z_axis.norm());
    float sin_theta_y = std::sqrt(1.0f - cos_theta_y*cos_theta_y);
    rotation_angle_y = std::atan2(sin_theta_y, cos_theta_y);
    //  std::cout << "Rotation angle:\n"
    //           << rotation_angle_y << std::endl;

    Eigen::Quaternionf rotation_quaternion_x , rotation_quaternion_y,rotation_quaternion;
    rotation_quaternion_x = Eigen::AngleAxisf(rotation_angle_x, x_axis);
    rotation_quaternion_y = Eigen::AngleAxisf(rotation_angle_y, y_axis);
    rotation_quaternion = rotation_quaternion_y*rotation_quaternion_x;
    return rotation_quaternion;

}


bool acc_grav()
{
    if (grav_cnt == 11)
    {
        // std::cout << "x_g_r: " << x_g_r << " y_g_r: " << y_g_r << "z_g_r:" <<z_g_r<<std::endl;
        Eigen::Vector3f grav_world(x_g_r, y_g_r, z_g_r);
        rotation_quaternion_grav = get_gravity_align_rotation(grav_world);
        return true;
    }else
    {
        return false;
    }
}

void pointstran_grav(PointType const * const pi, PointType * const po)
{
   
    Eigen::Vector3f p_body_lidar(pi->x, pi->y, pi->z);
    Eigen::Vector3f p_lidar_grav(rotation_quaternion_grav*p_body_lidar );

    po->x = p_lidar_grav(0);
    po->y = p_lidar_grav(1);
    po->z = p_lidar_grav(2);
    po->intensity = pi->intensity;

}


////角度计
void pointstran_grav1(PointType const * const pi, pcl::PointXYZI * const po)
{
   
    Eigen::Vector3f p_body_lidar(pi->x, pi->y, pi->z);
    Eigen::Vector3f p_lidar_grav(rotation_quaternion_grav*p_body_lidar );

    po->x = p_lidar_grav(0);
    po->y = p_lidar_grav(1);
    po->z = p_lidar_grav(2);
    po->intensity = pi->intensity;

}

double calculateAngleWithXAxis(PointType const * const pi) {
    // 提取点的x, y, z坐标
    double x = pi->x;
    double y = pi->y;
    double z = pi->z;

    // 使用atan2计算与x轴正方向向量的角度
    double angle_rad = atan2(y, x); // atan2返回的是弧度值，范围是[-π, π]
    double angle_deg = angle_rad/3.1415*180; // 转换为度

    // 根据atan2的返回值调整角度范围，确保它是0到360度之间
    // if (angle_deg < 0) {
    //     angle_deg += 360.0;
    // }

    return angle_deg;
}

double calculateMagnitude(PointType const * const pi) {
    if (pi == nullptr) {
        throw std::invalid_argument("Point pointer is null");
    }
    
    // 计算模长：sqrt(x^2 + y^2 + z^2)
    double magnitude = std::sqrt(pi->x * pi->x + pi->y * pi->y + pi->z * pi->z);
    return magnitude;
}

////检测是否在矩形内/////

// 定义一个结构体来表示二维点
struct RectanglePoint {
    double x;
    double y;
};

// 函数用于检测点是否在矩形内
// 矩形由两个对角顶点topLeft和bottomRight定义
bool isPointInRectangle(const PointType& point, 
                         const RectanglePoint& topLeft, 
                         const RectanglePoint& bottomRight) {
    // 检查点的x坐标是否在矩形的左边和右边之间
    bool withinX = point.x >= topLeft.x && point.x <= bottomRight.x;
    // 检查点的y坐标是否在矩形的顶部和底部之间
    bool withinY = point.y >= bottomRight.y && point.y <= topLeft.y;

    // 如果点的x和y坐标都在矩形的边界内，则点在矩形内
    return withinX && withinY;
}

//////
void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg) 
{
    mtx_buffer.lock();
    scan_count ++;
    double preprocess_start_time = omp_get_wtime();
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(msg->header.stamp.toSec());
    last_timestamp_lidar = msg->header.stamp.toSec();
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

double timediff_lidar_wrt_imu = 0.0;
bool   timediff_set_flg = false;

void livox_pcl_cbk(const livox_ros_driver2::CustomMsg::ConstPtr &msg) 
{
    if (acc_grav())
    {
        mtx_buffer.lock();
        double preprocess_start_time = omp_get_wtime();
        scan_count ++;
        if (msg->header.stamp.toSec() < last_timestamp_lidar)
        {
            ROS_ERROR("lidar loop back, clear buffer");
            lidar_buffer.clear();
        }
        last_timestamp_lidar = msg->header.stamp.toSec();
        
        if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty() )
        {
            printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n",last_timestamp_imu, last_timestamp_lidar);
        }

        if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
        {
            timediff_set_flg = true;
            timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
            printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
        }


        PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
        p_pre->process(msg, ptr);
        int size_init_cloud = ptr->size();
        int icout = 0;
        int icout1 = 0;
        PointCloudXYZI::Ptr laserCloud_tran_grav1(new PointCloudXYZI(size_init_cloud, 1));
        pcl::PointCloud<pcl::PointXYZI>::Ptr obstacle_cloud(new pcl::PointCloud<pcl::PointXYZI>(size_init_cloud, 1));        
        
        RectanglePoint topLeft = {-0.5, 0.5};
        RectanglePoint bottomRight = {2.0, -0.5};
        for (size_t i = 0; i < size_init_cloud; i++)
        {
            double angle = calculateAngleWithXAxis(&ptr->points[i]);
            double distance = calculateMagnitude(&ptr->points[i]);
            
            if (abs(angle)>150 && abs(angle)<=180 && distance < 1)
            {
                continue;
            }
            if (distance < 0.2)
            {
                continue;
            }
            pointstran_grav(&ptr->points[i], \
                            &laserCloud_tran_grav1->points[icout]);
            
            if (isPointInRectangle(laserCloud_tran_grav1->points[icout] , topLeft, bottomRight) && laserCloud_tran_grav1->points[icout].z>0)
            {
                pointstran_grav1(&ptr->points[i], \
                            &obstacle_cloud->points[icout1]);
                icout1++;
            }
            icout++;
        }
        obstacle_cloud->points.resize(icout1);
        laserCloud_tran_grav1->points.resize(icout);

        obstacle_buffer.push_back(obstacle_cloud);
        
        if (obstacle_buffer.size()>2)
        {
            obstacle_buffer.pop_front();
        }
        

        lidar_buffer.push_back(laserCloud_tran_grav1);
        time_buffer.push_back(last_timestamp_lidar);
        
        s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
        mtx_buffer.unlock();
        sig_buffer.notify_all();
    }
    
    
}


void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in) 
{
    if (grav_cnt <10)
    {
        x_g += msg_in->linear_acceleration.x;
        y_g += msg_in->linear_acceleration.y;
        z_g += msg_in->linear_acceleration.z;
        grav_cnt++;
    }
    if (grav_cnt ==10)
    {
        x_g_r = x_g/(grav_cnt);
        y_g_r = y_g/(grav_cnt);
        z_g_r = z_g/(grav_cnt);
        grav_cnt++;
    }
   


    if (acc_grav())
    {
         publish_count ++;
        // cout<<"IMU got at: "<<msg_in->header.stamp.toSec()<<endl;
        sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

        msg->header.stamp = ros::Time().fromSec(msg_in->header.stamp.toSec() - time_diff_lidar_to_imu);
        if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
        {
            msg->header.stamp = \
            ros::Time().fromSec(timediff_lidar_wrt_imu + msg_in->header.stamp.toSec());
        }

        double timestamp = msg->header.stamp.toSec();

        mtx_buffer.lock();

        if (timestamp < last_timestamp_imu)
        {
            ROS_WARN("imu loop back, clear buffer");
            imu_buffer.clear();
        }

        last_timestamp_imu = timestamp;
        
        imu_buffer.push_back(msg);
        mtx_buffer.unlock();
        sig_buffer.notify_all();
    }
    

}


////////

bool stop_flag = false;
void publish_stop(const ros::Publisher & bool_pub)
{
    std_msgs::Bool msg;
    msg.data = stop_flag; // 或者根据需要设置为false
    bool_pub.publish(msg);

}


// 自定义比较函数，按簇大小降序排序
bool compareClusters(const pcl::PointIndices& a, const pcl::PointIndices& b) {
    return a.indices.size() > b.indices.size();
}

Eigen::Vector2f calculateXYCentroid(const pcl::PointCloud<pcl::PointXYZI>& cloud) {
    Eigen::Vector2f centroid;
    centroid.setZero(); // 初始化xy平面上的质心为零向量

    // 遍历点云，累加所有点的x和y坐标
    for (const pcl::PointXYZI& point : cloud) {
        centroid[0] += point.x; // 累加x坐标
        centroid[1] += point.y; // 累加y坐标
    }

    // 将累加的坐标除以点的数量，得到xy平面上的中心坐标
    float size = static_cast<float>(cloud.size());
    centroid[0] = centroid[0]/size;
    centroid[1] = centroid[1]/size;

    return centroid;
}
/////点云分割///
double obstacle_distance;
pcl::PointCloud<pcl::PointXYZI>::Ptr seg_A_C (new pcl::PointCloud<pcl::PointXYZI>);
void cloud_seg(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
        
        ///聚类操作，在次去除噪点/////
        // 创建分割器对象
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        tree->setInputCloud(cloud);
        std::vector<pcl::PointIndices> cluster_indices;  
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(0.1); // 设置聚类的容差
        ec.setMinClusterSize(10); // 设置最小聚类大小
        ec.setMaxClusterSize(250000); // 设置最大聚类大小
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);


        // 从小到大排列
        //std::sort(cluster_indices.begin(), cluster_indices.end(), compareClusters);
        if (cluster_indices.size() ==0)
        {
            std::cout << "cluster fail!!"<<std::endl;
            stop_flag = false;
            return;
        }
        
        // 根据最大簇的索引去除小的簇
        // 提取最大的簇
        int size_cout = 0;
        for (size_t i = 0; i < cluster_indices.size(); i++)
        {
            for (std::vector<int>::const_iterator pit = cluster_indices[i].indices.begin(); pit != cluster_indices[i].indices.end(); ++pit)  
                seg_A_C->points.push_back(cloud->points[*pit]); 
            
            if (seg_A_C->points.size()<5)
            {
                seg_A_C->clear();
                continue;
            }

            seg_A_C->width = seg_A_C->points.size();  
            seg_A_C->height = 1;  
            seg_A_C->is_dense = true; 

            Eigen::Vector2f center;
            center = calculateXYCentroid(*seg_A_C);
 
            std::stringstream ss;
            // 使用<<操作符将整数写入stringstream
            ss << size_cout;
            // 使用.str()方法获取stringstream中的字符串
            std::string str = ss.str();
            pcl::io::savePCDFileASCII(root_dir+"pcd/"+str+".pcd", *seg_A_C);
            size_cout++;

            
            double distance_obtacle =std::sqrt(center[0]*center[0] + center[1]*center[1]);
            if (distance_obtacle < obstacle_distance)
            {
                std::cout << "obstacle distance is " << distance_obtacle <<std::endl;
                std::cout << "obstacle size is " << seg_A_C->points.size() <<std::endl;
                std::cout << "cluster size is " << cluster_indices.size() <<std::endl;
                stop_flag = true;
                return;
            }
            seg_A_C->clear();
        }
        stop_flag = false;
        
}
 


double lidar_mean_scantime = 0.0;
int    scan_num = 0;
bool sync_packages(MeasureGroup &meas)
{
    if (lidar_buffer.empty() || imu_buffer.empty()) {
        return false;
    }

    /*** push a lidar scan ***/
    if(!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();
        if (meas.lidar->points.size() <= 1) // time too little
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
            ROS_WARN("Too few input point cloud!\n");
        }
        else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        }
        else
        {
            scan_num ++;
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
            lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
        }

        meas.lidar_end_time = lidar_end_time;

        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time)
    {
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = imu_buffer.front()->header.stamp.toSec();
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    {
        imu_time = imu_buffer.front()->header.stamp.toSec();
        if(imu_time > lidar_end_time) break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}



void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    double match_start = omp_get_wtime();
    laserCloudOri->clear(); 
    corr_normvect->clear(); 
    total_residual = 0.0; 

    /** closest surface search and residual computation **/
    #ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for
    #endif
    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body  = feats_down_body->points[i]; 
        PointType &point_world = feats_down_world->points[i]; 

        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

        auto &points_near = Nearest_Points[i];

        if (ekfom_data.converge)
        {
            /** Find the closest surfaces in the map **/
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
        }

        if (!point_selected_surf[i]) continue;

        VF(4) pabcd;
        point_selected_surf[i] = false;
        if (esti_plane(pabcd, points_near, 0.1f))
        {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

            if (s > 0.9)
            {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
                res_last[i] = abs(pd2);
            }
        }
    }
    
    effct_feat_num = 0;

    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            total_residual += res_last[i];
            effct_feat_num ++;
        }
    }

    if (effct_feat_num < 1)
    {
        ekfom_data.valid = false;
        ROS_WARN("No Effective Points! \n");
        return;
    }

    res_mean_last = total_residual / effct_feat_num;
    match_time  += omp_get_wtime() - match_start;
    double solve_start_  = omp_get_wtime();
    
    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); //23
    ekfom_data.h.resize(effct_feat_num);

    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p  = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat<<SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() *norm_vec);
        V3D A(point_crossmat * C);
        if (extrinsic_est_en)
        {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); //s.rot.conjugate()*norm_vec);
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity;
    }
    solve_time += omp_get_wtime() - solve_start_;
}





int main(int argc, char** argv)
{
    ros::init(argc, argv, "obs_get");
    ros::NodeHandle nh;
    tf2_ros::TransformBroadcaster broadcaster;
    
    nh.param<bool>("publish/path_en",path_en, true);
    nh.param<bool>("publish/scan_publish_en",scan_pub_en, true);
    nh.param<bool>("publish/dense_publish_en",dense_pub_en, true);
    nh.param<bool>("publish/scan_bodyframe_pub_en",scan_body_pub_en, true);
    nh.param<int>("max_iteration",NUM_MAX_ITERATIONS,4);
    nh.param<string>("map_file_path",map_file_path,"");
    
    nh.param<string>("pose_transform/json_path",json_path,"");    
    
    nh.param<string>("common/lid_topic",lid_topic,"/livox/lidar");
    nh.param<string>("common/imu_topic", imu_topic,"/livox/imu");
    nh.param<bool>("common/time_sync_en", time_sync_en, false);
    nh.param<double>("common/time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);
    nh.param<double>("filter_size_corner",filter_size_corner_min,0.5);
    nh.param<double>("filter_size_surf",filter_size_surf_min,0.5);
    nh.param<double>("filter_size_map",filter_size_map_min,0.5);
    nh.param<double>("cube_side_length",cube_len,200);
    nh.param<float>("mapping/det_range",DET_RANGE,300.f);
    nh.param<double>("mapping/fov_degree",fov_deg,180);
    nh.param<double>("mapping/gyr_cov",gyr_cov,0.1);
    nh.param<double>("mapping/acc_cov",acc_cov,0.1);
    nh.param<double>("mapping/b_gyr_cov",b_gyr_cov,0.0001);
    nh.param<double>("mapping/b_acc_cov",b_acc_cov,0.0001);
    nh.param<double>("preprocess/blind", p_pre->blind, 0.01);
    nh.param<int>("preprocess/lidar_type", p_pre->lidar_type, AVIA);
    nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 16);
    nh.param<int>("preprocess/timestamp_unit", p_pre->time_unit, US);
    nh.param<int>("preprocess/scan_rate", p_pre->SCAN_RATE, 10);
    nh.param<int>("point_filter_num", p_pre->point_filter_num, 2);
    nh.param<bool>("feature_extract_enable", p_pre->feature_enabled, false);
    nh.param<bool>("runtime_pos_log_enable", runtime_pos_log, 0);
    nh.param<bool>("mapping/extrinsic_est_en", extrinsic_est_en, true);
    nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en, false);
    nh.param<int>("pcd_save/interval", pcd_save_interval, -1);
    nh.param<vector<double>>("mapping/extrinsic_T", extrinT, vector<double>());
    nh.param<vector<double>>("mapping/extrinsic_R", extrinR, vector<double>());

    nh.param<double>("pose_transform/obstacle_distance", obstacle_distance, 1);


    
    cout<<"p_pre->lidar_type "<<p_pre->lidar_type<<endl;
    
    path.header.stamp    = ros::Time::now();
    path.header.frame_id ="camera_init";

    /*** variables definition ***/
    int effect_feat_num = 0, frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
    bool flg_EKF_converged, EKF_stop_flg = 0;
    
    FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
    HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

    _featsArray.reset(new PointCloudXYZI());

    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));

    Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
    p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
    p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
    p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

    double epsi[23] = {0.001};
    fill(epsi, epsi+23, 0.001);
    kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);

    /*** debug record ***/
    FILE *fp;
    string pos_log_dir = root_dir + "/Log/pos_log.txt";
    fp = fopen(pos_log_dir.c_str(),"w");

    ofstream fout_pre, fout_out, fout_dbg;
    fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"),ios::out);
    fout_out.open(DEBUG_FILE_DIR("mat_out.txt"),ios::out);
    fout_dbg.open(DEBUG_FILE_DIR("dbg.txt"),ios::out);
    if (fout_pre && fout_out)
        cout << "~~~~"<<ROOT_DIR<<" file opened" << endl;
    else
        cout << "~~~~"<<ROOT_DIR<<" doesn't exist" << endl;


        /*** ROS subscribe initialization ***/
    ros::Subscriber sub_pcl = p_pre->lidar_type == AVIA ? \
        nh.subscribe(lid_topic, 200000, livox_pcl_cbk) : \
        nh.subscribe(lid_topic, 200000, standard_pcl_cbk);
    ros::Subscriber sub_imu = nh.subscribe(imu_topic, 200000, imu_cbk);

    ros::Publisher bool_pub = nh.advertise<std_msgs::Bool>("stop_signl", 10);



//------------------------------------------------------------------------------------------------------
    signal(SIGINT, SigHandle);
    ros::Rate rate(5000);
    bool status = ros::ok();
    while (status)
    {

        ros::spinOnce();
        if(sync_packages(Measures)) 
        {


            p_imu->Process(Measures, kf, feats_undistort);
            state_point = kf.get_x();

            pcl::PointCloud<pcl::PointXYZI>::Ptr obstacle_(new pcl::PointCloud<pcl::PointXYZI>);
            if(obstacle_buffer.size()>1)
            {
                for(int i=0;i<obstacle_buffer.size();i++)
                {
                    *obstacle_ += *obstacle_buffer.front();
                    obstacle_buffer.pop_front();
                }
                if(obstacle_->size()<20)
                {
                    std::cout << "no obstacle!!!"<<std::endl;
                    stop_flag = false;
                }else{
                    pcl::io::savePCDFileASCII(root_dir+"pcd/near.pcd", *obstacle_);
                    cloud_seg(obstacle_);
                }


            }

            publish_stop(bool_pub);

        }

        status = ros::ok();
        rate.sleep();
    }




    return 0;
}
