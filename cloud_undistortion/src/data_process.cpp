#include "cloud_undistortion/data_process.h"

#include <nav_msgs/Odometry.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <cmath>

using Sophus::SE3d;
using Sophus::SO3d;

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudtmp(new pcl::PointCloud<pcl::PointXYZI>());

ImuProcess::ImuProcess() : b_first_frame_(true), last_lidar_(nullptr), last_imu_(nullptr) 
{
    Eigen::Quaterniond q(1, 0, 0, 0);
    Eigen::Vector3d t(0, 0, 0);
    T_i_l = Sophus::SE3d(q, t);
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset() 
{
  ROS_WARN("Reset ImuProcess");

  b_first_frame_ = true;
  last_lidar_ = nullptr;
  last_imu_ = nullptr;

  gyr_int_.Reset(-1, nullptr);

  cur_pcl_in_.reset(new PointCloudXYZI());
  cur_pcl_un_.reset(new PointCloudXYZI());
}

void ImuProcess::IntegrateGyr(const std::vector<sensor_msgs::Imu::ConstPtr> &v_imu) 
{
  /// Reset gyr integrator
  gyr_int_.Reset(last_lidar_->header.stamp.toSec(), last_imu_);
  /// And then integrate all the imu measurements
  for (const auto &imu : v_imu) {
    gyr_int_.Integrate(imu);
  }
  ROS_INFO("integrate rotation angle [x, y, z]: [%.2f, %.2f, %.2f]",
           gyr_int_.GetRot().angleX() * 180.0 / M_PI,
           gyr_int_.GetRot().angleY() * 180.0 / M_PI,
           gyr_int_.GetRot().angleZ() * 180.0 / M_PI);
}

void ImuProcess::UndistortPcl(const PointCloudXYZI::Ptr &pcl_in_out,
                              double dt_be, const Sophus::SE3d &Tbe) 
{
  const Eigen::Vector3d &tbe = Tbe.translation();
  Eigen::Vector3d rso3_be = Tbe.so3().log();
  for (auto &pt : pcl_in_out->points) 
  {
    int ring = int(pt.intensity);
    float dt_bi = pt.intensity - ring;

    if (dt_bi == 0) laserCloudtmp->push_back(pt);
    double ratio_bi = dt_bi / dt_be;
    /// Rotation from i-e
    double ratio_ie = 1 - ratio_bi;

    Eigen::Vector3d rso3_ie = ratio_ie * rso3_be;
    SO3d Rie = SO3d::exp(rso3_ie);

    /// Transform to the 'end' frame, using only the rotation
    /// Note: Compensation direction is INVERSE of Frame's moving direction
    /// So if we want to compensate a point at timestamp-i to the frame-e
    /// P_compensate = R_ei * Pi + t_ei
    Eigen::Vector3d tie = ratio_ie * tbe;
    // Eigen::Vector3d tei = Eigen::Vector3d::Zero();
    Eigen::Vector3d v_pt_i(pt.x, pt.y, pt.z);
    Eigen::Vector3d v_pt_comp_e = Rie.inverse() * (v_pt_i - tie);

    /// Undistorted point
    pt.x = v_pt_comp_e.x();
    pt.y = v_pt_comp_e.y();
    pt.z = v_pt_comp_e.z();
  }
}

void ImuProcess::UndistortPcl(const sensor_msgs::PointCloud2ConstPtr pc_msg, 
                              const Sophus::SE3d &Tbe) 
{
  cur_pcl_un_.reset(new PointCloudXYZI());

  // 创建PointCloud2Iterator对象
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(*pc_msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(*pc_msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(*pc_msg, "z");
  sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(*pc_msg, "intensity");
  sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_range(*pc_msg, "ring");
  sensor_msgs::PointCloud2ConstIterator<double> iter_time(*pc_msg, "timestamp");
  
  float x;
  float y;
  float z;
  float intensity;
  uint16_t range;
  double time;

  //第一点的时间
  double start_time = *iter_time;

  //最后一个点的时间
  double lastest_time = pc_msg->header.stamp.toSec();

  //时间差
  double delta_time = lastest_time - start_time;

  const Eigen::Vector3d &tbe = Tbe.translation();
  Eigen::Vector3d rso3_be = Tbe.so3().log();

  // 遍历点云数据
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity, ++iter_range, ++iter_time)
  {
    x = *iter_x;
    y = *iter_y;
    z = *iter_z;
    intensity = *iter_intensity;
    range = *iter_range;
    time = *iter_time;

    pcl::PointXYZI pt;

    // 设置点的坐标和强度值
    pt.x = x;
    pt.y = y;
    pt.z = z;
    pt.intensity = range;
    
    if(!std::isnan(x) && !std::isnan(y) && !std::isnan(z))
    {
      double ratio_ie = 1 - (time - start_time) / delta_time;
      
      if(ratio_ie > 1.0) ratio_ie = 1.0;
  
      Eigen::Vector3d rso3_ie = ratio_ie * rso3_be;
      SO3d Rie = SO3d::exp(rso3_ie);
  
      Eigen::Vector3d tie = ratio_ie * tbe;
      // Eigen::Vector3d tei = Eigen::Vector3d::Zero();
      Eigen::Vector3d v_pt_i(pt.x, pt.y, pt.z);
      Eigen::Vector3d v_pt_comp_e = Rie.inverse() * (v_pt_i - tie);
  
      /// Undistorted point
      pt.x = v_pt_comp_e.x();
      pt.y = v_pt_comp_e.y();
      pt.z = v_pt_comp_e.z();
    
    }

    // 将点添加到点云中
    cur_pcl_un_->push_back(pt);
  }
}

void ImuProcess::Process(const MeasureGroup &meas) 
{
  ROS_ASSERT(!meas.imu.empty());
  ROS_ASSERT(meas.lidar != nullptr);
  ROS_DEBUG("Process lidar at time: %.4f, %lu imu msgs from %.4f to %.4f",
            meas.lidar->header.stamp.toSec(), meas.imu.size(),
            meas.imu.front()->header.stamp.toSec(),
            meas.imu.back()->header.stamp.toSec());

  auto pcl_in_msg = meas.lidar;

  if (b_first_frame_) {
    /// The very first lidar frame

    /// Reset
    Reset();

    /// Record first lidar, and first useful imu
    last_lidar_ = pcl_in_msg;
    last_imu_ = meas.imu.back();

    ROS_WARN("The very first lidar frame");

    /// Do nothing more, return
    b_first_frame_ = false;
    return;
  }

  /// Integrate all input imu message
  IntegrateGyr(meas.imu);

  /// Compensate lidar points with IMU rotation         
  //// Initial pose from IMU (with only rotation)
  
  SE3d T_l_c(gyr_int_.GetRot(), Eigen::Vector3d::Zero());

  Sophus::SE3d T_l_be = T_i_l.inverse() * T_l_c * T_i_l;

  // pcl::copyPointCloud(*cur_pcl_in_, *cur_pcl_un_);

  clock_t t1,t2;
  t1 = clock();

  // UndistortPcl(cur_pcl_un_, dt_l_c_, T_l_be);
  UndistortPcl(pcl_in_msg, T_l_be);

  t2 = clock();
  printf("time is: %f\n", 1000.0*(t2 - t1) / CLOCKS_PER_SEC);


  {
    static ros::Publisher pub_UndistortPcl =
        nh.advertise<sensor_msgs::PointCloud2>("/rslidar_points", 100);
    sensor_msgs::PointCloud2 pcl_out_msg;
    pcl::toROSMsg(*cur_pcl_un_, pcl_out_msg);
    pcl_out_msg.header = pcl_in_msg->header;
    pcl_out_msg.header.frame_id = "rslidar";
    pub_UndistortPcl.publish(pcl_out_msg);
  }

  /// Record last measurements
  last_lidar_ = pcl_in_msg;
  last_imu_ = meas.imu.back();
//  cur_pcl_in_.reset(new PointCloudXYZI());
//  cur_pcl_un_.reset(new PointCloudXYZI());
}
