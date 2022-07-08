#include <tf/transform_listener.h>
#include "backward.hpp"
#include "PUTN_classes.h"
#include <pcl/filters/passthrough.h>
#include <std_msgs/Float32MultiArray.h>

using namespace std;
using namespace Eigen;
using namespace PUTN;

namespace backward
{
backward::SignalHandling sh;
}

// ros related
ros::Subscriber pt_sub;
ros::Publisher obs_pub;
ros::Publisher obs_array_pub;
// simulation param from launch file
double resolution, local_x_l, local_x_u, local_y_l, local_y_u, local_z_l, local_z_u;

tf::TransformListener* listener_ptr;

void rcvVelodyneCallBack(const sensor_msgs::PointCloud2& velodyne_points);

void rcvVelodyneCallBack(const sensor_msgs::PointCloud2& velodyne_points)
{
  ROS_INFO("Receive velodyne!");

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(velodyne_points, *cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_PassThrough(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> passthrough;
  passthrough.setInputCloud(cloud);
  passthrough.setFilterFieldName("x");
  passthrough.setFilterLimits(local_x_l, local_x_u);
  passthrough.filter(*cloud_after_PassThrough);

  passthrough.setInputCloud(cloud_after_PassThrough);
  passthrough.setFilterFieldName("y");
  passthrough.setFilterLimits(local_y_l, local_y_u);
  passthrough.filter(*cloud_after_PassThrough);

  passthrough.setInputCloud(cloud_after_PassThrough);
  passthrough.setFilterFieldName("z");
  passthrough.setFilterLimits(local_z_l, local_z_u);
  passthrough.filter(*cloud_after_PassThrough);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filt(new pcl::PointCloud<pcl::PointXYZ>);
  Vector3d lowerbound(local_x_l, local_y_l, local_z_l);
  Vector3d upperbound(local_x_u, local_y_u, local_z_u);
  World local_world = World(resolution);
  local_world.initGridMap(lowerbound, upperbound);
  for (const auto& pt : (*cloud_after_PassThrough).points)
  {
    Vector3d obstacle(pt.x, pt.y, pt.z);
    if (local_world.isFree(obstacle))
    {
      local_world.setObs(obstacle);

      Vector3d obstacle_round = local_world.coordRounding(obstacle);
      pcl::PointXYZ pt_add;
      pt_add.x = obstacle_round(0);
      pt_add.y = obstacle_round(1);
      pt_add.z = obstacle_round(2);
      cloud_filt->points.push_back(pt_add);
    }
  }

  listener_ptr->waitForTransform("/world", "/aft_mapped", ros::Time(0), ros::Duration(2.0));
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tran(new pcl::PointCloud<pcl::PointXYZ>);

  std_msgs::Float32MultiArray obs_array;
  for (const auto& pt : cloud_filt->points)
  {
    geometry_msgs::PointStamped origin_point;
    origin_point.header.frame_id = "/aft_mapped";
    origin_point.header.stamp = ros::Time();
    origin_point.point.x = pt.x;
    origin_point.point.y = pt.y;
    origin_point.point.z = pt.z;

    geometry_msgs::PointStamped trans_point;

    listener_ptr->transformPoint("/world", origin_point, trans_point);

    pcl::PointXYZ _pt;
    if (!(-1.2 < pt.x && pt.x < 0.4 && -0.4 < pt.y && pt.y < 0.4))
    {
      obs_array.data.push_back(trans_point.point.x);
      obs_array.data.push_back(trans_point.point.y);
      obs_array.data.push_back(trans_point.point.z);
    }

    _pt.x = trans_point.point.x;
    _pt.y = trans_point.point.y;
    _pt.z = trans_point.point.z;

    cloud_tran->points.push_back(_pt);
  }

  sensor_msgs::PointCloud2 obs_vis;
  pcl::toROSMsg(*cloud_tran, obs_vis);

  obs_vis.header.frame_id = "/world";
  obs_pub.publish(obs_vis);
  obs_array_pub.publish(obs_array);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "local_obs_node");
  ros::NodeHandle nh("~");

  pt_sub = nh.subscribe("map", 1, rcvVelodyneCallBack);

  obs_pub = nh.advertise<sensor_msgs::PointCloud2>("obs_vis", 1);
  obs_array_pub = nh.advertise<std_msgs::Float32MultiArray>("/obs", 1);

  nh.param("map/resolution", resolution, 0.1);
  nh.param("map/local_x_l", local_x_l, -2.0);
  nh.param("map/local_x_u", local_x_u, 2.0);
  nh.param("map/local_y_l", local_y_l, -2.0);
  nh.param("map/local_y_u", local_y_u, 2.0);
  nh.param("map/local_z_l", local_z_l, -0.3);
  nh.param("map/local_z_u", local_z_u, 0.5);

  tf::TransformListener listener;
  listener_ptr = &listener;

  while (ros::ok())
  {
    timeval start;
    gettimeofday(&start, NULL);
    ros::spinOnce();
    double ms;
    do
    {
      timeval end;
      gettimeofday(&end, NULL);
      ms = 1000 * (end.tv_sec - start.tv_sec) + 0.001 * (end.tv_usec - start.tv_usec);
    } while (ms < 50);
  }
  return 0;
}
