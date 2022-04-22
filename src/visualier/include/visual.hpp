#include <iostream>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>//与tf监听有些区别
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <vector>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "fsd_common_msgs/Map.h"
#include "types.h"
#include <sensor_msgs/LaserScan.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>


ros::Subscriber tf_sub;
ros::Subscriber lidar_sub;

ros::Publisher state_pub;
ros::Publisher lidar_pub;
ros::Publisher laser_pub;

void tf_callback(tf2_msgs::TFMessage msgs);
void visual();
void lidar_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
void LaserScanToPointCloud(sensor_msgs::LaserScan::ConstPtr _laser_scan, pcl::PointCloud<pcl::PointXYZI>& _pointcloud);

Trajectory q;
sensor_msgs::LaserScan scan_msg;
sensor_msgs::PointCloud2 lidar_msg;
double state_rotate;
double state_x, state_y;
ros::Time old;