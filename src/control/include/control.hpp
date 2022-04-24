#include <iostream>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>//与tf监听有些区别
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
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
#include <nav_msgs/Odometry.h>


ros::Subscriber odom_sub;

ros::Publisher cmd_pub;

void odom_Callback(const nav_msgs::Odometry::ConstPtr& odom_msg);
void load_path();
void control();

nav_msgs::Odometry odom;
Trajectory Path_Global;
double state_x, state_y, state_theta;
int last_point;