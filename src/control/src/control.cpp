#include <iostream>
#include <ros/ros.h>
#include "control.hpp"

#define PI 3.141592653
using namespace std;
void odom_Callback(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
  nav_msgs::Odometry odom_msg1 = *odom_msg;
  odom.pose.pose = odom_msg1.pose.pose;
  // std::cout << " [" << odom.pose.pose.orientation.w << ", " << odom.pose.pose.orientation.x << ", "
  //           << odom.pose.pose.orientation.y << ", " << odom.pose.pose.orientation.z << "]" << std::endl
  //           << std::endl;
  state_x = odom.pose.pose.position.x;
  state_y = odom.pose.pose.position.y;
  double state_z = odom.pose.pose.position.z;
  double w = odom.pose.pose.orientation.w;
  double x = odom.pose.pose.orientation.x;
  double y = odom.pose.pose.orientation.y;
  double z = odom.pose.pose.orientation.z;
  double phi = atan((2*(w*x + y*z))/(1-2*(x*x+y*y)));
  state_theta = asin(2*(w*y-z*x))+1.57195;
  double cosi = atan((2*(w*z+x*y))/(1-2*(y*y+z*z)));
  cout<<state_x<<" "<<state_y<<" "<<state_theta<<endl;
}

void load_path() {
  freopen("path.txt", "r", stdin);
  double t;
  cin >>t;
  int tot=0;
  while(t != -99999){
    tot += 1;
    if (tot>=1000)
      break;
    TrajectoryPoint tmp;
    tmp.pts.x = t;
    cin>>t;
    tmp.pts.y = t;
    cin>>t;
    tmp.pts.z = t;
    cin>>t;
    Path_Global.push_back(tmp);
  }
  fclose(stdin);
  cout<<"Finish Reading Path"<<endl;
}

void control(){
  geometry_msgs::Point p;
  p.x = state_x;
  p.y = state_y;
  p.z = state_theta;
  last_point++;
  int i;
  for (i=min(int(Path_Global.size()), last_point+10); i>=last_point; i--) {
    double dis = hypot(Path_Global[i].pts.x - p.x, Path_Global[i].pts.y - p.y);
    if(dis <= 0.2)
      break;
  }
  last_point = i;
  double target_x = Path_Global[i].pts.x;
  double target_y = Path_Global[i].pts.y;
  double target_theta = Path_Global[i].pts.z;
  geometry_msgs::Twist twist;
  geometry_msgs::Vector3 linear;
  linear.x=target_x - state_x;
  linear.y=target_y - state_y;
  linear.z=0;
  geometry_msgs::Vector3 angular;
  angular.x=0;
  angular.y=0;
  //直行
  //angular.z=0;
  //转圈
  angular.z=target_theta - state_theta;
  twist.linear=linear;
  twist.angular=angular;

  cmd_pub.publish(twist);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "control");
  ros::NodeHandle nodeHandle("~");
  ros::Rate loop_rate(10);
  load_path();
  // tf_sub = nodeHandle.subscribe("/tf", 10, tf_callback);
  // lidar_sub = nodeHandle.subscribe("/rplidar/scan", 10, lidar_callback);
  odom_sub = nodeHandle.subscribe("/ep/odom", 1, odom_Callback);
  cmd_pub = nodeHandle.advertise<geometry_msgs::Twist>("/cmd_position", 1);
  // state_pub = nodeHandle.advertise<visualization_msgs::MarkerArray>("/state", 1);
  // lidar_pub = nodeHandle.advertise<sensor_msgs::PointCloud2>("/lidar_after", 1);
  // laser_pub = nodeHandle.advertise<sensor_msgs::LaserScan>("/laser", 1);
  while (ros::ok())
  {
    control();
    ros::spinOnce();   // Keeps node alive basically
    loop_rate.sleep(); // Sleep for loop_rate
  }
  return 0;
}