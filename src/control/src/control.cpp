#include <iostream>
#include <ros/ros.h>
#include <cmath>
#include "control.hpp"

#define PI 3.141592653
using namespace std;
void odom_Callback(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
  nav_msgs::Odometry odom_msg1 = *odom_msg;
  odom.pose.pose = odom_msg1.pose.pose;
  state_x = odom.pose.pose.position.x + 2.06371;
  state_y = odom.pose.pose.position.y - 3.74127;
  double state_z = odom.pose.pose.position.z;
  double w = odom.pose.pose.orientation.w;
  double x = odom.pose.pose.orientation.x;
  double y = odom.pose.pose.orientation.y;
  double z = odom.pose.pose.orientation.z;
  // double phi = atan((2*(w*x + y*z))/(1-2*(x*x+y*y)));
  double phi = atan((2*(w*x + y*z))/(1-2*(x*x+y*y)));
  double theta = asin(2*(w*y-z*x))+1.57195;
  double cosi = atan((2*(w*z+x*y))/(1-2*(y*y+z*z)));
  // state_theta = atan((2*(w*z+x*y))/(1-2*(y*y+z*z))) + 1.57326;
  state_theta = atan2(2*(w*z+x*y), 1-2*(y*y+z*z)) + 1.57326;
      // cout<<"phi: "<<phi<<" theta: "<<theta<<" cosi: "<<cosi<<endl;
  // double cosi = atan((2*(w*z+x*y))/(1-2*(y*y+z*z)));
  // cout<<state_x<<" "<<state_y<<" "<<state_theta<<endl;
  // if(fabs(state_theta - state_theta_old)>1.0) {
  //   state_theta = state_theta + 3.141592653;
  // }
  while(state_theta <= 0)
    state_theta += 2*3.141592653;
  state_theta_old = state_theta;
  cout<<"theta: "<<state_theta<<endl;
  if(fabs(odom_msg1.twist.twist.linear.x)>0.001 || 
      fabs(odom_msg1.twist.twist.linear.y)>0.001 || 
      fabs(odom_msg1.twist.twist.angular.z)>0.001)
    is_moving = true;
  else is_moving = false;
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
    while (t > 2*3.141592653)
      t -= 2*3.141592653;
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
  // last_point++;
  int i;
  // for (i=max(min(int(Path_Global.size()), last_point+5),1); i>last_point; i--) {
  //   double dis = hypot(Path_Global[i].pts.x - p.x, Path_Global[i].pts.y - p.y);
  //   if(dis <= 0.1)
  //     break;
  // }
  i = last_point + 1;
  // last_point = i%Path_Global.size()-1;
  last_point = i;
  double target_x = Path_Global[i].pts.x;
  double target_y = Path_Global[i].pts.y;
  double target_theta = Path_Global[i].pts.z;
  geometry_msgs::Twist twist;
  geometry_msgs::Vector3 linear;
  double diff_x=target_x - state_x;
  double diff_y=target_y - state_y;
  // linear.x = -(cos(diff_x)-sin(diff_y));
  // linear.y = -(sin(diff_x)+cos(diff_y));
  linear.x = diff_x;
  linear.x = (cos(state_theta-1.57326)*diff_x + sin(state_theta-1.57326)*diff_y);
  linear.y = diff_y;
  linear.y = (-sin(state_theta-1.57326)*diff_x + cos(state_theta-1.57326)*diff_y);
  linear.z=0;
  geometry_msgs::Vector3 angular;
  angular.x=0;
  angular.y=0;
  angular.z=target_theta - state_theta;
  cout<<"angular: "<<angular.z<<endl;
  if(fabs(angular.z) > 0.1) {
    linear.x = 0;
    linear.y = 0;
    last_point --;
  }
  else if(fabs(linear.x) >0.1 || fabs(linear.y) > 0.1){ 
    angular.z = 0;
    // linear.y = 0;
    last_point --;
  }
  // else if(fabs(linear.y) > 0.1) {
  //   angular.z = 0;
  //   linear.x = 0;
  //   last_point --;
  // }
  else angular.z = 0;
  cout<<"angular: "<<angular.z<<endl;
  twist.linear=linear;
  twist.angular=angular;
  cout<<"i: "<<i<<endl;
  cout<<"x: "<<target_x<<" "<<state_x<<" "<<cos(state_theta-1.57326)*diff_x<<" "<<sin(state_theta-1.57326)*diff_y<<endl
      <<"y: "<<target_y<<" "<<state_y<<" "<<sin(state_theta-1.57326)*diff_x<<" "<<cos(state_theta-1.57326)*diff_y<<endl
      <<"t: "<<target_theta<<" "<<state_theta<<endl<<endl;
  // cout<<linear.x<<" "<<linear.y<<endl;
  if (!is_moving)
    cmd_pub.publish(twist);
  // system("sleep")
  // Sleep(100);
  sleep(0.6);
}


void imu_Callback(const sensor_msgs::Imu::ConstPtr& imu_msg){
  // std::cout<<imu_ms
  double rota_z = imu_msg->angular_velocity.z;
  double linear_x = imu_msg->linear_acceleration.x;
  double linear_y = imu_msg->linear_acceleration.y;
  // if (fabs(rota_z)>0.01 || fabs(linear_x) >0.001 || fabs(linear_y) >0.001)
  //   is_moving = true;
  // else is_moving = false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "control");
  ros::NodeHandle nodeHandle("~");
  ros::Rate loop_rate(10);
  load_path();
  odom_sub = nodeHandle.subscribe("/ep/odom", 1, odom_Callback);
  imu_sub = nodeHandle.subscribe("/imu/data_raw", 1, imu_Callback);
  cmd_pub = nodeHandle.advertise<geometry_msgs::Twist>("/cmd_position", 1);
  while (ros::ok())
  {
    control();
    ros::spinOnce();   // Keeps node alive basically
    loop_rate.sleep(); // Sleep for loop_rate
  }
  return 0;
}