#include <iostream>
#include <ros/ros.h>
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
  state_theta = atan((2*(w*z+x*y))/(1-2*(y*y+z*z))) + 1.57326;
      double phi = atan((2*(w*x + y*z))/(1-2*(x*x+y*y)));
      double theta = asin(2*(w*y-z*x))+1.57195;
      double cosi = atan((2*(w*z+x*y))/(1-2*(y*y+z*z)));
      // cout<<"phi: "<<phi<<" theta: "<<theta<<" cosi: "<<cosi<<endl;
  // double cosi = atan((2*(w*z+x*y))/(1-2*(y*y+z*z)));
  // cout<<state_x<<" "<<state_y<<" "<<state_theta<<endl;
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
  // last_point++;
  int i;
  for (i=max(min(int(Path_Global.size()), last_point+5),100); i>last_point; i--) {
    double dis = hypot(Path_Global[i].pts.x - p.x, Path_Global[i].pts.y - p.y);
    if(dis <= 0.1)
      break;
  }
  // last_point = i%Path_Global.size()-1;
  last_point = i+1;
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
  linear.x = (cos(state_theta-1.57326)*diff_x - sin(state_theta-1.57326)*diff_y);
  linear.y = diff_y;
  linear.y = (sin(state_theta-1.57326)*diff_x + cos(state_theta-1.57326)*diff_y);
  linear.z=0;
  geometry_msgs::Vector3 angular;
  angular.x=0;
  angular.y=0;
  angular.z=target_theta - state_theta;
  twist.linear=linear;
  twist.angular=angular;
  cout<<"i: "<<i<<endl;
  cout<<"x: "<<target_x<<" "<<state_x<<" "<<cos(state_theta-1.57326)*diff_x<<" "<<sin(state_theta-1.57326)*diff_y<<endl
      <<"y: "<<target_y<<" "<<state_y<<" "<<sin(state_theta-1.57326)*diff_x<<" "<<cos(state_theta-1.57326)*diff_y<<endl
      <<"t: "<<target_theta<<" "<<state_theta<<endl<<endl;
  // cout<<linear.x<<" "<<linear.y<<endl;

  cmd_pub.publish(twist);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "control");
  ros::NodeHandle nodeHandle("~");
  ros::Rate loop_rate(1);
  load_path();
  odom_sub = nodeHandle.subscribe("/ep/odom", 1, odom_Callback);
  cmd_pub = nodeHandle.advertise<geometry_msgs::Twist>("/cmd_position", 1);
  while (ros::ok())
  {
    control();
    ros::spinOnce();   // Keeps node alive basically
    loop_rate.sleep(); // Sleep for loop_rate
  }
  return 0;
}
