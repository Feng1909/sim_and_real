#include <iostream>
#include <ros/ros.h>
#include "visual.hpp"

#define PI 3.141592653
using namespace std;

void visual(){
  visualization_msgs::MarkerArray visual;
  visual.markers.clear();
  std::vector<float> color_tmp;
  if (q.size() == 0)
    return;
  for (size_t i = 0; i < q.size() - 1; i++) {
    visualization_msgs::Marker tmp;
    tmp.header.frame_id = "map";
    tmp.header.stamp = ros::Time::now();
    tmp.ns = "line_" + std::to_string(i);
    tmp.action = visualization_msgs::Marker::ADD;
    tmp.type = visualization_msgs::Marker::LINE_STRIP;
    tmp.scale.x = 0.05;

    std::vector<float> color = {0, 1, 0};
    color_tmp = color;
    tmp.color.r = color_tmp[0];
    tmp.color.g = color_tmp[1];
    tmp.color.b = color_tmp[2];
    tmp.color.a = 1.0;
    geometry_msgs::Point p;
    p.x = q[i].pts.x;
    p.y = q[i].pts.y;
    tmp.points.push_back(p);
    p.x = q[i + 1].pts.x;
    p.y = q[i + 1].pts.y;
    tmp.points.push_back(p);
    visual.markers.push_back(tmp);
  }
  state_pub.publish(visual);
}

void tf_callback(tf2_msgs::TFMessage msgs){
    if (msgs.transforms[0].child_frame_id == "tracker_LHR_A655F116") {
      // if msgs.header.timestamp
      if (msgs.transforms[0].header.stamp <old) {
        freopen("path.txt", "w", stdout);
        for(int i=0; i<q.size(); i++){
          cout<<q[i].pts.x<<" "<<q[i].pts.y<<" "<<q[i].pts.z<<endl;
        }
        cout<<-99999;
        fclose(stdout);
        cout<<"finish save path"<<endl;
        q.clear();
      }
      old = msgs.transforms[0].header.stamp;
      TrajectoryPoint s;
      s.pts.x = -msgs.transforms[0].transform.translation.x + 4;
      s.pts.y = msgs.transforms[0].transform.translation.z;
      // cout<<msgs.transforms[0].header.stamp<<endl;
      // cout<<msgs.transforms[0].transform.rotation.x<<endl
      //   <<msgs.transforms[0].transform.rotation.y<<endl
      //   <<msgs.transforms[0].transform.rotation.z<<endl<<endl;
      double w = msgs.transforms[0].transform.rotation.w;
      double x = msgs.transforms[0].transform.rotation.x;
      double y = msgs.transforms[0].transform.rotation.y;
      double z = msgs.transforms[0].transform.rotation.z;
      double phi = atan((2*(w*x + y*z))/(1-2*(x*x+y*y)));
      double theta = asin(2*(w*y-z*x))+PI;
      double cosi = atan((2*(w*z+x*y))/(1-2*(y*y+z*z)));
      // cout<<"phi: "<<phi<<" theta: "<<theta<<" cosi: "<<cosi<<endl;
      state_rotate = theta;
      state_x = msgs.transforms[0].transform.translation.x + 4;
      state_y = msgs.transforms[0].transform.translation.y;
      // cout<<asin(s.pts.x/hypot(s.pts.x, s.pts.y))<<" "<<msgs.transforms[0].transform.rotation.y<<endl;
      cout<<s.pts.x<<" "<<s.pts.y<< " "<<theta<<endl;
      s.pts.z = theta;
      q.push_back(s);
    }
    visual();
}
void lidar_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  // msg->header.frame_id = "map";
  sensor_msgs::LaserScan t;
  t.ranges = msg->ranges;
  t.header = msg->header;
  t.header.frame_id = "map";
  t.angle_min = -PI/2;
  t.angle_max = PI/2;
  t.angle_increment = msg->angle_increment;
  t.time_increment = msg->time_increment;
  t.range_min = msg->range_min;
  t.range_max = msg->range_max;
  t.scan_time=msg->scan_time;
  
  t.intensities=msg->intensities;
  for (int i=0; i<msg->ranges.size(); i++) {
    t.ranges[i] = t.ranges[i]*0.8;
  }
  laser_pub.publish(t);
  pcl::PointCloud<pcl::PointXYZI>t_cloud;
  LaserScanToPointCloud(msg, t_cloud);
  pcl::toROSMsg(t_cloud, lidar_msg);
  lidar_msg.header.frame_id="map";
  // lidar_pub.publish(lidar_msg);
}
void LaserScanToPointCloud(sensor_msgs::LaserScan::ConstPtr _laser_scan, pcl::PointCloud<pcl::PointXYZI>& _pointcloud)
{
  _pointcloud.clear();
  pcl::PointXYZI newPoint;
  newPoint.z = 0.0;
  double newPointAngle;

  int beamNum = _laser_scan->ranges.size();
  for (int i = 0; i < beamNum; i++)
  {
      newPointAngle = _laser_scan->angle_min + _laser_scan->angle_increment * i - state_rotate;
      newPoint.x = (_laser_scan->ranges[i] - state_x) * cos(newPointAngle) + state_x;
      newPoint.y = (_laser_scan->ranges[i] - state_y) * sin(newPointAngle) + state_y;
      newPoint.intensity = _laser_scan->intensities[i];
      _pointcloud.push_back(newPoint);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "visual");
  ros::NodeHandle nodeHandle("~");
  ros::Rate loop_rate(10);
  tf_sub = nodeHandle.subscribe("/tf", 10, tf_callback);
  lidar_sub = nodeHandle.subscribe("/rplidar/scan", 10, lidar_callback);
  state_pub = nodeHandle.advertise<visualization_msgs::MarkerArray>("/state", 1);
  lidar_pub = nodeHandle.advertise<sensor_msgs::PointCloud2>("/lidar_after", 1);
  laser_pub = nodeHandle.advertise<sensor_msgs::LaserScan>("/laser", 1);
  while (ros::ok()) {
    ros::spinOnce();                // Keeps node alive basically
    loop_rate.sleep();              // Sleep for loop_rate
  }
  return 0;
}