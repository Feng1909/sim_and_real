#ifndef TYPE_H
#define TYPE_H

#include <iostream>
#include "std_msgs/Float64MultiArray.h"

// custom messages
#include "fsd_common_msgs/CarState.h"
#include "fsd_common_msgs/ControlCommand.h"
#include "fsd_common_msgs/Map.h"
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>

// STL
#include <cmath>
#include <vector>

struct TrajectoryPoint {
    geometry_msgs::Point pts;
    double yaw;
    double curvature;
    double velocity;
    double r;
    double acc;
};

typedef std::vector<TrajectoryPoint> Trajectory;

#endif