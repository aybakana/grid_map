#include <ros/ros.h>
#include "pc2gridmap.hpp"

int main(int argc, char** argv)
{
  // Initialize node and publisher.
  ros::init(argc, argv, "pointcloud_to_gridmap");
  ros::NodeHandle nh;
  grid_map_demos::PointCloudToGridMap pointCloudToGridmap(nh);
  ros::spin();
  return 0;
}