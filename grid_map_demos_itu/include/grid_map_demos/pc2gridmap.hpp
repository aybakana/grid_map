#include <ros/ros.h>

#include "sensor_msgs/PointCloud2.h"

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/io.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/transforms.h"

#include <string>



namespace grid_map_demos {
class PointCloudToGridMap
{
public:

	PointCloudToGridMap(ros::NodeHandle& nodeHandle);
    bool readParameters();
    bool initializeMap();
    void pointCloudCallback(const sensor_msgs::PointCloud2& msg);
    
virtual ~PointCloudToGridMap();

private:
	ros::NodeHandle& nodeHandle_;
	ros::Publisher gridMapPublisher_;
	ros::Publisher pcPublisher_;
	grid_map::GridMap map_;
	grid_map::GridMap map;
	ros::Subscriber pointCloudSubscriber_;

	std::string pointCloudTopic_;
	double mapLengthX_;
	double resolution_;
	double height_;
	double width_;
	int mlx;
	int add_x;
	int add_y;
	double i_;
	double counter;

};	
}