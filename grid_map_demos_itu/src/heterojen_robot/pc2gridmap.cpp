#include "grid_map_demos/pc2gridmap.hpp"

/*
catkin_make --pkg grid_map_demos_itu
rosrun grid_map_demos_itu pointcloud_to_gridmap_node
rosrun pcl_ros pcd_to_pointcloud pcdfiles/total_loam.pcd

*/


using namespace std;
using namespace ros;
using namespace grid_map;
using namespace Eigen;
#include <unistd.h>
namespace grid_map_demos {
// rosrun pcl_ros pcd_to_pointcloud pcd_files/husky_20_mart.pcd 0.1 _frame_id:=/world cloud_pcd:=/cloud_out
// rosrun rviz rviz ~/catkin_ws/src/Husky/heterojen_robot/rviz/husky_vlp16.rviz 

PointCloudToGridMap::PointCloudToGridMap(ros::NodeHandle& nodeHandle)
	: nodeHandle_(nodeHandle)
{
		
	readParameters();
	initializeMap();
	pointCloudSubscriber_ = nodeHandle_.subscribe("/cloud_pcd",10,&PointCloudToGridMap::pointCloudCallback,this);
	gridMapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("grid_map",10,true);
	pcPublisher_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>("cloud_out",10,true);
	//temp_gridMapPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("grid_map_temp",1,true);
	printf("Starting ... \n" );
	
	//boost::thread spin_thread = boost::thread(boost::bind(&spinThread));
	ros::Rate r(10);
	while(nodeHandle_.ok()){
		ros::spinOnce();

		r.sleep();
	}
}
PointCloudToGridMap::~PointCloudToGridMap()
{
}

bool PointCloudToGridMap::readParameters()
{
	nodeHandle_.param("resolution",resolution_,0.5);
	nodeHandle_.param("height",height_,150.0);
	nodeHandle_.param("witdh",width_,150.0);
	//nodeHandle_.param("rate",rate_,5);
	
	mlx=1/resolution_;
	add_x=height_/(resolution_*2);
	add_y=width_/(resolution_*2);
	counter=0;
	
	return true;
}
bool PointCloudToGridMap::initializeMap()
{
	//map.setBasicLayers({"elevation"});
	map.add("elevation");
	map.setFrameId("world");
    map.setGeometry(grid_map::Length(height_, width_), resolution_,Position(0.0, 0.0));
    ROS_INFO("Initialized map with size %f x %f m (%i x %i cells).", map.getLength().x(),
             map.getLength().y(), map.getSize()(0), map.getSize()(1));
    for (GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    	map.at("elevation", *iterator)=0;
    }
    /*
    for (GridMapIterator iterator(map_temp); !iterator.isPastEnd(); ++iterator) {
    	map_temp.at("elevation", *iterator)=0;
    }
    auto& confidence=map["elevation"];
    */   

	return true;
}

void PointCloudToGridMap::pointCloudCallback(const sensor_msgs::PointCloud2& msg)
{

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  	pcl::fromROSMsg ( msg, *cloud);
  	double z_min=0,z_max=0;
  	for(size_t i = 0;i<cloud->points.size();++i){
		double y_;
		y_ = cloud->points[i].y;
		cloud->points[i].y= -cloud->points[i].z;
		cloud->points[i].z = y_;		
		if (z_min>cloud->points[i].z) z_min = cloud->points[i].z;
		if (z_max<cloud->points[i].z) z_max = cloud->points[i].z;
  	}
  	cout<<"z_min: "<<z_min<<" z_max: "<<z_max<<endl;
  	for(size_t i = 0;i<cloud->points.size();++i){
  		cloud->points[i].z=cloud->points[i].z - z_min/2;
  	}
  	

  	sensor_msgs::PointCloud2 cloud_msg;
	toROSMsg(*cloud, cloud_msg);
	cloud_msg.header.frame_id ="world";
	cloud_msg.header.stamp = ros::Time::now();
	pcPublisher_.publish(cloud_msg);

  	grid_map::GridMapIterator it(map);
  	auto& data_to = map["elevation"];
  	//Index inx;
  	// Low level elevation map
		for(size_t i = 0;i<cloud->points.size();++i){
			cloud->points[i].x=round((cloud->points[i].x)*mlx)/mlx;
			cloud->points[i].y=round((cloud->points[i].y)*mlx)/mlx;
			cloud->points[i].z=round((cloud->points[i].z)*mlx)/mlx;
		}

	for(size_t i = 0;i<cloud->points.size();++i){
		if ((cloud->points[i].x<height_/2 && cloud->points[i].y<width_/2) && (cloud->points[i].x>-height_/2 && cloud->points[i].y>-width_/2)){
			if (cloud->points[i].z<1){
				if (data_to(-1*(cloud->points[i].x*mlx)+add_x,-1*(cloud->points[i].y*mlx)+add_y)<cloud->points[i].z){
					data_to(-1*(cloud->points[i].x*mlx)+add_x,-1*(cloud->points[i].y*mlx)+add_y)=cloud->points[i].z;

				}
			}
		}	
	}
	for(size_t i = 0;i<cloud->points.size();++i){
		if ((cloud->points[i].x<height_/2 && cloud->points[i].y<width_/2) && (cloud->points[i].x>-height_/2 && cloud->points[i].y>-width_/2)){
			if (data_to(-1*(cloud->points[i].x*mlx)+add_x,-1*(cloud->points[i].y*mlx)+add_y)>0){
				if (data_to(-1*(cloud->points[i].x*mlx)+add_x,-1*(cloud->points[i].y*mlx)+add_y)<cloud->points[i].z){
					data_to(-1*(cloud->points[i].x*mlx)+add_x,-1*(cloud->points[i].y*mlx)+add_y)=cloud->points[i].z;
				}
			}
		}	
	}
	map["elevation"]=data_to;

	printf(" publishing the grid map!\n"); 
  // Publish as grid map.
  grid_map_msgs::GridMap mapMessage;
  grid_map::GridMapRosConverter::toMessage(map, mapMessage);
  mapMessage.info.header.frame_id="world";
  gridMapPublisher_.publish(mapMessage);
  counter+=1;
  //if (counter%10){map.}
}
}