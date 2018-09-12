#include <iostream>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

int main (int argc, char* argv[])
{
  // Variables Declaration
  pcl::IterativeClosestPoint<PointT, PointT> icp;
  pcl::PointCloud<PointT>::Ptr input_ptr(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr target_ptr(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr output_ptr(new pcl::PointCloud<PointT>);
  int max_iter = 50;
  std::string input_name;
  std::string target_name;

  // ROS Initialization
  ros::init(argc, argv, "icp_matching");
  ros::NodeHandle n;
  ros::NodeHandle nh("~");
  ros::Publisher target_map_pub = n.advertise<sensor_msgs::PointCloud2>("/target_map", 10, 1);
  ros::Publisher input_map_pub  = n.advertise<sensor_msgs::PointCloud2>("/input_map", 10, 1);
  ros::Publisher output_map_pub = n.advertise<sensor_msgs::PointCloud2>("/output_map", 10, 1);

  // Get Parameters
  if (!nh.getParam("input", input_name))
  {
  	ROS_ERROR("Failed to get input filename");
  	return -1;
  }
  if (!nh.getParam("target", target_name))
  {
  	ROS_ERROR("Failed to get target filename");
  	return -1;
  }

  // Load PointCloud
  if (pcl::io::loadPCDFile<PointT>(input_name, *input_ptr) < 0)
  {
    ROS_ERROR("Failed to load %s.\n", input_name.c_str());
    return -1;
  }
  std::cout << "Loaded file " << input_name << " (" << input_ptr->size() << " points)" << std::endl;
  if (pcl::io::loadPCDFile<PointT>(target_name, *target_ptr) < 0)
  {
    ROS_ERROR("Failed to load %s.\n", target_name.c_str());
    return -1;
  }
  std::cout << "Loaded file " << target_name << " (" << target_ptr->size() << " points)" << std::endl;

  // Apply ICP
  icp.setMaximumIterations(max_iter);
  icp.setInputTarget(target_ptr);
  icp.setInputSource(input_ptr);
  std::cout << "Start aligning...\n";
  icp.align(*output_ptr);
  std::cout << "Fitness Score: " << icp.getFitnessScore() << std::endl;

  //Publish Result
  sensor_msgs::PointCloud2::Ptr cloud_msg_ptr(new sensor_msgs::PointCloud2);
  ros::Time now = ros::Time(0);
  pcl::toROSMsg(*target_ptr, *cloud_msg_ptr); //Target
  cloud_msg_ptr->header.frame_id = "icp";
  cloud_msg_ptr->header.stamp = now;
  target_map_pub.publish(*cloud_msg_ptr);
  pcl::toROSMsg(*input_ptr, *cloud_msg_ptr);  //Input
  cloud_msg_ptr->header.frame_id = "icp";
  cloud_msg_ptr->header.stamp = now;
  input_map_pub.publish(*cloud_msg_ptr);
  pcl::toROSMsg(*output_ptr, *cloud_msg_ptr); //Output
  cloud_msg_ptr->header.frame_id = "icp";
  cloud_msg_ptr->header.stamp = now;
  output_map_pub.publish(*cloud_msg_ptr);

  ros::spin();

  return 0;
}
