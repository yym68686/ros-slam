#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
void cloudCB(const sensor_msgs::PointCloud2 &input)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(input, cloud);//从ROS类型消息转为PCL类型消息
	// for(int i=0;i<cloud.points.size();i++)
	// {
	// 	std::cout << cloud.points[i] << std::endl;
	// }
  pcl::io::savePCDFileASCII("test_pcd.pcd", cloud);//保存pcd
  printf("height: %d, width: %d\n", input.height, input.width);
}

int main (int argc, char **argv)
{
  ros::init (argc, argv, "pcl_write");
  ros::NodeHandle nh;
  ros::Subscriber bat_sub = nh.subscribe("/camera/depth/points", 10, cloudCB);//接收点云
  ros::spin();
  return 0;
}