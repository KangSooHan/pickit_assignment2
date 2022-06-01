#include <sensor_msgs/PointCloud2.h>

#include <sstream>

#include "../include/pcl_driver.h"
#include "../include/marker_driver.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pickit_assignment");
	ros::NodeHandle n;

	PCLDriver p;
	MarkerDriver m;

	m.setPub(n.advertise<visualization_msgs::MarkerArray>("marker_array", 1));
	p.setedgePub(n.advertise<sensor_msgs::PointCloud2>("plane_pcd", 1));
	p.setplanePub(n.advertise<sensor_msgs::PointCloud2>("edge_pcd", 1));
	p.setintersectPub(n.advertise<sensor_msgs::PointCloud2>("intersect_pcd", 1));
	p.setMarkerDriver(m);

	ros::Subscriber sub1 = n.subscribe<pickit::PMarker>("marker_show", 1, &MarkerDriver::Subscribe , &m);
	ros::Subscriber sub2 = n.subscribe<pickit::Select>("select", 1, &PCLDriver::Select, &p);
	ros::Subscriber sub3 = n.subscribe<pickit::PPcd>("pcd_show", 1, &PCLDriver::Subscribe, &p);

	ros::spin();

	return 0;
}
