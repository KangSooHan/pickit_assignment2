#ifndef PCLDRIVER_H_
#define PCLDRIVER_H_

#include <iostream>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include "pickit/Select.h"
#include "pickit/PPcd.h"
#include "../include/marker_driver.h"

#define _DistanceThreshold 0.005
#define _Alpha 0.04
#define _Radius 0.02

class PCLDriver
{
public:
	PCLDriver();
	~PCLDriver();
	void Start();
	void View(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pcds);
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> Ransac(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcd);
	void Read(std::string path, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcd);
	pcl::PointCloud<pcl::PointXYZRGB> Read(std::string path);
	void RemoveNanColor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcd);
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> getEdge();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPlane();
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> getPCD();
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> EdgeDetect(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pcds);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getIntersect();

	void setMarkerDriver(MarkerDriver m);
	void Removeoutlier(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcd);
	void Select(const pickit::Select msg);
	void setedgePub(ros::Publisher m);
	void setplanePub(ros::Publisher m);
	void setintersectPub(ros::Publisher m);
	void Subscribe(pickit::PPcd msg);
	int Switch(int x);

private:
	bool is_view=false;
	int color[5][3] = {{30, 90, 90}, {100, 50, 20}, {100, 100, 20}, {0, 50, 100}, {50, 0, 50}};

	ros::Publisher edge_pub;
	ros::Publisher plane_pub;
	ros::Publisher intersect_pub;

	std::string edge_path = "/home/shkang/pickit_assignment/src/pickit/pcd/contour.pcd"; 
	std::string scene_path = "/home/shkang/pickit_assignment/src/pickit/pcd/scene.pcd"; 
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcd_plane;
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> edges;
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pcds;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr inter;
	MarkerDriver md;
};

#endif
