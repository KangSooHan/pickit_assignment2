#ifndef MARKERDRIVER_H_
#define MARKERDRIVER_H_

#include <ros/ros.h>
#include <kdl/frames.hpp>
#include <kdl/frames_io.hpp>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "pickit/PMarker.h"
#include <pcl/io/pcd_io.h>

class MarkerDriver
{
public:
	MarkerDriver();
	~MarkerDriver();

	void Start();
	void emitAxes(const std::string type_name, KDL::Frame f);
	KDL::Frame Pose2KDL(double x, double y, double z, double x2, double y2, double z2, double w2);
	void CentroidMarker();
	visualization_msgs::MarkerArray getMarkerArray();
	void Publish();
	void Subscribe(pickit::PMarker msg);
	void Clear();
	void setPub(ros::Publisher m);
	void addLineStrip(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcd);

private:
	visualization_msgs::MarkerArray markerArray;
	ros::Publisher marker_pub;
	int l1_idx;
	int l2_idx;
};
#endif

