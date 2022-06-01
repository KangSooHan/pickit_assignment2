#include "../include/marker_driver.h"

MarkerDriver::MarkerDriver()
{
	this->Start();
}
MarkerDriver::~MarkerDriver(){}

void MarkerDriver::Start()
{
	try
	{
		visualization_msgs::MarkerArray markerArray;
		this->markerArray = markerArray;
	}
	catch(const std::exception& e)
	{
		std::cerr << e.what() << "\n";
	}
}

void MarkerDriver::setPub(ros::Publisher m)
{
	this->marker_pub = m;
}

void MarkerDriver::Publish()
{
	try
	{
		this->marker_pub.publish(this->markerArray);
		return;
	}
	catch(const std::exception& e)
	{
		std::cerr << e.what() << "\n";
	}
}

void MarkerDriver::Subscribe(pickit::PMarker msg)
{
	try
	{
		if(msg.data==0)
		{
			this->Clear();
		}
		else
		{
			this->CentroidMarker();
			this->Publish();
		}
		return;
	}
	catch(const std::exception& e)
	{
		std::cerr << e.what() << "\n";
	}
}


void MarkerDriver::Clear()
{
	try
	{
		visualization_msgs::MarkerArray markerArray;
		this->markerArray = markerArray;
		visualization_msgs::Marker marker;
		marker.header.frame_id = "map";
		marker.header.stamp = ros::Time::now();
		marker.action = visualization_msgs::Marker::DELETEALL;
		this->markerArray.markers.push_back(marker);

		this->Publish();
	}
	catch(const std::exception& e)
	{
		std::cerr << e.what() << "\n";
	}
}



visualization_msgs::MarkerArray MarkerDriver::getMarkerArray()
{
	return this->markerArray;
}

void MarkerDriver::emitAxes(const std::string type_name, KDL::Frame f)
{
	try
	{
		visualization_msgs::Marker marker;
		marker.header.frame_id = "map";
		marker.header.stamp = ros::Time::now();
		marker.ns = "marker_" + type_name;
		marker.id = 0;
		marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		marker.action = visualization_msgs::Marker::ADD;
		marker.pose.position.x = f.p[0];
		marker.pose.position.y = f.p[1];
		marker.pose.position.z = f.p[2];

		double x, y, z, w;
		f.M.GetQuaternion(x, y, z, w);

		marker.pose.orientation.x = x;
		marker.pose.orientation.y = y;
		marker.pose.orientation.z = z;
		marker.pose.orientation.w = w;
		marker.text = type_name;
		marker.scale.x = 0.05;
		marker.scale.y = 0.05;
		marker.scale.z = 0.05;
		marker.color.r = 0.0f;
		marker.color.g = 0.0f;
		marker.color.b = 0.0f;
		marker.color.a = 1.0;


		this->markerArray.markers.push_back(marker);
	
		for(int i=0; i<3; ++i){
			visualization_msgs::Marker marker;
			marker.header.frame_id = "map";
			marker.header.stamp = ros::Time::now();
			marker.ns = "marker_" + type_name;
			marker.id = i+1;
			marker.type = visualization_msgs::Marker::ARROW;
			marker.action = visualization_msgs::Marker::ADD;

			marker.scale.x = 0.1;
			marker.scale.y = 0.01;
			marker.scale.z = 0.01;

			KDL::Rotation rot = f.M;

			if(i==0)
			{
				marker.color.r = 1.0f;
				marker.color.g = 0.0f;
				marker.color.b = 0.0f;
				marker.color.a = 1.0;
			}
			else if(i==1)
			{
				marker.color.r = 0.0f;
				marker.color.g = 1.0f;
				marker.color.b = 0.0f;
				marker.color.a = 1.0;

				KDL::Rotation align_y = KDL::Rotation::RPY(0.0, 0.0, KDL::PI/2);
				rot = rot * align_y;
			}
			else
			{
				marker.color.r = 0.0f;
				marker.color.g = 0.0f;
				marker.color.b = 1.0f;
				marker.color.a = 1.0;

				KDL::Rotation align_z = KDL::Rotation::RPY(0.0, -KDL::PI/2, 0.0);
				rot = rot * align_z;
			}

			double x, y, z, w;
			rot.GetQuaternion(x, y, z, w);

			marker.pose.position.x = f.p[0];
			marker.pose.position.y = f.p[1];
			marker.pose.position.z = f.p[2];
			marker.pose.orientation.x = x;
			marker.pose.orientation.y = y;
			marker.pose.orientation.z = z;
			marker.pose.orientation.w = w;

			this->markerArray.markers.push_back(marker);
		}
	}
	catch(const std::exception& e)
	{
		std::cerr << e.what() << "\n";
	}
}

KDL::Frame MarkerDriver::Pose2KDL(double x, double y, double z, double x2, double y2, double z2, double w2)
{
	try
	{
		return KDL::Frame(KDL::Rotation::Quaternion(x2, y2, z2, w2), KDL::Vector(x, y, z));
	}
	catch(const std::exception& e)
	{
		std::cerr << e.what() << "\n";
	}
}

void MarkerDriver::CentroidMarker()
{
	try
	{
		this->emitAxes("object1", this->Pose2KDL(-0.396642696188, 0.185761211272, 0.165868533873, -0.094055412734, -0.63493267117,  -0.0342168924793, 0.766056993384));
		this->emitAxes("object2", this->Pose2KDL(0.40158250532, 0.151780680023, 0.136891953985, -0.442444389804, 0.513437165111, 0.575184383585, -0.458026437098));
		this->emitAxes("object3", this->Pose2KDL(-0.270029758085, 0.307718964573,  0.135729257441, 0.450175553348, -0.39612089603, 0.519462550037, 0.608760138084));
		this->emitAxes("object4", this->Pose2KDL(0.116662200277, 0.319071585796,  0.131018938899, 0.602783144274, -0.00900398625071, 0.00323214799844, 0.797847725993));
		this->emitAxes("object5", this->Pose2KDL(-0.0155791516918, 0.0496786172516, 0.0157975119176, -0.00127696317417, 0.00029492956528, 0.0229005112278, 0.999736896456));
	}
	catch(const std::exception& e)
	{
		std::cerr << e.what() << "\n";
	}
}


void MarkerDriver::addLineStrip(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcd)
{
	this->Clear();
	this->CentroidMarker();
	visualization_msgs::Marker marker;
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time::now();
	marker.ns = "marker_Line";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::LINE_STRIP;
	marker.action = visualization_msgs::Marker::ADD;

	marker.scale.x = 0.005;
	marker.color.r = 0.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0;

	marker.pose.orientation.x = 0;
	marker.pose.orientation.y = 0;
	marker.pose.orientation.z = 0;
	marker.pose.orientation.w = 1;

	for(auto & pts: pcd->points)
	{
		geometry_msgs::Point p;
		p.x = pts.x;
		p.y = pts.y;
		p.z = pts.z;
		marker.points.push_back(p);
	}
	this->markerArray.markers.push_back(marker);
	this->Publish();
}
