#include <pcl/common/io.h>
#include <pcl/filters/filter.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "../include/pcl_driver.h"

PCLDriver::PCLDriver(){	this->Start();}

PCLDriver::~PCLDriver(){}

void PCLDriver::Start()
{
	try
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcd_scene (new pcl::PointCloud<pcl::PointXYZRGB>);
		this->Read(scene_path, pcd_scene);
		this->RemoveNanColor(pcd_scene);

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr ransac_pcd (new pcl::PointCloud<pcl::PointXYZRGB>);

		pcl::copyPointCloud(*pcd_scene, *ransac_pcd);
		std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pcds = this->Ransac(ransac_pcd);

		this->edges = this->EdgeDetect(pcds);
		this->pcd_plane = pcd_scene;
		this->pcds = pcds;

		if(this->is_view)
		{
			this->View(pcds);
		}
		return;
	}
	catch(const std::exception& e)
	{
		std::cerr << e.what() << "\n";
	}
}


std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> PCLDriver::EdgeDetect(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pcds)
{

	/*
	pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	ne.setInputCloud (pcd);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
	ne.setSearchMethod (tree);
	ne.setRadiusSearch (0.001);
	ne.compute (*normal);
	*/

	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> edges;
	for(int i=0; i<pcds.size(); ++i){
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr shape (new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr centers (new pcl::PointCloud<pcl::PointXYZRGB>);
		std::vector<pcl::Vertices> polygons_alpha;

		pcl::ConcaveHull<pcl::PointXYZRGB> concave_hull;
		concave_hull.setInputCloud(pcds[i]);
		concave_hull.setAlpha(_Alpha);
		concave_hull.setVoronoiCenters(centers);
		concave_hull.reconstruct(*shape, polygons_alpha);

		for(auto &pts: shape->points)
		{
			pts.r = this->color[i][0];
			pts.g = this->color[i][1];
			pts.b = this->color[i][2];
		}

		edges.push_back(shape);
	}

	return edges;
}

int PCLDriver::Switch(int x)
{
	int y;
	switch(x)
	{
		case 1:
			y=3;
			break;
		case 2:
			y=2;
			break;
		case 3:
			y=4;
			break;
		case 4:
			y=1;
			break;
		case 5:
			y=0;
			break;
	}
	return y;
}


void PCLDriver::Select(const pickit::Select msg)
{
	std::string l1, l2;
	l1 = msg.data[0];
	l2 = msg.data[1];

	int e1_idx = l1.back() -'0';
	int e2_idx = l2.back() -'0';

	e1_idx = this->Switch(e1_idx);
	e2_idx = this->Switch(e2_idx);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr edge1 = edges[e1_idx];
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr edge2 = edges[e2_idx];
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr intersect (new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
	kdtree.setInputCloud(edge1);

	for(auto &pts:edge2->points)
	{
		std::vector<int> idxes;
		std::vector<float> sqr_dists;
		kdtree.radiusSearch(pts, _Radius, idxes, sqr_dists);
		if(idxes.size()!=0)
		{
			intersect->push_back(pts);
		}
	}

	sensor_msgs::PointCloud2 intersect_msg;
	pcl::toROSMsg(*intersect, intersect_msg);
	intersect_msg.header.frame_id = "map";
	this->intersect_pub.publish(intersect_msg);
	this->inter = intersect;

	if(intersect->points.size()==0)
	{
		std::cout << l1 << " & " << l2 << " are not adjacent\n" << std::endl;
	}
	this->md.addLineStrip(intersect);

	return;
}



pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCLDriver::getIntersect(){return this->inter;}
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> PCLDriver::getEdge(){return this->edges;}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr PCLDriver::getPlane(){return this->pcd_plane;}
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> PCLDriver::getPCD(){return this->pcds;}

void PCLDriver::View(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pcds)
{
	try
	{
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer1(new pcl::visualization::PCLVisualizer("Simple Cloud Viewer"));

		for(int i=0; i<5; ++i)
		{
			std::string title = "Plane_";
			title += std::to_string(i);

			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(pcds[i], this->color[i][0], this->color[i][1], this->color[i][2]);
			viewer1->addPointCloud<pcl::PointXYZRGB>(pcds[i], single_color, title);
		}
		while (!viewer1->wasStopped()) {
			viewer1->spinOnce();
		}
	}
	catch(const std::exception& e)
	{
		std::cerr << e.what() << "\n";
	}
}

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> PCLDriver::Ransac(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcd)
{
	try
	{
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr indices(new pcl::PointIndices);
		pcl::ExtractIndices<pcl::PointXYZRGB> extract;

		pcl::SACSegmentation<pcl::PointXYZRGB> seg;
		seg.setOptimizeCoefficients (true);
		seg.setModelType (pcl::SACMODEL_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setDistanceThreshold(_DistanceThreshold);


		std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pcds;
		for(int i=0; i<5; ++i)
		{
			seg.setInputCloud(pcd);
			seg.segment(*indices, *coefficients);

			if (indices->indices.size () == 0)
				break;

			extract.setInputCloud(pcd);
			extract.setIndices(indices);
			extract.setNegative(false);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_pcd (new pcl::PointCloud<pcl::PointXYZRGB>);
			extract.filter(*new_pcd);
			this->Removeoutlier(new_pcd);
			pcds.push_back(new_pcd);

			extract.setInputCloud(pcd);
			extract.setIndices(indices);
			extract.setNegative(true);
			pcl::PointCloud<pcl::PointXYZRGB> f_scene;
			extract.filter(f_scene);
			pcd->swap(f_scene);
		}
		return pcds;
	}
	catch(const std::exception& e)
	{
		std::cerr << e.what() << "\n";
	}
}

void PCLDriver::Read(std::string path, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcd)
{
	try{
		if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(path, *pcd) == -1)
		{
			std::cout << ("Couldn't read file " + path + "\n") << std::endl;
			return;
		}
		return;
	}
	catch(const std::exception& e)
	{
		std::cerr << e.what() << "\n";
	}
}

void PCLDriver::Subscribe(pickit::PPcd msg)
{
	try
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr clear (new pcl::PointCloud<pcl::PointXYZRGB>);
		std::cout << msg.scene << msg.edge << std::endl;
		if(msg.scene==0)
		{
			sensor_msgs::PointCloud2 plane_msg;
			pcl::toROSMsg(*clear, plane_msg);
			plane_msg.header.frame_id = "map";
			this->plane_pub.publish(plane_msg);
		}
		else
		{
			sensor_msgs::PointCloud2 plane_msg;
			pcl::toROSMsg(*this->pcd_plane, plane_msg);
			plane_msg.header.frame_id = "map";
			this->plane_pub.publish(plane_msg);
		}

		if(msg.edge==0)
		{
			sensor_msgs::PointCloud2 edge_msg;
			pcl::toROSMsg(*clear, edge_msg);
			edge_msg.header.frame_id = "map";
			this->edge_pub.publish(edge_msg);
		}
		else
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcd_edge (new pcl::PointCloud<pcl::PointXYZRGB>);
			for(int i=0; i<this->edges.size(); ++i)
			{
				*pcd_edge = *pcd_edge + *edges[i];
			}
			sensor_msgs::PointCloud2 edge_msg;
			pcl::toROSMsg(*pcd_edge, edge_msg);
			edge_msg.header.frame_id = "map";

			this->edge_pub.publish(edge_msg);
		}


		return;
	}
	catch(const std::exception& e)
	{
		std::cerr << e.what() << "\n";
	}
}

void PCLDriver::setintersectPub(ros::Publisher m)
{
	this->intersect_pub = m;
}

void PCLDriver::setplanePub(ros::Publisher m)
{
	this->plane_pub = m;
}

void PCLDriver::setedgePub(ros::Publisher m)
{
	this->edge_pub = m;
}

void PCLDriver::Removeoutlier(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcd)
{
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
	sor.setInputCloud(pcd);
	sor.setMeanK (50);
	sor.setStddevMulThresh (1.0);
	sor.filter (*pcd);
	return;
}

void PCLDriver::setMarkerDriver(MarkerDriver m)
{
	this->md = m;
}

void PCLDriver::RemoveNanColor(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcd)
{
	try
	{
		for(auto iter=pcd->begin(); iter!=pcd->end(); )
		{
			if(isnan(iter->rgb))
			{
				pcd->erase(iter);
			}
			else
			{
				++iter;
			}
		}
		return;
	}
	catch(const std::exception& e)
	{
		std::cerr << e.what() << "\n";
	}
}
