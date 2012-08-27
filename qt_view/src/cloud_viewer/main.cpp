/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Kevin J. Walchko.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Kevin  nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Kevin J. Walchko on 3/1/2012
 *********************************************************************
 *
 * Simple 3D point cloud viewer
 *
 * Change Log:
 *  3 Mar 2012 Created
 *
 **********************************************************************
 *
 *  kinect -> raw_depth_image -> cloud_maker -> cloud_viewer
 *
 *  
 */

//----------- C++ -------------
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <sys/types.h>
#include <time.h>
#include <math.h>
#include <list>
#include <vector>
#include <iostream>

//------------ Boost -----------
#include <boost/thread/mutex.hpp>

//------------ PCL -------------
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
//#include <pcl/features/fpfh.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

//------------ ROS --------------
#include <ros/ros.h>
#include <image_geometry/pinhole_camera_model.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

namespace enc = sensor_msgs::image_encodings;

//----------- OpenCV -----------
#include <opencv2/opencv.hpp>

//------------ PCL Tools ---------
#include <pcl_tools/cloud_maker.hpp>
#include <pcl_tools/viewer_3d.h>


////////////////////////////////////////////////////////////////
#include <qapplication.h>
#include <QGLViewer/qglviewer.h>


/**
 * Simple Qt OpenGL viewer that displays point clouds
 */
class Filter : public CloudMaker {
public:

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointXYZRGB Point;

	/**
	 * Filters a point cloud to remove noise. For every point, it samples 50 (sor.setMeanK()) number
	 * of nearest neighbors and removes any points that are more than 1.0 std dev (sor.setStddevMulThresh())
	 * away from a point.
	 */
	PointCloud::Ptr statisticalFilter(PointCloud::Ptr cloud){
	  PointCloud::Ptr cloud_filtered (new PointCloud);
	
	  // Create the filtering object
	  pcl::StatisticalOutlierRemoval<Point> sor;
	  sor.setInputCloud (cloud);
	  sor.setMeanK (10);
	  sor.setStddevMulThresh (1.0);
	  sor.filter (*cloud_filtered);
	  
	  //std::cout<<"inPointCloud["<<cloud->points.size()<<"] StatisticalFilter["<<cloud_filtered->points.size()<<"]"<<std::endl;
	  
	  return cloud_filtered;
	
	}
	
	/**
	 * Downsample by converting a point cloud to a voxel grid.
	 */
	PointCloud::Ptr makeVoxelGrid(PointCloud::Ptr cloud){
	  PointCloud::Ptr cloud_filtered (new PointCloud);
	  
	  pcl::VoxelGrid<Point> vox;
	  vox.setInputCloud (cloud);
	  vox.setLeafSize (0.01f, 0.01f, 0.01f);
	  vox.filter (*cloud_filtered);
	  
	  //std::cout<<"inPointCloud["<<cloud->points.size()<<"]  VoxelGrid["<<cloud_filtered->points.size()<<"]"<<std::endl;
	  
	  return cloud_filtered;
	}
    
	virtual void process(){
		// do something with cloud
    	PointCloud::Ptr fc = getCloud(); 
    	//cloud = statisticalFilter( fc );
    	cloud = makeVoxelGrid(fc);
	}
};


////////////////////////////////////////////////////////////////



int main(int argc, char** argv)
{
  QApplication application(argc,argv);
  
    ros::init(argc, argv, "pcl_view");
    ros::NodeHandle n;
    ros::Rate loop_rate(100);
    ROS_INFO("Start");
    
	CloudMaker *filter = new Filter();
	
	Viewer *viewer = new Viewer();
	viewer->setFPSIsDisplayed(true);
	viewer->setFilter(filter);
	viewer->setWindowTitle("animation");
	

    image_transport::ImageTransport transport(n);
    image_transport::CameraSubscriber depth_sub = transport.subscribeCamera("/camera/depth/image_raw", 1, 
    												&Filter::depthCb, filter);
  
  	viewer->show();

  	return application.exec();
}
