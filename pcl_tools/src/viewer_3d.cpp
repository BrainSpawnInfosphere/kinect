

//----------- C++ -------------
//#include <stdio.h>
//#include <stdlib.h>
//#include <assert.h>
//#include <sys/types.h>
//#include <time.h>
//#include <math.h>
//#include <list>
//#include <vector>
//#include <iostream>

//------------ Boost -----------
//#include <boost/thread/mutex.hpp>

//------------ PCL -------------
//#include <pcl/io/pcd_io.h>
//#include <pcl/point_types.h>
//#include <pcl/filters/passthrough.h>
//#include <pcl/filters/statistical_outlier_removal.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/features/normal_3d.h>
//#include <pcl/features/fpfh.h>
//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/sample_consensus/model_types.h>
//#include <pcl/segmentation/sac_segmentation.h>

//------------ ROS --------------
//#include <ros/ros.h>
//#include <image_geometry/pinhole_camera_model.h>
//#include <camera_info_manager/camera_info_manager.h>
//#include <image_transport/image_transport.h>
//#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs/Image.h>
//#include <sensor_msgs/image_encodings.h>
//#include <sensor_msgs/PointCloud.h>
//#include <sensor_msgs/PointCloud2.h>

//namespace enc = sensor_msgs::image_encodings;

//----------- OpenCV -----------
//#include <opencv2/opencv.hpp>


//------------ PCL Tools ---------
//#include <pcl_tools/cloud_maker.hpp>


#include <pcl_tools/viewer_3d.h>
#include <pcl_tools/cloud_maker.hpp>


//Viewer::Viewer(){;}

//Viewer::~Viewer(){;}

void Viewer::draw(){
	//drawText(30,30,"This is a test");
	PointCloud::Ptr cloud_filtered = cloud_maker->getCloud();
	drawCloud( cloud_filtered );
}

void Viewer::init(){
	restoreStateFromFile();
	glDisable(GL_LIGHTING);
	//glPointSize(3.0);
	setGridIsDrawn();
	//help(); 
	setAnimationPeriod(0); // fast as possible
	startAnimation();
	showEntireScene();
	//setSceneRadius(5.0);
	//setFPSIsDisplayed(true);
	
	// Make world axis visible
	setAxisIsDrawn(false);
	
	// Move camera according to viewer type (on X, Y or Z axis)
	camera()->setPosition(qglviewer::Vec(0.0,-1.0,-1.0));
	camera()->lookAt(sceneCenter());
	camera()->setUpVector(qglviewer::Vec(0.0,-1.0,0.0));
	
	//camera()->setType(Camera::ORTHOGRAPHIC);
	//camera()->showEntireScene();
	
	/*
	// Forbid rotation
	qglviewer::WorldConstraint* constraint = new qglviewer::WorldConstraint();
	constraint->setRotationConstraintType(qglviewer::AxisPlaneConstraint::FORBIDDEN);
	camera()->frame()->setConstraint(constraint);
	*/
}

void Viewer::animate(){
	ros::spinOnce();
}

QString Viewer::helpString() const {
  QString text("<h2>A n i m a t i o n</h2>");
  text += "Use the <i>animate()</i> function to implement the animation part of your ";
  text += "application. Once the animation is started, <i>animate()</i> and <i>draw()</i> ";
  text += "are called in an infinite loop, at a frequency that can be fixed.<br><br>";
  text += "Press <b>Return</b> to start/stop the animation.";
  return text;
}

void Viewer::drawCloud(PointCloud::Ptr cloud){
	uint8_t r = 0, g = 0, b = 0;
	glBegin(GL_POINTS);
		for (size_t i = 0; i < cloud->points.size (); ++i) {
			CloudMaker::unpack(cloud->points[i],r,g,b);
			glColor3ub(r,g,b);
			glVertex3f(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
		}
	glEnd();
}

