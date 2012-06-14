

#ifndef __VIEWER_3D_H__
#define __VIEWER_3D_H__

#include <qapplication.h>
#include <QGLViewer/qglviewer.h>

#include <pcl_tools/cloud_maker.hpp>
#include <pcl/point_types.h>
#include <ros/ros.h>


/**
 * Simple Qt OpenGL viewer that displays point clouds
 */
//template <class T>
class Viewer : public QGLViewer {

public:

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::PointXYZRGB Point;

	Viewer(): cloud(new PointCloud) { cloud_maker = NULL;}
	//virtual ~Viewer();
	
	void setFilter(CloudMaker* cm){ cloud_maker = cm;}
	void setCloud(PointCloud::Ptr c){ cloud = c;}
	//void setCloud(const T);
	
protected :

  virtual void draw();
  virtual void init();
  virtual void animate();
  virtual QString helpString() const ;

	void drawCloud(PointCloud::Ptr);
	//void drawCloud(T);
	
	PointCloud::Ptr cloud;
	//T cloud;
	
	CloudMaker *cloud_maker;
};


#endif