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
 *
 *
 */


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