/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Kevin J. Walchko.
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
 * Author: Kevin J. Walchko on 6/20/2011
 *********************************************************************
 */

#include <pcl_tools/viewer_3d.h>
#include <pcl_tools/cloud_maker.hpp>

void Viewer::draw(){
	//drawText(30,30,"This is a test");
	if(!cloud_maker){
		ROS_ERROR("No CloudMaker filter created");
		return;
	}
	
	PointCloud::Ptr cloud_filtered = cloud_maker->getCloud();
	drawCloud( cloud_filtered );
}

void Viewer::init(){
	restoreStateFromFile();
	glDisable(GL_LIGHTING);
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

