
#ifndef __CLOUD_MAKER_HPP__
#define __CLOUD_MAKER_HPP__

//----------- C++ -------------
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <sys/types.h>
#include <time.h>
#include <math.h>
#include <list>
#include <vector>
#include <boost/thread/mutex.hpp>

//------------ PCL -------------
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
//#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
//#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <iostream>

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
//#include <opencv2/highgui/highgui.hpp>

//----------- Glut & OpenGL -----
#include <GLUT/glut.h>	 // GLUT
#include <OpenGL/glu.h>	    // GLU
#include <OpenGL/gl.h>	    // OpenGL



/*
#ifndef PI
#define PI 3.1415926535897
#endif
#ifndef LARGE
#define LARGE 3.1415926535897
#endif

using namespace std;

// colors
float WHITE[] = {1,1,1,1};
float BLACK[] = {0,0,0,1};
float DARK[] = {.1,.1,.1,1};
float LIGHTGRAY[] = {.8,.8,.8,1};
float LIGHT[] = {.9,.9,.9,1};
float RED[] = {1,0,0,1};
float BLUE[] = {0,0,1,1};
float GREEN[] = {0,1,0,1};
float ORANGE[] = {1,.5,0,1};
float LIGHTBLUE[] = {0,1,1,1};
float PURPLE[] = {1,0,1,1};
*/


//////////////////////////////////////////////////////////////

#include <algorithm>
#include <limits>


namespace enc = sensor_msgs::image_encodings;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class CloudMaker {
public:
    CloudMaker() : cloud (new PointCloud){
        ;
    }
    
    ~CloudMaker(){;}
    
    
    template<typename T> void fillCloud(cv::Mat& depth){
       
#if 0    
        testCloud();
        return;
#elif 1     
        
        //ROS_INFO("fillCloud");
        boost::mutex::scoped_lock lock (buffer_mutex);
        cloud->width = cloud->height = 0;
        cloud->resize(0);
        
        const int width_ = depth.cols;
        const int height_ = depth.rows;
        const double SHIFT_SCALE = 0.125;
        const double shift_offset_ = 1084.0;
        const double baseline_ = 0.075; // 7.5 cm
        const double fT = model.fx() * baseline_;
        
        // can I do this in one loop?
        for (int v = 0; v < height_; ++v) // rows
        {
          for (int u = 0; u < width_; ++u) // cols
          {
            double d = SHIFT_SCALE * (shift_offset_ - (double)(depth.at<T>(v,u))); // disparity
            if (d <= 0.0) // div by zero
              continue;
    
            // Fill in XYZ
            pcl::PointXYZ pt;
            pt.z = fT / d;
            pt.x = ((u - model.cx()) / model.fx()) * pt.z;
            pt.y = ((v - model.cy()) / model.fy()) * pt.z;
            
            if(pt.z < 0.01) continue;
            
            cloud->points.push_back(pt);
            //ROS_INFO("%f %f %f",pt.x,pt.y,pt.z);
    
            /*
            // Fill in RGB from corresponding pixel in rectified RGB image
            Eigen::Vector4d uvd1;
            uvd1 << u, v, d, 1;
            Eigen::Vector3d uvw;
            uvw = depth_to_rgb_ * uvd1;
            int u_rgb = uvw[0]/uvw[2] + 0.5;
            int v_rgb = uvw[1]/uvw[2] + 0.5;
            
            int32_t rgb_packed = 0;
            if (u_rgb >= 0 && u_rgb < width_ && v_rgb >= 0 && v_rgb < height_) {
              cv::Vec3b rgb = rgb_rect_.at<cv::Vec3b>(v_rgb, u_rgb);
              rgb_packed = (rgb[0] << 16) | (rgb[1] << 8) | rgb[2];
            }
            cloud_.channels[0].values.push_back(*(float*)(&rgb_packed));
            */
          }
        }
#elif 0 // not sure value? use pinhole camerea model ray to 3d    
        
        //ROS_INFO("fillCloud");
        boost::mutex::scoped_lock lock (buffer_mutex);
        cloud->width = cloud->height = 0;
        cloud->resize(0);
        
        const int width_ = depth.cols;
        const int height_ = depth.rows;
        const double SHIFT_SCALE = 0.125;
        const double shift_offset_ = 1084.0;
        const double baseline_ = 0.075; // 7.5 cm
        const double fT = model.fx() * baseline_;
        
        // can I do this in one loop?
        int k = 0;
        for (int v = 0; v < height_; ++v) // rows
        {
          for (int u = 0; u < width_; ++u, ++k) // cols
          {
            double d = SHIFT_SCALE * (shift_offset_ - (double)(depth.at<T>(v,u))); // disparity
            if (d <= 0.0) // div by zero
              continue;
    
            // Fill in XYZ
            pcl::PointXYZ pt;
            
            cv::Point2d p2(u,v);
            cv::Point3d p3 = fT/d*model.projectPixelTo3dRay(p2);
            pt.x = p3.x;
            pt.y = p3.y;
            pt.z = p3.z;
            
            cloud->points.push_back(pt);
            
          }
        }    
#else             
        // Generate pointcloud data
        boost::mutex::scoped_lock lock (buffer_mutex);
        if(cloud->width != 640.0*480.0){
            cloud->width = 640.0f*480.0f;
            cloud->height = 1;
            cloud->points.resize (cloud->width * cloud->height);
        }
        
        //float max = -1.0;
        //float min = 1000000.0;
        
        for(int i=0;i<480;++i)
            for(int j=0;j<640;++j){
                cloud->points[j+i*640].x = j;
                cloud->points[j+i*640].y = i;
                cloud->points[j+i*640].z = (float)(depth.at<T>(i,j));
                
                //max = (cloud->points[j+i*640].z > max ? cloud->points[j+i*640].z : max);
                //min = (cloud->points[j+i*640].z < min ? cloud->points[j+i*640].z : min);
            }
                
        //ROS_INFO("max %f min %f",max,min);       
#endif   

        
    }

    /**
     * Callback which converts a depth image and camera info into a point cloud
     */
    void depthCb(const sensor_msgs::ImageConstPtr& depth_msg,
                 const sensor_msgs::CameraInfoConstPtr& info_msg)
    {
        // Update camera model
        model.fromCameraInfo(info_msg);
        
        // convert image
        cv_bridge::CvImagePtr cv_msg;
        
        //ROS_INFO("model: %s %d %d", info_msg->header.frame_id,info_msg->width,info_msg->height); 

        // check /include/sensor_msgs/image_encodings.h
        if (depth_msg->encoding == enc::TYPE_8UC1)
            //convert<uint16_t>(depth_msg, cloud_msg);
            ROS_INFO("TYPE_8UC1 not supported");

        else if (depth_msg->encoding == enc::TYPE_32FC1)
            //convert<float>(depth_msg, cloud_msg);
            ROS_INFO("TYPE_32FC1 not supported");
            
        else if (depth_msg->encoding == enc::MONO8){
            //convert<float>(depth_msg, cloud_msg);
            //ROS_INFO("MONO8");
            cv_msg = cv_bridge::toCvCopy(depth_msg, "mono8");
            fillCloud<unsigned char>(cv_msg->image);
        }
            
        else if (depth_msg->encoding == enc::MONO16)
            //convert<float>(depth_msg, cloud_msg);
            ROS_INFO("MONO16 not supported");

        else
        {
            //NODELET_ERROR_THROTTLE(5, "Depth image has unsupported encoding [%s]", depth_msg->encoding.c_str());
            ROS_ERROR("Error Cloud::depthCb got: %s",depth_msg->encoding.c_str());
            return;
        }

        /*
        //ROS_INFO("depthCb");
        //PointCloud::Ptr cloud_msg(new PointCloud);
        PointCloud::Ptr cloud_msg(cloud);
        cloud_msg->header = depth_msg->header;
        cloud_msg->height = depth_msg->height;
        cloud_msg->width  = depth_msg->width;
        cloud_msg->is_dense = false;
        cloud_msg->points.resize(cloud_msg->height * cloud_msg->width);
        */
        //pub_point_cloud_.publish (cloud_msg);
    }


    PointCloud::Ptr testCloud(float s=1024.0f) {

        // Generate pointcloud data
        boost::mutex::scoped_lock lock (buffer_mutex);
        cloud->width = 640.0f*480.0f;
        cloud->height = 1;
        cloud->points.resize (cloud->width * cloud->height);

        for (size_t i = 0; i < cloud->points.size (); ++i)
        {
            cloud->points[i].x = 640.0 * rand () / (RAND_MAX + 1.0f);
            cloud->points[i].y = 480.0 * rand () / (RAND_MAX + 1.0f);
            cloud->points[i].z = s * rand () / (RAND_MAX + 1.0f);
        }

        return cloud;

    }
    
    void gl(){
        boost::mutex::scoped_lock lock (buffer_mutex);
        glBegin(GL_POINTS);
            for (size_t i = 0; i < cloud->points.size (); ++i) 
                glVertex3f(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
        glEnd();
        
    }

protected:
    image_geometry::PinholeCameraModel model;
    PointCloud::Ptr cloud;
    boost::mutex buffer_mutex;

};

#endif