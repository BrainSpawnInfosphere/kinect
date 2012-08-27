
#ifndef __CLOUD_MAKER_HPP__
#define __CLOUD_MAKER_HPP__

//----------- C++ -------------
#include <boost/thread/mutex.hpp>

//------------ PCL -------------
#include <pcl/point_types.h>

//------------ ROS --------------
#include <ros/ros.h>
#include <image_geometry/pinhole_camera_model.h>
//#include <camera_info_manager/camera_info_manager.h>
//#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
//#include <sensor_msgs/PointCloud.h>
//#include <sensor_msgs/PointCloud2.h>

namespace enc = sensor_msgs::image_encodings;

//----------- OpenCV -----------
#include <opencv2/opencv.hpp>

#include "colorizer.hpp"

/*
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

// should these be global like this or scoped in the class??
namespace enc = sensor_msgs::image_encodings;
//typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

class CloudMaker {
public:

//namespace enc = sensor_msgs::image_encodings;
//typedef sensor_msgs::image_encodings enc;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

    CloudMaker() : cloud (new PointCloud){
    }
    
    ~CloudMaker(){;}
    
    
    template<typename T> void fillCloud(cv::Mat& depth){
        
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
        unsigned char rgb[3];
        
        // can I do this in one loop?
        //int k=0;
        for (int v = 0; v < height_; ++v) // rows
        {
          for (int u = 0; u < width_; ++u /*, k+=3*/) // cols
          {
            unsigned short pixel = depth.at<T>(v,u);
            double d = SHIFT_SCALE * (shift_offset_ - (double)(pixel)); // disparity
            if (d <= 0.0) // div by zero
              continue;
    
            // Fill in XYZ
            pcl::PointXYZRGB pt;
            
#if 1           
            pt.z = fT / d;
            pt.x = ((u - model.cx()) / model.fx()) * pt.z;
            pt.y = ((v - model.cy()) / model.fy()) * pt.z;
#else
            cv::Point2d p2(u,v);
            cv::Point3d p3 = fT/d*model.projectPixelTo3dRay(p2);
            pt.x = p3.x;
            pt.y = p3.y;
            pt.z = p3.z;

#endif            
            depthColor.depthColor(pixel,rgb);
            pack(pt,rgb[0],rgb[1],rgb[2]);
            cloud->points.push_back(pt);
            //ROS_INFO("%f %f %f",pt.x,pt.y,pt.z);
          }
        }
        
    }

    /**
     * Callback which converts a depth image and camera info into a point cloud. A
     * virtual function process() is called so that users can define their own 
     * additional processing.
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
            ROS_INFO("MONO8 not supported");
            //cv_msg = cv_bridge::toCvCopy(depth_msg, "mono8");
            //fillCloud<unsigned char>(cv_msg->image);
        }
            
        else if (depth_msg->encoding == enc::MONO16){
            //convert<float>(depth_msg, cloud_msg);
            //ROS_INFO("MONO16 not supported");
            cv_msg = cv_bridge::toCvCopy(depth_msg, "mono16");
            fillCloud<unsigned short>(cv_msg->image);
        }

        else
        {
            //NODELET_ERROR_THROTTLE(5, "Depth image has unsupported encoding [%s]", depth_msg->encoding.c_str());
            ROS_ERROR("Error Cloud::depthCb got: %s",depth_msg->encoding.c_str());
            return;
        }
        
        // perform additional user processing from the virtual function
        process();

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
    
    /*
    void convertToCloudMsg(){
    	//PointCloud::Ptr cloud_msg(new PointCloud);
        PointCloud::Ptr cloud_msg(cloud);
        cloud_msg->header = depth_msg->header;
        cloud_msg->height = depth_msg->height;
        cloud_msg->width  = depth_msg->width;
        cloud_msg->is_dense = false;
        cloud_msg->points.resize(cloud_msg->height * cloud_msg->width);
    }
    */

	/*
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
    */
    
    inline PointCloud::Ptr getCloud(){ return cloud; }

	/**
	 * Static member function to unpack the r, g, and b colors from the point 
	 * cloud. Got this form pointclouds.org.
	 */
	static void unpack(pcl::PointXYZRGB& p, uint8_t& r, uint8_t& g, uint8_t& b){
		 // unpack rgb into r/g/b
		 uint32_t rgb = *reinterpret_cast<int*>(&p.rgb);
		 r = (rgb >> 16) & 0x0000ff;
		 g = (rgb >> 8)  & 0x0000ff;
		 b = (rgb)       & 0x0000ff;
 	}
 	
protected:

	virtual void process() = 0;
	
	inline void pack(pcl::PointXYZRGB& p, const uint8_t r, const uint8_t g, const uint8_t b){
		// pack r/g/b into rgb
		 uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
		 p.rgb = *reinterpret_cast<float*>(&rgb);
	 }
 	
    image_geometry::PinholeCameraModel model;
    PointCloud::Ptr cloud;
    boost::mutex buffer_mutex;
    
    Colorize depthColor;

};

#endif