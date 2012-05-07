
#ifndef __COLORIZER_HPP__
#define __COLORIZER_HPP__

//------------ ROS --------------
#include <ros/ros.h>
//#include <image_geometry/pinhole_camera_model.h>
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


/**
 *
 */
class Colorize {
public:
    Colorize(){
        int i;
        for (i=0; i<2048; i++) {
            float v = i/2048.0;
            v = powf(v, 3)* 6;
            t_gamma[i] = v*6*256;
        }
	}
	
	
	template<typename T> cv::Mat& convert(cv::Mat& src, cv::Mat& dst){
	    dst.create(src.rows, src.cols, CV_8UC3); // only creates if different
        
#if 0 // doesn't work??
        cv::Mat_<cv::Vec3b>::iterator it = dst.begin<cv::Vec3b>();
        cv::Mat_<cv::Vec3b>::iterator itEnd = dst.end<cv::Vec3b>();
        
        cv::Mat_<T>::iterator ii = src.begin<T>();
        
        for(; it != itEnd; ++it, ++ii) depthColor(*ii,(T*)it);
#else
		cv::Vec3b p;
		for (int v = 0; v < dst.rows; ++v) // rows
        {
          for (int u = 0; u < dst.cols; ++u) // cols
          {
            unsigned short pixel = src.at<T>(v,u);
            p = dst.at<cv::Vec3b>(v,u);
            depthColor(pixel,&p[0]);
            dst.at<cv::Vec3b>(v,u) = p;
          }
        }

#endif
	    
	    
	    return dst;
	}
	
//protected:

	// From libfreenect - colorizes 16b depth image
	inline void depthColor(const unsigned short depth, unsigned char* depth_mid){
		int pval = t_gamma[depth];
		int lb = pval & 0xff;
		switch (pval>>8) { // switch on significant bits
			case 0:
				depth_mid[0] = 255;
				depth_mid[1] = 255-lb;
				depth_mid[2] = 255-lb;
				break;
			case 1:
				depth_mid[0] = 255;
				depth_mid[1] = lb;
				depth_mid[2] = 0;
				break;
			case 2:
				depth_mid[0] = 255-lb;
				depth_mid[1] = 255;
				depth_mid[2] = 0;
				break;
			case 3:
				depth_mid[0] = 0;
				depth_mid[1] = 255;
				depth_mid[2] = lb;
				break;
			case 4:
				depth_mid[0] = 0;
				depth_mid[1] = 255-lb;
				depth_mid[2] = 255;
				break;
			case 5:
				depth_mid[0] = 0;
				depth_mid[1] = 0;
				depth_mid[2] = 255-lb;
				break;
			default:
				depth_mid[0] = 0;
				depth_mid[1] = 0;
				depth_mid[2] = 0;
				break;
		}
	}

    unsigned short t_gamma[2048];
};

#endif