/**
 * Todo:
 * - fix command line
 * - break Colorize out into own file
 *   - add point cloud capability im->cloud->im
 *   - handle native ROS Image/Cloud msgs
 */

//---------- ROS ---------------
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

//----------- OpenCV -----------
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>


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
	
protected:

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

Colorize color;

// create display image
cv::Mat img(480, 640, CV_8UC3);

void depthCb( const sensor_msgs::ImageConstPtr& image )
{
    // convert to cv image
    cv_bridge::CvImagePtr bridge;
        
    try
    {
        if (image->encoding == sensor_msgs::image_encodings::MONO16){
            bridge = cv_bridge::toCvCopy(image, "mono16");
            color.convert<unsigned short>(bridge->image,img);
        }
        else{
            ROS_ERROR("Error Cloud::depthCb got: %s",image->encoding.c_str());
            return;
        }
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Failed to transform depth image.");
        return;
    }
    
    // display
    cv::imshow("Depth Viewer", img);
    cv::waitKey(1);
}

int main( int argc, char* argv[] )
{
    ros::init( argc, argv, "depth_viewer" );
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    cv::namedWindow("Depth Viewer");
    
    ros::Subscriber sub = n.subscribe("/camera/depth/image_raw", 3, &depthCb);
    ros::spin();
}

