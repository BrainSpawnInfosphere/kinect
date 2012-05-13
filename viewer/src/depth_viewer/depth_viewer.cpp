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

#include <pcl_tools/colorizer.hpp>


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
    
    std::string name;
    
    if(argc == 2){
    	name = argv[1];
    }
    else {
    	name = "/camera/depth/image_raw";
    }
    
    //ros::Subscriber sub = n.subscribe("/camera/depth/image_raw", 3, &depthCb);
    ros::Subscriber sub = n.subscribe(name.c_str(), 3, &depthCb);
    ros::spin();
}

