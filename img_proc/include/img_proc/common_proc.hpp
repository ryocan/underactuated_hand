//-----------------------------------------------------
// INCLUDE FILES
//-----------------------------------------------------
// standard libraries
#include <iostream>

// about image processing
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// about ros
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>

#include <std_msgs/String.h>
#include <random>
//-----------------------------------------------------
// CLASS
//-----------------------------------------------------
class CommonProc
{
    private:
	    /*--- ros setup ---*/
		ros::NodeHandle nh;
		image_transport::ImageTransport it;
		image_transport::Subscriber img_color_sub;
		image_transport::Subscriber img_depth_sub;

		/*--- image callback ---*/
		void imgColorCb(const sensor_msgs::ImageConstPtr& msg);
		void imgDepthCb(const sensor_msgs::ImageConstPtr& msg);

    public:
		CommonProc():it(nh)
		{
			img_color_sub = it.subscribe("/camera/color/image_raw", 1, &CommonProc::imgColorCb, this);
			img_depth_sub = it.subscribe("/camera/aligned_depth_to_color/image_raw", 1, &CommonProc::imgDepthCb, this);
		}

		~CommonProc()
		{
            cv::destroyAllWindows();
		}

        /*--- images ---*/
        cv::Mat img_src_color_;
        cv::Mat img_src_depth_;
        cv::Mat img_black_;

        /*--- string ---*/
        std::string mode_obj = "obj";
        std::string mode_hand = "hand";

        /*--- for contact detection using in drawContours ---*/
        std::vector<cv::Point> centroid_hand_, centroid_obj_;
        int contact_hand_l_ = 0;
        int contact_hand_r_ = 0;

        /*--- functions ---*/
        // extract region based on its color
        cv::Mat extractRegion(int H_MIN, int H_MAX, int S_MIN, int S_MAX, int V_MIN, int V_MAX);

        // calculation contour from mask image: mode = obj, hand
        std::vector<std::vector<cv::Point>> calcContours(cv::Mat img_mask, std::string mode);

        // calculate centroid 
        std::vector<cv::Point> calcCentroid(std::vector<std::vector<cv::Point>> contours, std::string mode);

        // draw contours using Line Iterator for contact detection 
        void drawContours(std::vector<std::vector<cv::Point>> contours, cv::Mat img_mask, std::string mode);
};