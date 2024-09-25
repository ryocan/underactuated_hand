//-----------------------------------------------------
// INCLUDE FILES
//-----------------------------------------------------
#include "img_proc/fb_control.hpp"

/****************************************************************************************
 *	@Function
 *		[Main function for object detection] detect object
****************************************************************************************/
bool FbControl::objectDetection()
{
	// if background image is available, then do background subtraction and object detection
	if(getBgImage())
	{
		backgroundSubtraction();
		objectColorExtraction();
		return true;
	}

	return false;
}

/****************************************************************************************
 *	@Function
 *		only get background image.
 *		The reason this function is boolean is because this process needs to be in a loop.
****************************************************************************************/
bool FbControl::getBgImage()
{
	if (cv::waitKey(1) == 's') 
	{
        img_fg_ = img_src_color_.clone();
		return true;
	}
	else
		return false;
}

/****************************************************************************************
 *	@Function
 *		do background subtraction and detect object
****************************************************************************************/
void FbControl::backgroundSubtraction()
{
	// change color space
	cv::Mat img_bg_gray, img_fg_gray;
	cv::cvtColor(img_bg_, img_bg_gray, cv::COLOR_BGR2GRAY);
	cv::cvtColor(img_fg_, img_fg_gray, cv::COLOR_BGR2GRAY);

	// background subtraction
	cv::Mat img_diff_abs;
	cv::absdiff(img_bg_gray, img_fg_gray, img_diff_abs);

	// noise handling
	cv::Mat img_diff_color = cv::Mat::zeros(img_src_color_.size(), img_src_color_.type());
	for (int v = img_diff_abs.rows/2 ; v < img_diff_abs.rows; v++)
	{
		for (int u = 0; u < img_diff_abs.cols; u++)
		{
			uchar pixel_gray = img_diff_abs.at<uchar>(v, u);

			if (pixel_gray >= 0 && pixel_gray <= 5)
			{
				img_diff_color.at<cv::Vec3b>(v, u) = cv::Vec3b(0, 0, 0);
			}
			else
			{
				// use color for ground color extraction
				int b = img_src_color_.at<cv::Vec3b>(v, u)[0];
				int g = img_src_color_.at<cv::Vec3b>(v, u)[1];
				int r = img_src_color_.at<cv::Vec3b>(v, u)[2];

				// Denoising: if the bgr values meet threshold, the pixel is considered as ground
				if(b <= 60 && g <= 60 && r <= 60)
					img_diff_color.at<cv::Vec3b>(v, u) = cv::Vec3b(0, 0, 0);
				else
					img_diff_color.at<cv::Vec3b>(v, u) = img_src_color_.at<cv::Vec3b>(v, u);	// color
			}
		}
	}
	cv::imshow("img_diff_color", img_diff_color);
	
	// output diff image
	if(!img_diff_color.empty())
		img_obj_th_ = img_diff_color.clone();
}


/****************************************************************************************
 *	@Function
 *		extract object color range(HSV) from the result of 
 * 		backgroundSubtraction()
****************************************************************************************/
void FbControl::objectColorExtraction()
{
	if(!img_obj_th_.empty())
	{
		// make binary image
		cv::Mat img_gray, img_bin;
    	cv::cvtColor(img_obj_th_, img_gray, cv::COLOR_BGR2GRAY);
		cv::threshold(img_gray, img_bin, 1, 255, cv::THRESH_BINARY);

		// find connected region
		cv::Mat labels, stats, centroids;
    	int num_labels = cv::connectedComponentsWithStats(img_bin, labels, stats, centroids);

		// find the largest region
		int max_area = 0, max_label = 0;
		for (int i = 1; i < num_labels; i++) 
		{
			int area = stats.at<int>(i, cv::CC_STAT_AREA);
			if (area > max_area) 
			{
				max_area = area;
				max_label = i;
			}
		}

		// create a mask of the largest region
		cv::Mat largest_region_mask = (labels == max_label);

		// convert color space of the result of backgroundSubtraction()
		cv::Mat img_hsv;
		cv::cvtColor(img_obj_th_, img_hsv, cv::COLOR_BGR2HSV);

		// split to each channel
		std::vector<cv::Mat> hsv_channels;
    	cv::split(img_hsv, hsv_channels);

		// find HSV range
		double h_min, h_max, s_min, s_max, v_min, v_max;
		cv::minMaxLoc(hsv_channels[0], &h_min, &h_max, nullptr, nullptr, largest_region_mask);
		cv::minMaxLoc(hsv_channels[1], &s_min, &s_max, nullptr, nullptr, largest_region_mask);
		cv::minMaxLoc(hsv_channels[2], &v_min, &v_max, nullptr, nullptr, largest_region_mask);

		// cast value
		H_MIN_OBJ_ = static_cast<int>(h_min);	H_MAX_OBJ_ = static_cast<int>(h_max);
		S_MIN_OBJ_ = static_cast<int>(s_min);	S_MAX_OBJ_ = static_cast<int>(s_max);
		V_MIN_OBJ_ = 100;	V_MAX_OBJ_ = static_cast<int>(v_max); 

		std::cout << "H_MIN_OBJ: " << H_MIN_OBJ_ << ", H_MAX_OBJ: " << H_MAX_OBJ_ << std::endl;
		std::cout << "S_MIN_OBJ: " << S_MIN_OBJ_ << ", S_MAX_OBJ: " << S_MAX_OBJ_ << std::endl;
		std::cout << "V_MIN_OBJ: " << V_MIN_OBJ_ << ", V_MAX_OBJ: " << V_MAX_OBJ_ << std::endl;
	}
	else
		ROS_ERROR("Failed to extract object color");
}