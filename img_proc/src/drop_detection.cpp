//-----------------------------------------------------
// INCLUDE FILES
//-----------------------------------------------------
#include "img_proc/fb_control.hpp"

/****************************************************************************************
 *	@Function
 *		[Main function for drop detection] detect drop and rotation
****************************************************************************************/
bool FbControl::dropDetection()
{
	int pos_size = arm_traj_point_.size();

	// detect hand centorid
	cv::Mat img_mask_hand = extractRegion(H_MIN_HAND_, H_MAX_HAND_, S_MIN_HAND_, S_MAX_HAND_, V_MIN_HAND_, V_MAX_HAND_);
	std::vector<std::vector<cv::Point>> contours_hand = calcContours(img_mask_hand, "hand");
	centroid_hand_ = calcCentroid(contours_hand, "hand");

	// detect obj centroid
	cv::Mat img_mask_obj = extractRegion(H_MIN_OBJ_, H_MAX_OBJ_, S_MIN_OBJ_, S_MAX_OBJ_, 50, V_MAX_OBJ_);
	std::vector<std::vector<cv::Point>> contours_obj = calcContours(img_mask_obj, mode_obj);
	centroid_obj_ = calcCentroid(contours_obj, "obj");
	imshow("img_mask_obj", img_mask_obj);

	// -------- droppage detection ------------------------------------------
	// calc average centroid point of two hands
	cv::Point centroid_hand_average;
	centroid_hand_average.x = (centroid_hand_[0].x + centroid_hand_[1].x) / 2;
	centroid_hand_average.y = (centroid_hand_[0].y + centroid_hand_[1].y) / 2;

	// calc distance of two point and droppage
	double distance_centroid, distance_drop;
	if(frame_current_ == 0)
		distance_centroid_std_ = centroid_obj_[0].y - centroid_hand_average.y;
	else
	{
		distance_centroid = centroid_obj_[0].y - centroid_hand_average.y;
		distance_drop = distance_centroid - distance_centroid_std_;
		std::cout << "distance_drop: " << distance_drop << std::endl;
	}

	// --------- PCA -----------------------------------------------------------
	cv::Mat data_points = cv::Mat(contours_obj[0].size(), 2, CV_64F);

    for (size_t i = 0; i < contours_obj[0].size(); i++) 
	{
        data_points.at<double>(i, 0) = contours_obj[0][i].x;
        data_points.at<double>(i, 1) = contours_obj[0][i].y;
    }

	// Perform PCA on the contour points
    cv::PCA pca_analysis(data_points, cv::Mat(), cv::PCA::DATA_AS_ROW);

    // The eigenvectors are sorted by eigenvalue, so the first eigenvector is the first principal component
    cv::Point2d first_eigenvector(pca_analysis.eigenvectors.at<double>(0, 0),pca_analysis.eigenvectors.at<double>(0, 1));

    // Calculate the angle of the first principal component (in radians)
    double angle = atan2(first_eigenvector.y, first_eigenvector.x);

    // Convert radians to degrees for easier interpretation
    double angle_degrees = angle * 180.0 / CV_PI;
	std::cout << "angle_degrees: " << angle_degrees << std::endl;

	// -------- common procceding ------------------------------------------
	// adjust grasping force as time. 10[frame] and 100[frame] is threshold 
	int time_grasp = 0;
	if(frame_current_ == 0)
		time_grasp = 10;
	else
		time_grasp = 100 / (frame_current_ - frame_drop_);

	// regrasping
	if((frame_current_ > 5) && (distance_drop > 15))
	{
		ROS_INFO("REGRASPING");

		dynamixelCommandMsg("", 1, "Goal_Velocity", DYNAMIXEL_GOAL_VELOCITY_);
		dynamixelCommandMsg("", 2, "Goal_Velocity", DYNAMIXEL_GOAL_VELOCITY_);
		sleep(time_grasp);

		dynamixelCommandMsg("", 1, "Goal_Velocity", 0);
		dynamixelCommandMsg("", 2, "Goal_Velocity", 0);

		// initialize
		distance_centroid_std_ = distance_centroid;
		frame_drop_ = frame_current_;
	}

	// debug image
	cv::Mat m = img_src_color_.clone();
	cv::line(m, centroid_hand_average, centroid_obj_[0], cv::Scalar(255, 255, 255), 8, 8, 0);
	cv::line(m, centroid_hand_average, centroid_obj_[0], cv::Scalar(  0,   0,   0), 4, 8, 0);

    cv::Point2d endpoint(centroid_obj_[0].x + 10 * first_eigenvector.x, centroid_obj_[0].y + 10 * first_eigenvector.y);
	cv::line(m, centroid_obj_[0], endpoint, cv::Scalar(255, 255, 255), 8, 8, 0);
	cv::line(m, centroid_obj_[0], endpoint, cv::Scalar(255, 0, 0), 8, 8, 0);
	imshow("drop detection", m);

	// update
	frame_current_++;
	if(img_mask_hand.empty() || img_mask_obj.empty())
	{
		flag_entire_process_ = 7;
		return true;
	}
	else if(count_ < 310)
	{
		flag_entire_process_ = 6;
		count_++;
		return true;
	}
	else if(control_fin_ == pos_size)
	{
		ROS_INFO("Dobot Control FIN");
		flag_entire_process_ = 7;
		count_ = 0;
		return true;
	}
	else if(pos_num_ == control_fin_)
	{
		ROS_INFO("Dobot Control CONTINUE");
		flag_entire_process_ = 5;
		count_ = 0;
		return true;
	}

	return false;
}