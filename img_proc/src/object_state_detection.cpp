//-----------------------------------------------------
// INCLUDE FILES
//-----------------------------------------------------
#include "img_proc/fb_control.hpp"

/****************************************************************************************
 *	@Function
 *		[Main function for object state detection] detect deformation and so on
****************************************************************************************/
bool FbControl::objectStateDetection()
{
	// // IfG1: robotic hand detection
	// if(handMoveDetection())
	// 	return true;

	// IfG2: object's v-direction movement
	if(objMoveDetection())
		return true;

	// IfG3: deformation detection
	if(deformationDetection())
		return true;
	
	// IfG4: occlusion detection
	if(handOcclusionDetection())
		return true;

	// count for detection IfG1 and IfG4
	count_frame_++;

	return false;
}

/****************************************************************************************
 *	@Function
 *		detect movement of robotic hand
****************************************************************************************/
bool FbControl::handMoveDetection()
{
	// detect hand region
	cv::Mat img_mask_hand = extractRegion(H_MIN_HAND_, H_MAX_HAND_, S_MIN_HAND_, S_MAX_HAND_, V_MIN_HAND_, V_MAX_HAND_);

	// get hand contour
	std::vector<std::vector<cv::Point>> contours_hand = calcContours(img_mask_hand, "hand");
    
	// get hand centroid
	centroid_hand_ = calcCentroid(contours_hand, "hand");

	if (!centroid_hand_prev_.empty())
    {
        // calculate hand movement
        double hand_r_shift = (centroid_hand_prev_[0].x - centroid_hand_[0].x) + (centroid_hand_prev_[0].y - centroid_hand_[0].y);
        double hand_l_shift = (centroid_hand_prev_[1].x - centroid_hand_[1].x) + (centroid_hand_prev_[1].y - centroid_hand_[1].y);

        // detection
		double shift = 5.0;
        if ((hand_r_shift <= shift) && (hand_l_shift <= shift) && (count_frame_ >= 30))
        {
            ROS_INFO("Grasp Detection (IfG1)");
            return true;
        }
    }

	// update info
	centroid_hand_prev_.clear();
	for(int i = 0; i < centroid_hand_.size(); i++)
		centroid_hand_prev_.push_back(centroid_hand_[i]);

	return false;
}

/****************************************************************************************
 *	@Function
 *		object's v-direction movement
****************************************************************************************/
bool FbControl::objMoveDetection()
{
	// detect object region
	cv::Mat img_mask_obj = extractRegion(H_MIN_OBJ_, H_MAX_OBJ_, S_MIN_OBJ_, S_MAX_OBJ_, V_MIN_OBJ_, V_MAX_OBJ_);

	// calc largest contours
	std::vector<std::vector<cv::Point>> contours_obj = calcContours(img_mask_obj, mode_obj);

	// get obj centroid
    centroid_obj_ = calcCentroid(contours_obj, "obj");

    if (!centroid_obj_prev_.empty())
    {
        // calc shift
        double centroid_obj_v_shift = centroid_obj_prev_[0].y - centroid_obj_[0].y;

		int threshold_shift = 5;  // pixel
        if (centroid_obj_v_shift >= threshold_shift)
        {
            ROS_INFO("Grasp Detection (IfG2)");
            return true;
        }
    }

	// update info
	centroid_obj_prev_.clear();
	centroid_obj_prev_.push_back(centroid_obj_[0]);

	return false;	
}

/****************************************************************************************
 *	@Function
 *		detect 3d deformation using point cloud
****************************************************************************************/
bool FbControl::deformationDetection()
{
	// detect object region
	cv::Mat img_mask_obj = extractRegion(H_MIN_OBJ_, H_MAX_OBJ_, S_MIN_OBJ_, S_MAX_OBJ_, V_MIN_OBJ_, V_MAX_OBJ_);

	// calc largest contours
	std::vector<std::vector<cv::Point>> contours_obj = calcContours(img_mask_obj, mode_obj);

	// mask image for only object region
	cv::Mat img_mask_obj_largest = cv::Mat::zeros(img_src_color_.size(), CV_8UC1);
	cv::drawContours(img_mask_obj_largest, contours_obj, -1, cv::Scalar(255), cv::FILLED);

	// only the object image
	cv::Mat img_obj_region;
	img_src_color_.copyTo(img_obj_region, img_mask_obj_largest);

	// convert color space for optical flow
	cv::Mat img_obj_gray;
	cv::cvtColor(img_obj_region, img_obj_gray, CV_BGR2GRAY);

	// calculate centroid of object which is used for deformation calculation
	centroid_obj_ = calcCentroid(contours_obj, mode_obj);

	if(!point_optflow_2d_prev_.empty())
	{
		// calculate LK-Optical flow
		std::vector<cv::Point2f> point_optflow_2d;
		std::vector<uchar> status;
    	std::vector<float> err;
    	cv::TermCriteria criteria = cv::TermCriteria((cv::TermCriteria::COUNT)+(cv::TermCriteria::EPS), 10, 0.03);
		cv::calcOpticalFlowPyrLK(img_obj_gray_prev_, img_obj_gray, point_optflow_2d_prev_, point_optflow_2d, status, err, cv::Size(20, 20), 2, criteria);

		// calculate deformation
		double deform_l_x = 0.0, deform_r_x = 0.0;
		double deform_l_y = 0.0, deform_r_y = 0.0;
		double deform_l_z = 0.0, deform_r_z = 0.0;
		for(int i = 0; i < point_optflow_2d.size(); i++)
		{
			// calculate diff
			double x_2d = point_optflow_2d[i].x - point_optflow_2d_prev_[i].x;
			double y_2d = point_optflow_2d[i].y - point_optflow_2d_prev_[i].y;

			if(abs(x_2d) <= 1.0 && abs(y_2d) <= 1.0)	// exclude outlier
			{
				// get 3d info
				cv::Point3f point_optflow_3d = calcPoint3d(point_optflow_2d[i]);

				// calculate 3d diff
				double x_3d = point_optflow_3d.x - point_optflow_3d_prev_[i].x;
				double y_3d = point_optflow_3d.y - point_optflow_3d_prev_[i].y;
				double z_3d = point_optflow_3d.z - point_optflow_3d_prev_[i].z;

				// calc 3d diff for each finger
				if(point_optflow_2d[i].x < centroid_obj_[0].x)
				{
					deform_l_x = x_3d;
					deform_l_y = y_3d;
					deform_l_z = z_3d;	
				}
				else
				{
					deform_r_x = x_3d;
					deform_r_y = y_3d;
					deform_r_z = z_3d;	
				}
				
				// check object move
				double obj_move_x = abs(centroid_obj_[0].x - centroid_obj_prev_[0].x);

				// calc 3d deformation
				int obj_move_th = 2; //pixel
				if (obj_move_x < obj_move_th)
				{
					double deform_l_total = std::sqrt(std::pow(deform_l_x, 2) + std::pow(deform_l_y, 2) + std::pow(deform_l_z, 2));
					double deform_r_total = std::sqrt(std::pow(deform_r_x, 2) + std::pow(deform_r_y, 2) + std::pow(deform_r_z, 2));
					deformation_total_3d_ += (deform_l_total + deform_r_total);
				}
				else
					deformation_total_3d_ += 0.0;	// object move means object is not deforming

				// normalize by the size of the object
				int deformation_ratio = deformation_total_3d_ / point_optflow_2d_prev_.size();
				if( deformation_ratio > DEFORMATION_TH_ )
				{
					ROS_INFO("Grasp Detection (IfG3)");
					return true;
				}
			}
		}
	}

	// update gray image
	img_obj_gray_prev_ = img_obj_gray.clone();

	// update point data: contours + random points within the object region
	point_optflow_2d_prev_.clear();
	for(int i = 0; i < contours_obj[0].size(); i++)
		point_optflow_2d_prev_.push_back(static_cast<cv::Point2f>(contours_obj[0][i]));
	generateRandomPointsInContours(img_mask_obj_largest);

	// calc 3d info
	for(int i = 0; i < point_optflow_2d_prev_.size(); i++)
	{
		cv::Point3f point = calcPoint3d(point_optflow_2d_prev_[i]);
		point_optflow_3d_prev_.push_back(point);
	}

	// update centroid
	centroid_obj_prev_.clear();
	centroid_obj_prev_.push_back(centroid_obj_[0]);

	// debug
	cv::Mat m = img_obj_gray.clone();
	for (int i = 0; i < point_optflow_2d_prev_.size(); i++) {
		cv::Point pointTest;
		pointTest.x = static_cast<int>(point_optflow_2d_prev_[i].x);
		pointTest.y = static_cast<int>(point_optflow_2d_prev_[i].y);
        cv::circle(m, pointTest, 2, cv::Scalar(255, 0, 0), -1);
    }
	imshow("m", m);

	return false;
}


/****************************************************************************************
 *	@Function
 *		generate random point for optical flow
****************************************************************************************/
void FbControl::generateRandomPointsInContours(cv::Mat img_mask)
{
    // Extract white regions (mask is 8-bit single channel, white is 255)
    std::vector<cv::Point> white_points;
    cv::findNonZero(img_mask, white_points);  // Get coordinates of white region

    if (white_points.empty())
    {
        std::cerr << "No white region found in the mask." << std::endl;
        return;
    }

    // Randomly select points from the white region
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, white_points.size() - 1);

    // Generate the specified number of random points and draw them on the image
    int numPoints = 50;
    for (int i = 0; i < numPoints; ++i)
    {
        int random_index = dis(gen);  // Generate a random index
        cv::Point2f point = white_points[random_index];  // Select a random point from the white region

        point_optflow_2d_prev_.push_back(point);
    }
}

/****************************************************************************************
 *	@Function
 *		generate pointcloud
****************************************************************************************/
cv::Point3f FbControl::calcPoint3d(cv::Point2f point_input)
{
	const float depth_scale = 0.01f;
	const float fx = 616.0f; 
	const float fy = 616.0f;
	const float cx = 320.0f; // x-coordinate of the image center
	const float cy = 240.0f; // y-coordinate of the image center

	// get depth
	uint16_t depth_value = img_src_depth_.at<uint16_t>(point_input.y, point_input.x);

	// calc xyz
	cv::Point3f point;
	point.z = depth_value * depth_scale;
	point.x = (point_input.x - cx) * point.z / fx;
	point.y = (point_input.y - cy) * point.z / fy;

	return point;
}

/****************************************************************************************
 *	@Function
 *		
****************************************************************************************/
bool FbControl::handOcclusionDetection()
{
	// detect hand and object region
	cv::Mat img_mask_hand = extractRegion(H_MIN_HAND_, H_MAX_HAND_, S_MIN_HAND_, S_MAX_HAND_, V_MIN_HAND_, V_MAX_HAND_);
	cv::Mat img_mask_obj = extractRegion(H_MIN_OBJ_, H_MAX_OBJ_, S_MIN_OBJ_, S_MAX_OBJ_, V_MIN_OBJ_, V_MAX_OBJ_);

	// get largest contour
	std::vector<std::vector<cv::Point>> contours_hand = calcContours(img_mask_hand, "hand");
	std::vector<std::vector<cv::Point>> contours_obj = calcContours(img_mask_obj, mode_obj);

	// total both hand's area
	double hand_area = cv::contourArea(contours_hand[0]) + cv::contourArea(contours_hand[1]);

    // initialize area as a standard
    if (count_frame_ == 0)
        hand_area_init_ = hand_area;
    
    // calc occlusion
    double occlusion_area = hand_area_init_ - hand_area;
    double occlusion_rate = occlusion_area / contours_obj[0].size();

    if (occlusion_rate > OCCLUSION_TH_)
    {
        ROS_INFO("grasp detected(IfG4).");
        return true;
    }

	return false;
}