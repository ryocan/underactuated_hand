//-----------------------------------------------------
// INCLUDE FILES
//-----------------------------------------------------
#include "img_proc/fb_control.hpp"

/****************************************************************************************
 *	@Function
 *		[Main function for contact detection] detect contact between obj and hand
****************************************************************************************/
bool FbControl::contactDetection()
{
	// hand and obj detection
	cv::Mat img_mask_obj = extractRegion(H_MIN_OBJ_, H_MAX_OBJ_, S_MIN_OBJ_, S_MAX_OBJ_, V_MIN_OBJ_, V_MAX_OBJ_);
	cv::Mat img_mask_hand = extractRegion(H_MIN_HAND_, H_MAX_HAND_, S_MIN_HAND_, S_MAX_HAND_, V_MIN_HAND_, V_MAX_HAND_);

	// calc contours
	std::vector<std::vector<cv::Point>> contours_obj, contours_hand; 
	contours_obj = calcContours(img_mask_obj, mode_obj);
	contours_hand = calcContours(img_mask_hand, mode_hand);

	// calc centroid
	centroid_obj_ = calcCentroid(contours_obj, mode_obj);
    centroid_hand_ = calcCentroid(contours_hand, mode_hand);

    // draw contours
	img_black_ = cv::Mat::zeros(img_src_color_.size(), img_src_color_.type());
    drawContours(contours_obj, img_mask_obj, "obj");
    drawContours(contours_hand, img_mask_hand, "hand");

    // display image
    cv::imshow("Contact Detection", img_black_);

	// judge
    if ((contact_hand_l_ > 20) && (contact_hand_r_ > 20))
    {
        cv::destroyWindow("Contact Detection");
        return true;
    }

	return false;
}