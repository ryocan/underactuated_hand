//-----------------------------------------------------
// INCLUDE FILES
//-----------------------------------------------------
#include "img_proc/common_proc.hpp"

//------------------------------------------------------------
// FUNCTIONS
//------------------------------------------------------------
/*************************************************************
 * Function:     subscribe 2D color image
 * Comment:
**************************************************************/
void CommonProc::imgColorCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        img_src_color_ = cv_ptr->image.clone();
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Failed to read img_src_color_: %s", e.what());
        return;
    }
}

/*************************************************************
 * Function:     subscribe depth image
 * Comment:
**************************************************************/
void CommonProc::imgDepthCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        img_src_depth_ = cv_ptr->image.clone();
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Failed to read img_src_depth_: %s", e.what());
        return;
    }
}

/****************************************************************************************
 * @Function
 *    extract region based on its color
****************************************************************************************/
cv::Mat CommonProc::extractRegion(int H_MIN, int H_MAX, int S_MIN, int S_MAX, int V_MIN, int V_MAX)
{
    cv::Mat img_mask = cv::Mat::zeros(img_src_color_.size(), img_src_color_.type());

    // convert to HSV image
    cv::Mat img_hsv;
    cvtColor(img_src_color_, img_hsv, cv::COLOR_BGR2HSV);

    // extract region based on HSV parameter
    cv::Scalar Lower(H_MIN, S_MIN, V_MIN);
    cv::Scalar Upper(H_MAX, S_MAX, V_MAX);
    cv::inRange(img_hsv, Lower, Upper, img_mask);

    // noise handling
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(img_mask, img_mask, cv::MORPH_OPEN, kernel);

    return img_mask;
}

/****************************************************************************************
 * @Function
 *    calculation contour from mask image
****************************************************************************************/
std::vector<std::vector<cv::Point>> CommonProc::calcContours(cv::Mat img_mask, std::string mode)
{
    std::vector<std::vector<cv::Point>> contours_output;

    if(!img_mask.empty())
    {
        // findContours from img_mask
        std::vector<std::vector<cv::Point>> contours; 
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(img_mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // detect based on area info
        double area = 0.;


        // change contours calculation method
        if (mode == "obj")
        {
            // Assuming there is only one object, extract only the object with the largest area.
            double area_prev = 0.;
            for (int i = 0; i < contours.size(); i++)
            {
                area = cv::contourArea(contours[i]);
                if (area > area_prev)
                {
                    contours_output.clear();
                    contours_output.shrink_to_fit();
                    contours_output.push_back(contours[i]);
                    area_prev = area;
                }
            }
        } 
        else if (mode == "hand")
        {
            // extract based on size of hand
            double area_th = 1000.; //15000
            for (int i = 0; i < contours.size(); i++)
            {
                area = cv::contourArea(contours[i]);
                if (area > area_th)
                    contours_output.push_back(contours[i]);
            }
        }
    }
    else
    {
        contours_output[0].push_back(cv::Point(0, 0));
    }

    return contours_output;
}


/****************************************************************************************
 * @Function
 *    calculate centroid 
****************************************************************************************/
std::vector<cv::Point> CommonProc::calcCentroid(std::vector<std::vector<cv::Point>> contours, std::string mode)
{
    std::vector<cv::Point> centroid_output;

    // calc mu
    std::vector<cv::Moments> mu;
    for (int i = 0; i < contours.size(); i++)
        mu.push_back(moments(contours[i], false));
    
    // display color setting
    cv::Scalar color;
    if (mode == "obj")
        color = cv::Scalar(0, 0, 255);
    else if (mode == "hand")
        color = cv::Scalar(255, 0, 0);

    // calc mc    
    std::vector<cv::Point2f> mc(contours.size());
    for (int i = 0; i < mu.size(); i++)
    {
        mc[i] = cv::Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);

        if (std::isnan(mu[i].m10 / mu[i].m00) != true && std::isnan(mu[i].m01 / mu[i].m00) != true)
        {
            centroid_output.push_back(mc[i]);
        }
    }

    return centroid_output;
}


/****************************************************************************************
 * @Function
 *    draw contours using Line Iterator for contact detection 
****************************************************************************************/
void CommonProc::drawContours(std::vector<std::vector<cv::Point>> contours, cv::Mat img_mask, std::string mode)
{
    // declare for LineIterator
    cv::Point li_start;
    cv::Point li_goal;
    int line_width = 12;
    
    for (int i = 0; i < contours.size(); i++)
    {
        for (int j = 0; j < contours[i].size(); j++)
        {
            // specify start and end point for cv::LineIterator
            li_start = contours[i][j];

            if (j == contours[i].size() - 1)
                li_goal = contours[i][0];
            else    
                li_goal = contours[i][j + 1];

            // using cv::LineIterator
            cv::LineIterator LI(img_mask, li_start, li_goal, 8, false);
        
            // get point on the line
            std::vector<cv::Point> li_point(LI.count);
            for (int l = 0; l < LI.count; l++, ++LI)
                li_point[l] = LI.pos();

            // draw
            for (int k = 0; k < li_point.size(); k++)
            {
                // based on the object's centroid, change the direction to make the line thicker
                int sign_x = 1;
                if (li_point[k].x > centroid_obj_[0].x) 
                    sign_x = -1;
                
                for (int n = 0; n <= line_width; n++)
                {
                    if (mode == "obj")
                        img_black_.at<cv::Vec3b>(li_point[k].y, li_point[k].x + sign_x * n) = cv::Vec3b(0, 0, 255);
                    else if (mode == "hand")
                        img_black_.at<cv::Vec3b>(li_point[k].y, li_point[k].x + sign_x * n)[0] += 255; //overwrite on Hand's line


                    if ((img_black_.at<cv::Vec3b>(li_point[k].y, li_point[k].x + sign_x * n)) == cv::Vec3b(255, 0, 255))
                    {
                        if (li_point[k].x < centroid_obj_[0].x) // to judge which hand is touched
                            contact_hand_l_++;
                        else 
                            contact_hand_r_++;
                    }
                }
            }
        }
    }
}