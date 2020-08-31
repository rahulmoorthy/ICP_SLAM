//
// Created by rakesh on 17/08/18.
//

#include <icp_slam/utils.h>
#include <iostream>
using namespace std;
namespace icp_slam
{
namespace utils
{

cv::Mat laserScanToPointMat(const sensor_msgs::LaserScanConstPtr &scan)
{
  // TODO
//Creating a Nx2 matrix for laser data
float x=(scan->angle_max-scan->angle_min)/scan->angle_increment;
cv::Mat las_mat((x+1),2,CV_32F);

for (int i=0;i<=x;i++)
{ 
float angle=scan->angle_min+i*scan->angle_increment;
float R=scan->ranges[i];
icp_slam::utils::polarToCartesian(R,angle,las_mat.at<float>(i,0),las_mat.at<float>(i,1));
//ROS_INFO_THROTTLE(1.0,"polar i:%d ,  x: %f, y: %f",i,las_mat.at<float>(i,0),las_mat.at<float>(i,1));
}


return las_mat;

}

cv::Mat transformPointMat(tf::Transform transform, cv::Mat &point_mat)
{
  assert(point_mat.data);
  assert(!point_mat.empty());

  cv::Mat point_mat_homogeneous(3, point_mat.rows, CV_32F, cv::Scalar(1.0f));

  cv::Mat(point_mat.t()).copyTo(point_mat_homogeneous.rowRange(0, 2));

  auto T = transformToMatrix(transform);
  cv::Mat transformed_point_mat =  T * point_mat_homogeneous;
  return cv::Mat(transformed_point_mat.t()).colRange(0, 2);
}

} // namespace utils
} // namespace icp_slam
