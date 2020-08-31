//
// Created by rakesh on 13/08/18.
//
#include <cmath>
#include <map>
#include <numeric>
#include <chrono>
#include <iostream>
#include <string> 
#include <boost/atomic/atomic.hpp>
#include <opencv2/flann/miniflann.hpp>

#include <icp_slam/icp_slam.h>
#include <icp_slam/utils.h>
#include <icp_slam/config.h>

#define TIME_DIFF(tic, toc) ((std::chrono::duration<double, std::milli>((toc) - (tic))).count())

namespace icp_slam
{

ICPSlam::ICPSlam(tfScalar max_keyframes_distance, tfScalar max_keyframes_angle, double max_keyframes_time)
  : max_keyframes_distance_(max_keyframes_distance),
    max_keyframes_angle_(max_keyframes_angle),
    max_keyframes_time_(max_keyframes_time),
    last_kf_laser_scan_(new sensor_msgs::LaserScan()),
    is_tracker_running_(false)
{
  last_kf_tf_odom_laser_.stamp_ = ros::Time(0);
   count_kf=0; // for keeping track of keyframes
}

bool ICPSlam::track(const sensor_msgs::LaserScanConstPtr &laser_scan,
                    const tf::StampedTransform &current_frame_tf_odom_laser,
                    tf::StampedTransform &tf_map_laser)
{


  if (is_tracker_running_)
  {
    ROS_WARN_THROTTLE(1.0, "Couldn't track frame, tracker already running");
    return false;
  }
  // TODO: find the pose of laser in map frame
  if (count_kf==0) // For the first frame, the odometry update is only taken
    {
   ROS_WARN_THROTTLE(1.0,"Inside the First Key frame");
   *last_kf_laser_scan_=*laser_scan;
   last_kf_tf_odom_laser_=current_frame_tf_odom_laser;
}
  if(ICPSlam::isCreateKeyframe(current_frame_tf_odom_laser,last_kf_tf_odom_laser_)|| (count_kf==0)) 
{    

//Estimate E21 
ROS_ERROR("Creating key frame %d",count_kf);
//Using pose geometry
tf::Transform poseDiff = last_kf_tf_odom_laser_.inverse() * current_frame_tf_odom_laser;

tf::Transform tf_pose=ICPSlam::icpRegistration(last_kf_laser_scan_,laser_scan,poseDiff,count_kf);
tf::Transform tf_odom_laser=last_kf_tf_odom_laser_* tf_pose; //Converting the relative data wrt last key frame, back to relative to odom
tf_map_laser.setData(tf_odom_laser);
tf_map_laser.child_frame_id_="/odom";
tf_map_laser.frame_id_="/map";
tf_map_laser.stamp_=ros::Time::now();

last_kf_tf_odom_laser_.stamp_ = current_frame_tf_odom_laser.stamp_;
last_kf_tf_odom_laser_=current_frame_tf_odom_laser;
*last_kf_laser_scan_=*laser_scan;
count_kf++;
return true;

}
else
{
//sending the odometry update wrt last keyframe
tf::Transform poseDiff=last_kf_tf_odom_laser_.inverse() * current_frame_tf_odom_laser;
tf_map_laser.setData(poseDiff);
return false; 
}

  
// if a new keyframe is created, run ICP
  // if not a keyframe, obtain the laser pose in map frame based on odometry update
}
//Function to calculate error

float error_calc(cv::Mat x,cv::Mat p)
{
  float err=0.0;
  for(int i=0;i<x.rows;i++)
  {
    err+=pow(pow(x.at<float>(i,0)-p.at<float>(i,0),2)+pow(x.at<float>(i,1)-p.at<float>(i,1),2),0.5);
  }
  err=err/x.rows;
  return err;
}

tf::Transform ICPSlam::icpRegistration(const sensor_msgs::LaserScanConstPtr &laser_scan1,
                                       const sensor_msgs::LaserScanConstPtr &laser_scan2,
                                       const tf::Transform &T_2_1,int kf_id)
{

//1. Convert to cv::mat
cv::Mat last_mat=icp_slam::utils::laserScanToPointMat(laser_scan1);
cv::Mat curr_mat=icp_slam::utils::laserScanToPointMat(laser_scan2);
int count=0;
ICPSlam::vizClosestPoints(last_mat,curr_mat,T_2_1,kf_id,count);
tf::Transform T_scan2_scan1;
cv::Mat updated_curr=icp_slam::utils::transformPointMat(T_2_1,curr_mat); //convert current data
auto pose =T_2_1;
float old_err,err;
err=error_calc(last_mat,updated_curr);
//2. run ICP (SVD) iterations till converges or 10 times
do
{ 
  count++;
  old_err=err;
  std::cout<<"Error:"<<old_err;
T_scan2_scan1 = ICPSlam::icpIteration(last_mat,curr_mat,pose,kf_id,count);
pose=T_scan2_scan1;
updated_curr=icp_slam::utils::transformPointMat(T_scan2_scan1,curr_mat);
err=error_calc(last_mat,updated_curr);

}while(count<10 && (old_err-err>0.0001)); // Comparing old error and new error, and if similar-> transforms have converged
//Refine estimate
//Run again till no significant change
return pose;

}

tf::Transform ICPSlam::icpIteration(cv::Mat &point_mat1,
                                    cv::Mat &point_mat2, tf::Transform &T_2_1,int kf_id,int iter)
{
cv::Mat last_mat,curr_mat;
point_mat1.copyTo(last_mat);
point_mat2.copyTo(curr_mat);

std::vector<int> closest_indices;
std::vector<float> closest_distances_2;

auto updated_curr=icp_slam::utils::transformPointMat(T_2_1,curr_mat); // VVVVVVVImp step

ICPSlam::closestPoints(last_mat,updated_curr,closest_indices,closest_distances_2);
//sort closest points corr depending on distances, then trim
float mean,std_dev;
icp_slam::utils::meanAndStdDev(closest_distances_2,mean,std_dev);
float max=mean+2*std_dev;
cv::Mat curr_re((closest_indices.size()),2,CV_32F);
//Reordering matrices according to closest indices
for(int i=0;i<closest_indices.size();i++)
{
curr_re.at<float>(i,0)=curr_mat.at<float>(closest_indices[i],0);
curr_re.at<float>(i,1)=curr_mat.at<float>(closest_indices[i],1);
}

cv::Mat x(0,2,CV_32F);
cv::Mat p(0,2,CV_32F);
//Trimming matrices
for (int i=0;i<closest_distances_2.size();i++)
{
if(closest_distances_2[i]<=max)
{
  x.push_back(last_mat.row(i));
  p.push_back(curr_re.row(i));
}
}

//Visualizing after trimming
ICPSlam::vizClosestPoints(x,p,T_2_1,kf_id,iter);

std::vector<float> xxi;
x.col(0).copyTo(xxi);
std::vector<float> xyi;
x.col(1).copyTo(xyi);
std::vector<float> pxi;
p.col(0).copyTo(pxi);
std::vector<float> pyi;
p.col(1).copyTo(pyi);

float xxmean,xymean,pxmean,pymean,std_xp;
//Calculating Center of mass
icp_slam::utils::meanAndStdDev(xxi,xxmean,std_xp);
icp_slam::utils::meanAndStdDev(xyi,xymean,std_xp);
icp_slam::utils::meanAndStdDev(pxi,pxmean,std_xp);
icp_slam::utils::meanAndStdDev(pyi,pymean,std_xp);
for(int i=0;i<x.rows;i++)
{
x.at<float>(i,0)=x.at<float>(i,0)-xxmean;
x.at<float>(i,1)=x.at<float>(i,1)-xymean;
p.at<float>(i,0)=p.at<float>(i,0)-pxmean;
p.at<float>(i,1)=p.at<float>(i,1)-pymean;
}


cv::Mat ux(2,1,CV_32F);
cv::Mat up(2,1,CV_32F);
ux.at<float>(0,0)=xxmean;
ux.at<float>(1,0)=xymean;
up.at<float>(0,0)=pxmean;
up.at<float>(1,0)=pymean;

cv::Scalar s=0.0;
cv::Mat W = cv::Mat::zeros(2,2,CV_32F);
//Calculating W matrix
W=x.t()*p;
//SVD
cv::SVD svd(W);

cv::Mat R(2,2,CV_32F);
cv::Mat t(2,1,CV_32F);
tf::Vector3 t1;
R=svd.u*svd.vt;
t=ux-R*up;
//Rotation and translation obtained using W matrix
t1.setX(t.at<float>(0,0));
t1.setY(t.at<float>(1,0));
	
tf::Transform new_pose;

float rotation=atan2(R.at<float>(1,0),R.at<float>(0,0));
new_pose.setOrigin(t1);
new_pose.setRotation(tf::createQuaternionFromYaw(rotation));
//New pose obtained
return new_pose;
}



bool ICPSlam::isCreateKeyframe(const tf::StampedTransform &current_frame_tf, const tf::StampedTransform &last_kf_tf) const
{
  assert(current_frame_tf.frame_id_ == last_kf_tf.frame_id_);
  assert(current_frame_tf.child_frame_id_ == last_kf_tf.child_frame_id_);
  // TODO: check whether you want to create keyframe (based on max_keyframes_distance_, max_keyframes_angle_, max_keyframes_time_)
  auto last=last_kf_tf.getOrigin();
  auto curr=current_frame_tf.getOrigin();
  float dist=last.distance2(curr);

  if(dist>max_keyframes_distance_)
   return true;

  float r1=tf::getYaw(last_kf_tf.getRotation());
  float r2=tf::getYaw(current_frame_tf.getRotation());

  if(r2-r1>max_keyframes_angle_)
   return true;

  auto k=(current_frame_tf.stamp_ - last_kf_tf.stamp_).toSec();

if (k > max_keyframes_time_)
   return true;
 
  return false;


}

void ICPSlam::closestPoints(cv::Mat &point_mat1,
                            cv::Mat &point_mat2,
                            std::vector<int> &closest_indices,
                            std::vector<float> &closest_distances_2)
{
  // uses FLANN for K-NN e.g. http://www.morethantechnical.com/2010/06/06/iterative-closest-point-icp-with-opencv-w-code/
  closest_indices = std::vector<int>(point_mat1.rows, -1);
  closest_distances_2 = std::vector<float>(point_mat1.rows, -1);


  cv::Mat multi_channeled_mat1;
  cv::Mat multi_channeled_mat2;

  point_mat1.convertTo(multi_channeled_mat1, CV_32FC2);
  point_mat2.convertTo(multi_channeled_mat2, CV_32FC2);

  cv::flann::Index flann_index(multi_channeled_mat2, cv::flann::KDTreeIndexParams(2));  // using 2 randomized kdtrees

  cv::Mat mat_indices(point_mat1.rows, 1, CV_32S);
  cv::Mat mat_dists(point_mat1.rows, 1, CV_32F);
  flann_index.knnSearch(multi_channeled_mat1, mat_indices, mat_dists, 1, cv::flann::SearchParams(64) );

  int* indices_ptr = mat_indices.ptr<int>(0);
  //float* dists_ptr = mat_dists.ptr<float>(0);
  for (int i=0;i<mat_indices.rows;++i) {
    closest_indices[i] = indices_ptr[i];
  }

  mat_dists.copyTo(cv::Mat(closest_distances_2));

  // ---------------------------- naive version ---------------------------- //
  // max allowed distance between corresponding points
//  const float max_distance = 0.5;
//
//  for (size_t i = 0, len_i = (size_t)point_mat1.rows; i < len_i; i++)
//  {
//    int closest_point_idx = -1;
//    float closest_distance_2 = std::pow(max_distance, 2.0f);
//
//    for (size_t j = 0, len_j = (size_t)point_mat2.rows; j < len_j; j++)
//    {
//      auto distance2 =
//        std::pow(point_mat2.at<float>(j, 0) - point_mat1.at<float>(i, 0), 2.0f)
//        + std::pow(point_mat2.at<float>(j, 1) - point_mat1.at<float>(i, 1), 2.0f);
//
//      if (distance2 < closest_distance_2)
//      {
//        closest_distance_2 = distance2;
//        closest_point_idx = (int)j;
//      }
//    }
//
//    if (closest_point_idx >= 0)
//    {
//      closest_indices[i] = closest_point_idx;
//      closest_distances_2[i] = closest_distance_2;
//    }
//  }
}

void ICPSlam::vizClosestPoints(cv::Mat &point_mat1,
                               cv::Mat &point_mat2,
                               const tf::Transform &T_2_1,int kf_id,int iter)
{
  assert(point_mat1.size == point_mat2.size);
  static int laser_count=0;
  const float resolution = 0.005;

  float *float_array = (float*)(point_mat1.data);
  float size_m = std::accumulate(
    float_array, float_array + point_mat1.total(), std::numeric_limits<float>::min(),
    [](float max, float current)
    {
      return current > max ? current : max;
    }
  );
  // add some slack
  size_m += 0.5;

  int size_pix = (int)(size_m / resolution);

  cv::Mat img(
    size_pix,
    size_pix,
    CV_8UC3,
    cv::Scalar(0, 0, 0)
  );

  auto meters_to_pix = [&size_pix, resolution](float meters) {
    int pix = (int)(meters / resolution + size_pix / 2.0f);
    pix = std::max(0, pix);
    pix = std::min(size_pix - 1, pix);
    return pix;
  };

  cv::Mat transformed_point_mat2 = utils::transformPointMat(T_2_1, point_mat2);

  for (size_t i = 0, len_i = (size_t)point_mat1.rows; i < len_i; i++)
  {
    float x1 = point_mat1.at<float>(i, 0);
    float y1 = point_mat1.at<float>(i, 1);
    float x2 = transformed_point_mat2.at<float>(i, 0);
    float y2 = transformed_point_mat2.at<float>(i, 1);

    auto pix_x1 = meters_to_pix(x1);
    auto pix_y1 = meters_to_pix(y1);
    auto pix_x2 = meters_to_pix(x2);
    auto pix_y2 = meters_to_pix(y2);

    cv::Point point1(pix_x1, pix_y1);
    cv::Point point2(pix_x2, pix_y2);

    cv::circle(img, point1, 5, cv::Scalar(0, 0, 255), -1);
    cv::circle(img, point2, 5, cv::Scalar(255, 0, 0), -1);

    cv::line(img, point1, point2, cv::Scalar(0, 255, 0), 2);
  }

  cv::Mat tmp;
  cv::flip(img, tmp, 0);
  std::string add="/home/mannatk/sfuhome/icp_lasers/"+std::to_string(kf_id)+"_"+std::to_string(iter)+".png";
  cv::imwrite(add, img);
  laser_count++;
}

} // namespace icp_slam

