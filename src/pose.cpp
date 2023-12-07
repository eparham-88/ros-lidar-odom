#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<vector>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>


#include "Eigen/Core"
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/features2d.hpp>
#include "opencv2/highgui.hpp"
// #include <opencv2/xfeatures2d.hpp>
#include "tf2_eigen/tf2_eigen.h"
#include <Eigen/SVD>


#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>


using namespace sensor_msgs;
using namespace message_filters;



class Lidar
{
public:
  Lidar() {
    // Initialize your Lidar parameters here
    beam_altitude_angles << 44.47, 43.93, 43.22, 42.34, 41.53, 40.95, 40.23, 39.38, 38.58, 37.98, 37.27, 
                            36.44, 35.65, 35.05, 34.33, 33.52, 32.74, 32.12, 31.41, 30.63, 29.85, 29.23, 
                            28.52, 27.75, 26.97, 26.35, 25.65, 24.88, 24.12, 23.49, 22.79, 22.04, 21.29, 
                            20.65, 19.96, 19.21, 18.48, 17.83, 17.12, 16.41, 15.66, 15.02, 14.32, 13.6, 
                            12.88, 12.22, 11.53, 10.82, 10.1, 9.44, 8.74, 8.04, 7.33, 6.66, 5.97, 5.27, 
                            4.56, 3.89, 3.2, 2.5, 1.8, 1.12, 0.43, -0.26, -0.96, -1.65, -2.33, -3.02, 
                            -3.73, -4.42, -5.1, -5.79, -6.48, -7.2, -7.89, -8.56, -9.26, -9.98, -10.67, 
                            -11.35, -12.04, -12.77, -13.45, -14.12, -14.83, -15.56, -16.26, -16.93, -17.63, 
                            -18.37, -19.07, -19.73, -20.44, -21.19, -21.89, -22.55, -23.25, -24.02, -24.74, 
                            -25.39, -26.09, -26.87, -27.59, -28.24, -28.95, -29.74, -30.46, -31.1, -31.81, 
                            -32.62, -33.35, -33.99, -34.71, -35.54, -36.27, -36.91, -37.63, -38.47, -39.21, 
                            -39.84, -40.57, -41.44, -42.2, -42.81, -43.55, -44.45, -45.21, -45.82;
    
    beam_azimuth_angles << 11.01, 3.81, -3.25, -10.19, 10.57, 3.63, -3.17, -9.88, 10.18, 3.48, -3.1, -9.6, 
                            9.84, 3.36, -3.04, -9.37, 9.56, 3.23, -2.99, -9.16, 9.3, 3.14, -2.95, -8.98, 9.08, 
                            3.05, -2.91, -8.84, 8.9, 2.98, -2.88, -8.71, 8.74, 2.92, -2.85, -8.59, 8.6, 2.87, 
                            -2.83, -8.51, 8.48, 2.82, -2.81, -8.43, 8.39, 2.78, -2.79, -8.37, 8.31, 2.75, -2.79, 
                            -8.33, 8.25, 2.72, -2.78, -8.3, 8.22, 2.71, -2.79, -8.29, 8.19, 2.69, -2.79, -8.29, 
                            8.18, 2.7, -2.8, -8.29, 8.2, 2.7, -2.81, -8.32, 8.22, 2.71, -2.82, -8.36, 8.27, 2.72, 
                            -2.83, -8.41, 8.32, 2.74, -2.85, -8.48, 8.41, 2.78, -2.87, -8.55, 8.5, 2.81, -2.89, 
                            -8.65, 8.63, 2.86, -2.92, -8.77, 8.76, 2.92, -2.97, -8.91, 8.93, 2.99, -3, -9.06, 
                            9.12, 3.06, -3.06, -9.24, 9.34, 3.15, -3.12, -9.46, 9.6, 3.26, -3.18, -9.71, 9.91, 
                            3.38, -3.25, -10, 10.26, 3.53, -3.34, -10.35, 10.68, 3.7, -3.43, -10.74;
    
    beam_to_lidar_transform << 1, 0, 0, 27.116, 
                                0, 1, 0, 0,
                                0, 0, 1, 0,
                                0, 0, 0, 1;
    
    lidar_to_sensor_transform << -1, 0, 0, 0,
                                0, -1, 0, 0,
                                0, 0, 1, 38.195,
                                0, 0, 0, 1;

    lidar_to_beam_origin_mm = 27.116;
    
    scan_width = 1024;
    scan_height = 128;
    
    n = sqrt((beam_to_lidar_transform(0,3)*beam_to_lidar_transform(0,3)) + (beam_to_lidar_transform(2,3)*beam_to_lidar_transform(2,3)));
  }

  Eigen::Vector4d getXYZCoords(int u, int v, u_int16_t d) {

    // std::cout << "d: " << d;
    // std::cout << ", u: " << u;
    // std::cout << ", v: " << v << std::endl;

    double r = static_cast<double>(d) * 4.0; // convert to mm

    double theta_encoder = 2.0 * M_PI * (1.0 - static_cast<double>(v) / static_cast<double>(scan_width));
    double theta_azimuth = 0.0 * (-2.0 * M_PI * (beam_azimuth_angles(u) / 360.0));
    double phi = 2.0 * M_PI * (beam_altitude_angles(u) / 360.0);

    double x = (r - n) * cos(theta_encoder + theta_azimuth) * cos(phi) + beam_to_lidar_transform(0, 3) * cos(theta_encoder);
    double y = (r - n) * sin(theta_encoder + theta_azimuth) * cos(phi) + beam_to_lidar_transform(0, 3) + sin(theta_encoder);
    double z = (r - n) * sin(phi) + beam_to_lidar_transform(2, 3);

    // Correct for lidar to sensor
    Eigen::Vector4d homogeneous;
    homogeneous << x, y, z, 1.0;

    Eigen::Vector4d corrected = lidar_to_sensor_transform * homogeneous;
    corrected /= corrected(3);

    return corrected;
  }


private:
  Eigen::VectorXd beam_altitude_angles = Eigen::VectorXd(128);
  Eigen::VectorXd beam_azimuth_angles = Eigen::VectorXd(128);
  Eigen::Matrix4d beam_to_lidar_transform;
  Eigen::Matrix4d lidar_to_sensor_transform;
  double lidar_to_beam_origin_mm;
  int scan_width;
  int scan_height;
  double n;
};




class Frame
{
public:
  Frame() {}

  Frame(cv::Mat *image, cv::Mat *depth_image, Lidar *my_lidar, cv::Ptr<cv::BRISK> detector) {
    depth_img = *depth_image;
    cv::GaussianBlur(*image, img, cv::Size(5,5), 0);
    detector->detectAndCompute(img, cv::Mat(), kp, des);

    filter_keypoints();
    
    find_world_coordinates(my_lidar);
  }

  void find_match_coordinates(std::vector<cv::DMatch> *matches, bool query) {

    Eigen::MatrixXd points(4,int(matches->size()));

    for (unsigned int i=0; i<matches->size(); i++) {
      int i_m;

      if (query) {
        i_m = (*matches)[i].queryIdx;
      } else {
        i_m = (*matches)[i].trainIdx;
      }

      points.col(i) = xyz.col(i_m);
    }

    match_xyz = points;
  }



  cv::Mat des;
  Eigen::MatrixXd match_xyz;

private:
  cv::Mat img;
  cv::Mat depth_img;
  std::vector<cv::KeyPoint> kp;
  Eigen::MatrixXd xyz;
  cv::Mat mask;

  void filter_keypoints() {
    cv::Mat laplacian;
    
    cv::Laplacian(depth_img, laplacian, CV_64F);

    cv::GaussianBlur(laplacian, laplacian, cv::Size(9,9), 0);

    cv::Mat min_depth_mask = depth_img<200;

    laplacian.setTo(65536, min_depth_mask);
    
    cv::dilate(laplacian, laplacian, cv::Mat::ones(5, 5, CV_16U), cv::Point(-1, -1), 4, 1, 1);

    cv::GaussianBlur(laplacian, laplacian, cv::Size(19,19), 0);

    mask = laplacian > 350;

    std::vector<cv::KeyPoint> kp_f;
    cv::Mat des_f;

    int u = 0;
    int v = 0;

    for (unsigned int i = 0; i < kp.size(); i++) {
      u = int(kp[i].pt.x);
      v = int(kp[i].pt.y);
      u_int8_t mask_val = mask.at<u_int8_t>(v,u);

      if (mask_val > 0) {
        continue;
      } else {
        kp_f.push_back(kp[i]);
        des_f.push_back (des.row(i));
      }
      
    }

    kp = kp_f;
    des = des_f;

    if (false) {
      cv::Mat img_keypoints = img.clone();

      cv::imshow("Mask", mask);

      img_keypoints.setTo(0, mask);
      
      cv::drawKeypoints( img_keypoints, kp, img_keypoints );
      cv::imshow("Filtered BRISK Key Points", img_keypoints );
      cv::waitKey();
    }

    return;
  }

  void find_world_coordinates(Lidar *my_lidar) {
    int u = 0;
    int v = 0;
    u_int16_t d = 0;
    Eigen::Vector4d point(4);

    Eigen::MatrixXd points(4, int(kp.size()));

    // int temp_u = 56;
    // int temp_v = 232;
    // u_int16_t temp_d = 1161;
    // point = my_lidar->getXYZCoords(temp_v, temp_u, temp_d);
    // std::cout << point << std::endl;

    for (int i = 0; i < kp.size(); i++) {
      u = int(kp[i].pt.y);
      v = int(kp[i].pt.x);
      d = depth_img.at<u_int16_t>(u,v);
      point = my_lidar->getXYZCoords(u, v, d);
      // std::cout << point << std::endl;
      points.col(i) = point;

    }

    xyz = points;

    return;
  }

};






class SubscribeAndPublish
{
public:
  SubscribeAndPublish():sync2(sig_sub_,rng_sub_,10)
  {
    sig_sub_.subscribe(n_, "/ouster/signal_image", 1);

    rng_sub_.subscribe(n_, "/ouster/range_image", 1);

    sync2.registerCallback(boost::bind(&SubscribeAndPublish::callback, this, _1, _2));

    pub_ = n_.advertise<nav_msgs::Odometry>("lidar/odom", 50);

    my_lidar = Lidar();

    /* Threshold, Octaves, Pattern Scale */
    brisk_ptr = cv::BRISK::create(5, 2, 1.0f);


    flann_matcher = cv::FlannBasedMatcher(cv::makePtr<cv::flann::LshIndexParams>(6, 12, 2));

  }


  cv_bridge::CvImageConstPtr GetImage(const sensor_msgs::ImageConstPtr &img_msg, bool mono16=false)
  {
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      if (mono16) {
        cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO16);
      } else {
        cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
      }
      
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    
    return cv_ptr;
  }

  void callback(const ImageConstPtr& sig_img_ptr, const ImageConstPtr& rng_img_ptr)
  {

    cv_bridge::CvImageConstPtr cv_sig_ptr = GetImage(sig_img_ptr);
    cv_bridge::CvImageConstPtr cv_rng_ptr = GetImage(rng_img_ptr, true);

    cv::Mat sig_img = cv_sig_ptr->image.clone();
    cv::Mat rng_img = cv_rng_ptr->image.clone();

    this_frame = Frame(&sig_img, &rng_img, &my_lidar, brisk_ptr);

    if (seq_==0) {
      /* No previous frame will exist, so set it and output a zero translation/rotation */


    } else {
      /* This frame and previous frame should be populated */
      std::vector<std::vector<cv::DMatch>> preliminary_matches;
      flann_matcher.knnMatch(prev_frame.des, this_frame.des, preliminary_matches, 2);

      std::vector<cv::DMatch> matches;
      for (unsigned int i=0; i<preliminary_matches.size(); i++) {
        if (preliminary_matches[i][0].distance < 0.7 * preliminary_matches[i][1].distance) {
          matches.push_back(preliminary_matches[i][0]);
        }
      }

      prev_frame.find_match_coordinates(&matches, true);

      this_frame.find_match_coordinates(&matches, false);

      Eigen::Matrix4d M = fit_transformation(&this_frame.match_xyz, &prev_frame.match_xyz);




    }


    




  


    

    ros::Time time = ros::Time::now();

		nav_msgs::Odometry msg;
		msg.header.stamp = time;
		msg.header.frame_id = "os_sensor";
		msg.header.seq = seq_;
    // msg.pose.pose = pose_msg.pose;

		pub_.publish(msg);

    prev_frame = this_frame;

		seq_++;
  }

private:
  ros::NodeHandle n_;
  ros::Publisher pub_;

  message_filters::Subscriber<Image> rng_sub_;
  message_filters::Subscriber<Image> sig_sub_;

  TimeSynchronizer<Image, Image> sync2;

  int seq_ = 0;

  Frame this_frame;
  Frame prev_frame;

  Lidar my_lidar;
  cv::Ptr<cv::BRISK> brisk_ptr;
  cv::FlannBasedMatcher flann_matcher;

  
  Eigen::Matrix4d fit_transformation(Eigen::MatrixXd *From, Eigen::MatrixXd *To) {

    Eigen::MatrixXd From_inliers = (*From);
    Eigen::MatrixXd To_inliers = (*To);

    std::cout << "difference norm: " << ((*To) - (*From)).colwise().norm() << std::endl;

    Eigen::Matrix4d M;

    int last_k = 0;

    for (int j=0; j<20; j++) {

      Eigen::MatrixXd new_from_inliers(From_inliers.rows(), From_inliers.cols());
      Eigen::MatrixXd new_to_inliers(To_inliers.rows(), To_inliers.cols());

      /* Find transformation */
      M = find_transform(&From_inliers, &To_inliers);

      /* Find norm difference between points */
      Eigen::MatrixXd diff = To_inliers - M * From_inliers;


      Eigen::VectorXd norm = diff.colwise().norm();

      // std::cout << "norm mean: " << norm.mean() << std::endl;



      const double std_dev = sqrt((norm.array() - norm.mean()).square().sum() / (norm.size()-1));
      // std::cout << "std_dev = " << std_dev << std::endl;

      int k=0;

      for (int i=0; i<norm.size(); i++) {
        if (norm.coeff(i) < norm.mean()+0.8*std_dev) {
          new_from_inliers.col(k) = From_inliers.col(i);
          new_to_inliers.col(k) = To_inliers.col(i);
          k++;
        }
      }

      if (k < 10 || k == last_k) {
        break;
      }

      From_inliers = new_from_inliers.block(0,0,4,k);
      To_inliers = new_to_inliers.block(0,0,4,k);

      last_k = k;
    }

    std::cout << "Filtered: " << From->cols() << " -> " << From_inliers.cols() << std::endl;

    return M;

  }



  Eigen::Matrix4d find_transform(Eigen::MatrixXd *From, Eigen::MatrixXd *To) {

    /* Reconfigure data into 3xn */
    Eigen::MatrixXd P = From->topRows(3);
    Eigen::MatrixXd Q = To->topRows(3);

    /* Find centroids */
    Eigen::Vector3d P_bar = P.rowwise().mean();
    Eigen::Vector3d Q_bar = Q.rowwise().mean();

    // /* Offset by centroids */
    Eigen::MatrixXd X = P - P_bar;
    Eigen::MatrixXd Y = Q - Q_bar;

    // /* Calculate 3x3 covariance */
    Eigen::MatrixXd cov = X*Y.transpose();

    // /* Use SVD to calculate the 3x3 Matrices U and V from coveriance */
    Eigen::JacobiSVD<Eigen::MatrixXd> svd;
    svd.compute(cov, Eigen::ComputeFullU | Eigen::ComputeFullV);

    // /* Find rotation */
    Eigen::MatrixXd m = Eigen::MatrixXd::Identity(int(svd.matrixU().rows()), int(svd.matrixU().cols()));
    int i = svd.matrixU().rows()-1;
    int j = svd.matrixU().cols()-1;
    
    m(svd.matrixU().rows()-1,svd.matrixU().cols()-1) = (svd.matrixV() * svd.matrixU().transpose()).determinant();
    Eigen::Matrix3d R = svd.matrixV() * m * svd.matrixU().transpose();

    // /* Find translation */
    Eigen::Vector3d T = Q_bar - R * P_bar;

    Eigen::Matrix4d M;
    M << R.coeff(0,0), R.coeff(0,1), R.coeff(0,2), T.coeff(0),
         R.coeff(1,0), R.coeff(1,1), R.coeff(1,2), T.coeff(1),
         R.coeff(2,0), R.coeff(2,1), R.coeff(2,2), T.coeff(2),
                    0,            0,            0,          1;

    return M;
  }

};






int main(int argc, char** argv)
{
  std::cout << "Starting LiDAR Odometry" << std::endl;
  ros::init(argc, argv, "lidar_odom_node");

  SubscribeAndPublish SubscriberAndPublisher;

  ros::spin();


  return 0;
}