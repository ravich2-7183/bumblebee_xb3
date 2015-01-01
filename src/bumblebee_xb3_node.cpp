/*
  bumblebee_xb3 subscribes to image_raw output of the
  camera1394 driver, which receives data in format7_mode rgb8 mode
  from a Bumblebee XB3 camera.

  The received image is deinterlaced and three stereo cameras are set
  up: stereocam_LC, stereocam_CR and stereocam_LR.

  TODO: mention that code is based on camer1394stereo package
 */

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <image_transport/camera_publisher.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>

class BumblebeeXB3 {
private:
  enum CameraSelector {RIGHT=0, CENTER=1, LEFT=2};
  ros::NodeHandle camera1394_nh_;
  ros::NodeHandle camera_nh_[3]; // camera node handles

  // image transport interfaces
  boost::shared_ptr<image_transport::ImageTransport> it_;
  image_transport::CameraPublisher image_pub_[3]; // image publishers

  // camera info
  boost::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_mgr_[3];
  sensor_msgs::CameraInfo camera_info_[3];

  // camera calibration information
  bool camera_info_is_set_[3];

  // TODO make these shared pointers (why?)
  sensor_msgs::Image image_msg_[3];

  std::vector<cv::Mat> split_images_;

public:
  BumblebeeXB3(ros::NodeHandle camera1394_nh):
    camera1394_nh_(camera1394_nh),
    it_(new image_transport::ImageTransport(camera1394_nh_))
  {
    const std::string camera_name[3] = {"right", "center", "left"};
    for(int i=0; i<3; i++)
    {
      camera_nh_[i] = ros::NodeHandle(camera1394_nh_, camera_name[i]);
      image_pub_[i] = it_->advertiseCamera(camera_name[i]+"/image_raw", 1);

      camera_info_mgr_[i] = boost::shared_ptr<camera_info_manager::CameraInfoManager>(new camera_info_manager::CameraInfoManager(camera_nh_[i]));
      camera_info_mgr_[i]->setCameraName("bb_xb3_" + camera_name[i]);
      camera_info_is_set_[i] = false;
      // set camera_info_ partially
      if(camera_info_mgr_[i]->isCalibrated())
      {
        camera_info_[i] = camera_info_mgr_[i]->getCameraInfo();
      }
      else
      {
        camera_info_[i].header.frame_id = "bb_xb3_" + camera_name[i];
      }
    }

  // ros::NodeHandle stereocam_LC_nh("stereocam_LC"),
  //                 stereocam_CR_nh("stereocam_CR"),
  //                 stereocam_LR_nh("stereocam_LR");
  }

  void imageCallback(const sensor_msgs::ImagePtr& ros_img_ptr)
  {
    // Deinterlace the image obtained from camera1394
    /*
      Note that the Bumblebee XB3 outputs image data formatted as
      pixel (byte) interleaved stereo images using Format_7. Pixel
      interleaved images use a raw 24bit-per-pixel format. Each sensor
      represents an RGB color channel, where red is from the right
      camera, blue from the left and green from the middle. Also each
      of these is a (GBRG) bayered image. Future firmware versions may
      support line (row) interleaved images, where the rows from each
      of the cameras are interleaved to speed processing.
     */

    cv_bridge::CvImagePtr cv_img_ptr(new cv_bridge::CvImage);

    try
    {
      cv_img_ptr = cv_bridge::toCvCopy(ros_img_ptr, sensor_msgs::image_encodings::RGB8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // cv::split splits an rgb image into 3 separate images
    cv::split(cv_img_ptr->image, split_images_);

    cv_bridge::CvImage cv_img;
    for(int i=0; i<3; i++)
    {
      /*
        First convert cv::Mat to CvImage and then to sensor_msgs::Image
        (see the CvBridge tutorial)
      
        CvBridge will recognize Bayer pattern encodings as having OpenCV
        type 8UC1 (8-bit unsigned, one channel), but it will not perform
        conversions to or from Bayer pattern. This is done instead by
        image_proc.
      */
      cv_img = cv_bridge::CvImage(cv_img_ptr->header,
                                  sensor_msgs::image_encodings::BAYER_GBRG8,
                                  split_images_[i]);
      cv_img.toImageMsg(image_msg_[i]);
      image_msg_[i].header.frame_id = camera_info_[i].header.frame_id;
      
      camera_info_[i].header.stamp = ros_img_ptr->header.stamp;
      if(!camera_info_is_set_[i])
      {
        camera_info_[i].width  = image_msg_[i].width;
        camera_info_[i].height = image_msg_[i].height;

        camera_info_is_set_[i] = true;
      }
      image_pub_[i].publish(image_msg_[i], camera_info_[i]);
    }
  }
}; // end of class BumblebeeXB3

int main(int argc, char **argv) {
  ros::init(argc, argv, "bumblebee_xb3");
  ros::NodeHandle nh;
  ros::NodeHandle camera1394_nh("camera"); // TODO get "camera" from a param/argv
  BumblebeeXB3 bb_xb3(camera1394_nh);
                      // left_nh, center_nh, right_nh, 
                      // stereocam_LC_nh, stereocam_CR_nh, stereocam_LR_nh);
  // TODO: remove hard coded assumption that camera1394 publishes in a certain namespace 
  // or else document this. 
  ros::Subscriber camera1394_rgb8_sub = nh.subscribe("camera/image_raw", 
                                                     1, 
                                                     &BumblebeeXB3::imageCallback, 
                                                     &bb_xb3);
  ros::spin();
  return 0;
}
