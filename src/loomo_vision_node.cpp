//
// Created by kai on 17-7-20.
//

#include <stdio.h>
#include <string>

#include "ros/ros.h"
#include "ros/publisher.h"

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>

#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include "image_transport/subscriber_filter.h"

#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"
#include <depth_image_proc/depth_traits.h>
#include "sensor_msgs/image_encodings.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

using namespace std;
using namespace sensor_msgs;
using namespace depth_image_proc;
namespace enc = sensor_msgs::image_encodings;

typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo>
MySyncPolicy;

static string gRsTopicPrefix = "/loomo/realsense";
ros::Publisher mrgbImagePubr;

bool DEBUG = 0;

//std::vector<Eigen::Vector3f> depthImage2PointClouds(cv::Mat depthImage){
//    const float camera_scalar = 1000;
//    int rows = depthImage.rows;
//    int cols = depthImage.cols;
//    cv::Mat depth = depthImage.clone();
//    float constant_x = 1.0f / (camera_scalar * camera.fx);
//    float constant_y = 1.0f / (camera_scalar * camera.fy);
//    std::vector<Eigen::Vector3f> depthPoints;
//    depthPoints.clear();
//
//    for (int r = 0; r < rows; r++) {
//        unsigned short* data = depthImage.ptr<ushort>(r);
//        for (int c = 0; c < cols; c += 2) {
//            unsigned short z = data[c];
//            // check if depth data available at this pixel
//            if (z == 0)
//            {
//                continue;
//            }
//
//            Eigen::Vector4f p;
//            Eigen::Vector3f dst_p;
//            float projection_ratio1 = z * constant_x;
//            float projection_ratio2 = z * constant_y;
//            p[0] = (c - camera.cx) * projection_ratio1;
//            p[1] = (r - camera.cy) * projection_ratio2;
//            p[2] = (z) / camera_scalar;
//            p[3] = 1;
//
//            dst_p = p;
//
//            depthPoints.push_back(dst_p);
//
//        }
//    }
//    return depthPoints;
//}


//DS4 color::
//        FocalLength = 625.807 625.807
//PrincipalPoint = 319.895 234.772
//Distortion = 0 0 0 0 0
//
//DS4 depth::
//        FocalLength = 303.485 303.485
//PrincipalPoint = 159.5 119.161
//Distortion = 0 0 0 0 0
//
//DS4 depth to color extrinsic::
//translation = -58.0402 -0.0668228 -0.568635
//rotation =
//1 0 0
//0 1 0
//0 0 1
template<typename T> void convert(const sensor_msgs::ImageConstPtr& depth_msg,const sensor_msgs::ImageConstPtr& rgb_msg,
const PointCloud2::Ptr& cloud_msg,int red_offset, int green_offset, int blue_offset, int color_step);

    sensor_msgs::CameraInfo rgb_model_;
    sensor_msgs::CameraInfo depth_model_;

PointCloud2Ptr depthColor2Pc2(const ImageConstPtr& depth, const CameraInfoConstPtr& depthInfo, \
                            const ImageConstPtr& color, const CameraInfoConstPtr& colorInfo) {
  //check if the input image has to be resized
  rgb_model_ = * colorInfo;
  depth_model_ = *depthInfo;
  sensor_msgs::ImageConstPtr rgb_msg = color;
  
  if(depth->width != color->width || depth->height != color->height)
  {
    sensor_msgs::CameraInfo info_msg_tmp = *colorInfo;
    info_msg_tmp.width = depthInfo->width;
    info_msg_tmp.height = depthInfo->height;
    float ratio = float(depthInfo->width)/float(rgb_msg->width);
    info_msg_tmp.K[0] *= ratio;
    info_msg_tmp.K[2] *= ratio;
    info_msg_tmp.K[4] *= ratio;
    info_msg_tmp.K[5] *= ratio;
    info_msg_tmp.P[0] *= ratio;
    info_msg_tmp.P[2] *= ratio;
    info_msg_tmp.P[5] *= ratio;
    info_msg_tmp.P[6] *= ratio;
    rgb_model_ = info_msg_tmp;
  
    cv_bridge::CvImageConstPtr cv_ptr;
    try{
      cv_ptr = cv_bridge::toCvShare(rgb_msg, rgb_msg->encoding);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
    cv_bridge::CvImage cv_rsz;
    cv_rsz.header = cv_ptr->header;
    cv_rsz.encoding = cv_ptr->encoding;
    cv::resize(cv_ptr->image.rowRange(0,depth->height/ratio), cv_rsz.image, cv::Size(depth->width, depth->height));
    cv::Mat newCameraMatrix;
    
    if ((rgb_msg->encoding == enc::RGB8) || (rgb_msg->encoding == enc::BGR8) || (rgb_msg->encoding == enc::MONO8))
      rgb_msg = cv_rsz.toImageMsg();
    else
      rgb_msg = cv_bridge::toCvCopy(cv_rsz.toImageMsg(), enc::RGB8)->toImageMsg();
  }
  else
    rgb_msg = color;
  

  int red_offset, green_offset, blue_offset, color_step;
  if (rgb_msg->encoding == enc::RGB8)
  {
    red_offset   = 0;
    green_offset = 1;
    blue_offset  = 2;
    color_step   = 3;
  }
  else if (rgb_msg->encoding == enc::BGR8)
  {
    red_offset   = 2;
    green_offset = 1;
    blue_offset  = 0;
    color_step   = 3;
  }
  else if (rgb_msg->encoding == enc::MONO8)
  {
    red_offset   = 0;
    green_offset = 0;
    blue_offset  = 0;
    color_step   = 1;
  }
  else
  {
    try
    {
      rgb_msg = cv_bridge::toCvCopy(rgb_msg, enc::RGB8)->toImageMsg();
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR_THROTTLE(5, "Unsupported encoding [%s]: %s", rgb_msg->encoding.c_str(), e.what());
    }
    red_offset   = 0;
    green_offset = 1;
    blue_offset  = 2;
    color_step   = 3;
  }
    
  PointCloud2Ptr pc2ptr(new PointCloud2());
  pc2ptr->header = depth->header;
  pc2ptr->height = depth->height;
  pc2ptr->width  = depth->width;
  pc2ptr->is_dense = false;
  pc2ptr->is_bigendian = false;
  
  sensor_msgs::PointCloud2Modifier pcd_modifier(*pc2ptr);
  pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  if (depth->encoding == enc::TYPE_16UC1 || depth->encoding == enc::MONO16)
  {
    convert<uint16_t>(depth, rgb_msg, pc2ptr, red_offset, green_offset, blue_offset, color_step);
  }
  else if (depth->encoding == enc::TYPE_32FC1)
  {
    convert<float>(depth, rgb_msg, pc2ptr, red_offset, green_offset, blue_offset, color_step);
  }
  else
  {
    ROS_ERROR_THROTTLE(5, "Depth image has unsupported encoding [%s]", depth->encoding.c_str());
  }
    return pc2ptr;
}

template<typename T>
void convert(const sensor_msgs::ImageConstPtr& depth_msg,const sensor_msgs::ImageConstPtr& rgb_msg,
const PointCloud2::Ptr& cloud_msg,int red_offset, int green_offset, int blue_offset, int color_step)
{
  //mrgbImagePubr.publish(rgb_msg);
  //cout<<rgb_msg->height<<","<<rgb_msg->width<<endl;
  
  double unit_scaling = DepthTraits<T>::toMeters( T(1) );
  float constant_x = unit_scaling / depth_model_.K.at(2);
  float constant_y = unit_scaling / depth_model_.K.at(5);
  float bad_point = std::numeric_limits<float>::quiet_NaN ();
  
  const T* depth_row = reinterpret_cast<const T*>(&depth_msg->data[0]);
  int row_step = depth_msg->step / sizeof(T);
  const uint8_t* rgb = &rgb_msg->data[0];
  int rgb_skip = rgb_msg->step - rgb_msg->width * color_step;

  sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*cloud_msg, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*cloud_msg, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*cloud_msg, "b");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_a(*cloud_msg, "a");

  for (int v = 0; v < int(cloud_msg->height); ++v, depth_row += row_step, rgb += rgb_skip)
  {
    for (int u = 0; u < int(cloud_msg->width); ++u, rgb += color_step, ++iter_x, ++iter_y, ++iter_z, ++iter_a, ++iter_r, ++iter_g, ++iter_b)
    {
      T depth = depth_row[u];

      // Check for invalid measurements
      if (!DepthTraits<T>::valid(depth))
      {
        *iter_x = *iter_y = *iter_z = bad_point;
      }
      else
      {
        // Fill in XYZ
        *iter_x = (u - depth_model_.K.at(0) ) * depth * constant_x;
        *iter_y = (v - depth_model_.K.at(4) ) * depth * constant_y;
        *iter_z = DepthTraits<T>::toMeters(depth);
      }

      // Fill in color
      *iter_a = 255;
      *iter_r = rgb[red_offset];
      *iter_g = rgb[green_offset];
      *iter_b = rgb[blue_offset];
    }
  }
}

PointCloudPtr depth2PointCloud(const ImageConstPtr& depth, const CameraInfoConstPtr& depthInfo) {
    const float camera_scalar = 1000;

    int width = depthInfo->width, height = depthInfo->height;
    float fx = depthInfo->K[0], fy = depthInfo->K[4], cx = depthInfo->K[2], cy = depthInfo->K[5];
    float constant_x = 1.0f / (camera_scalar * fx);
    float constant_y = 1.0f / (camera_scalar * fy);

    PointCloudPtr pcptr(new PointCloud());

    pcptr->points.resize(width * height);
    pcptr->header = depth->header;

    for (int y = 0; y < height; ++y) {
        const unsigned short* data = (const unsigned short*)&depth->data[y * depth->step];
        geometry_msgs::Point32* target = &pcptr->points[y * width];
        for (int x = 0; x < width; ++x) {
            unsigned short z = data[x];
            if (!z)
                continue;
            target[x].x = (x - cx) * z * constant_x;
            target[x].y = (y - cy) * z * constant_y;
            target[x].z = z / camera_scalar;
        }
    }
    return pcptr;
}


class VisualizerRepubr
{
public:
    // subscribed topic names: relative, topic_base as prefix
    static string RsColorTopic, RsColorInfoTopic, RsDepthTopic, RsDepthInfoTopic;
    // published topic names: relative to node handle private topic
    static string RsPointCloudTopic;
    static string RsPointCloud2Topic;

    VisualizerRepubr(ros::NodeHandle& nh, const string& topic_base): \
            mNhPtr(nh), mIt(nh), mRsColorSubr(mIt, topic_base + RsColorTopic, 5), mRsDepthSubr(nh, topic_base + RsDepthTopic, 5), \
            mRsColorInfoSubr(nh, topic_base + RsColorInfoTopic, 5), mRsDepthInfoSubr(nh, topic_base + RsDepthInfoTopic, 5), \
            mSync4(MySyncPolicy(1000), mRsColorSubr, mRsColorInfoSubr, mRsDepthSubr, mRsDepthInfoSubr) {

        printf("Initialized image transport::SubscriberFilter[%s,%s], CameraInfo subscriber[%s,%s]\n", mRsColorSubr.getTopic().c_str(), \
                mRsDepthSubr.getTopic().c_str(), mRsColorInfoSubr.getTopic().c_str(), mRsDepthInfoSubr.getTopic().c_str());
//        mRsColorSubr.registerCallback(boost::bind(&VisualizerRepubr::rsColorCb, this, _1));
//        mRsColorInfoSubr.registerCallback(boost::bind(&VisualizerRepubr::rsColorInfoCb, this, _1));
//        mRsDepthSubr.registerCallback(boost::bind(&VisualizerRepubr::rsDepthCb, this, _1));
//        mRsDepthInfoSubr.registerCallback(boost::bind(&VisualizerRepubr::rsDepthInfoCb, this, _1));
        mSync4.registerCallback(boost::bind(&VisualizerRepubr::sync4Callback, this, _1, _2, _3, _4));

        mRsPcPubr = nh.advertise<sensor_msgs::PointCloud>(RsPointCloudTopic, 10);
	mRsPc2Pubr = nh.advertise<sensor_msgs::PointCloud2>(RsPointCloud2Topic, 10);
	mrgbImagePubr = nh.advertise<sensor_msgs::Image>("rgbimage", 10);
    }

protected:

    void sync4Callback(const sensor_msgs::ImageConstPtr& color, const sensor_msgs::CameraInfoConstPtr& colorInfo, \
                    const sensor_msgs::ImageConstPtr& depth, const sensor_msgs::CameraInfoConstPtr& depthInfo) {
        static int cc = 0;
        static double pts = 0.d;
        double curtime = color->header.stamp.toSec();
        if (!cc)
            pts = curtime;
        ++cc;
        double diff = curtime - pts;
        if (diff > 1.d) {
            double fps = (cc - 1) / diff;
            ROS_INFO("synced image callback fps[%.2lf] avg[%.2lf]ms {color,colorInfo,depth,depthInfo}[%.3lf,%.3lf,%.3lf,%.3lf]", fps, 1000.d/fps, \
                color->header.stamp.toSec(), colorInfo->header.stamp.toSec(), \
                depth->header.stamp.toSec(), depthInfo->header.stamp.toSec());
            pts = curtime;
            cc = 1;
        }

        PointCloudPtr pcptr = depth2PointCloud(depth, depthInfo);
        mRsPcPubr.publish(pcptr);
	
	PointCloud2Ptr pcptr2 = depthColor2Pc2(depth, depthInfo, color, colorInfo);
        mRsPc2Pubr.publish(pcptr2);
    }

    // helper for debug input stream
    void rsColorCb(const sensor_msgs::ImageConstPtr& color) {
        printf("rgb image callback [%s,%.3lf]\n", color->header.frame_id.c_str(), color->header.stamp.toSec());
    }

    // helper for debug input stream
    void rsColorInfoCb(const sensor_msgs::CameraInfoConstPtr& info) {
        printf("rgb info callback [%s,%.3lf]\n", info->header.frame_id.c_str(), info->header.stamp.toSec());
    }
    // helper for debug input stream
    void rsDepthCb(const sensor_msgs::ImageConstPtr& depth) {
        printf("depth image callback [%s,%.3lf]\n", depth->header.frame_id.c_str(), depth->header.stamp.toSec());
    }

    // helper for debug input stream
    void rsDepthInfoCb(const sensor_msgs::CameraInfoConstPtr& info) {
        printf("depth info callback [%s,%.3lf]\n", info->header.frame_id.c_str(), info->header.stamp.toSec());
    }

private:
    ros::NodeHandle mNhPtr;
    image_transport::ImageTransport mIt;
    image_transport::SubscriberFilter mRsColorSubr;
    message_filters::Subscriber<Image> mRsDepthSubr;
    message_filters::Subscriber<CameraInfo> mRsColorInfoSubr, mRsDepthInfoSubr;

    // synced RS 4 images based on approximate time
    message_filters::Synchronizer<MySyncPolicy> mSync4;

    // PointCloud publisher
    ros::Publisher mRsPcPubr;
    ros::Publisher mRsPc2Pubr;
    
};

// default values for these topic names
string VisualizerRepubr::RsColorTopic = "/rgb";
string VisualizerRepubr::RsColorInfoTopic = "/rgb/camera_info";
string VisualizerRepubr::RsDepthTopic = "/depth";
string VisualizerRepubr::RsDepthInfoTopic = "/depth/camera_info";
string VisualizerRepubr::RsPointCloudTopic = "realsense/pointcloud";
string VisualizerRepubr::RsPointCloud2Topic = "realsense/pointcloud2";

int main(int argc, char **argv) {
    ros::init(argc, argv, "loomo_vision_node");

    ros::NodeHandle nh("~");

    VisualizerRepubr repubr(nh, gRsTopicPrefix);

    ros::spin();

    return 0;
}
