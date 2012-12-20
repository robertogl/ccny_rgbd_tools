#ifndef CCNY_RGBD_RGBD_KEYFRAME_H
#define CCNY_RGBD_RGBD_KEYFRAME_H

#include <vector>
#include <queue>
#include <boost/filesystem.hpp>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include "ccny_rgbd/structures/rgbd_frame.h"

namespace ccny_rgbd
{

class RGBDKeyframe: public RGBDFrame
{
  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    RGBDKeyframe();
  
    RGBDKeyframe(const RGBDFrame& frame);
  
    tf::Transform pose;
    PointCloudT   cloud;

    bool manually_added;

    double path_length_linear;
    double path_length_angular;

    void constructDensePointCloud(
      double max_z = 5.5,
      double max_stdev_z = 0.03);
};

typedef Eigen::aligned_allocator<RGBDKeyframe> KeyframeAllocator;
typedef std::vector<RGBDKeyframe, KeyframeAllocator> KeyframeVector;
typedef std::queue<RGBDKeyframe> KeyframeQueue;

bool saveKeyframe(
  const RGBDKeyframe& keyframe, 
  const std::string& path,
  bool in_fixed_frame);

bool loadKeyframe(
  RGBDKeyframe& keyframe, 
  const std::string& path);

bool saveKeyframes(
  const KeyframeVector& keyframes, 
  const std::string& path,
  bool in_fixed_frame = false);

bool loadKeyframes(
  KeyframeVector& keyframes, 
  const std::string& path);

} //namespace ccny_rgbd


#endif // CCNY_RGBD_RGBD_KEYFRAME_H
