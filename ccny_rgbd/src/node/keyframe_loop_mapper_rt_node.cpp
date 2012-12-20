#include "ccny_rgbd/apps/keyframe_loop_mapper_rt.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "KeyframeLoopMapperRT");  
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  ccny_rgbd::KeyframeLoopMapperRT km(nh, nh_private);
  ros::spin();
  return 0;
}
