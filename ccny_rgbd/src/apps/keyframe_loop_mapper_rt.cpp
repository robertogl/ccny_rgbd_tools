#include "ccny_rgbd/apps/keyframe_loop_mapper_rt.h"

namespace ccny_rgbd
{

KeyframeLoopMapperRT::KeyframeLoopMapperRT(ros::NodeHandle nh, ros::NodeHandle nh_private):
  KeyframeMapper(nh, nh_private),
  n_processed_keyframes_(0)
{
  ROS_INFO("Starting RGBD Keyframe Loop Mapper RT");

  loop_solver_ = new KeyframeLoopSolverTORO(nh, nh_private);

  process_thread_ = boost::thread(&KeyframeLoopMapperRT::processLoop, this);  
}

KeyframeLoopMapperRT::~KeyframeLoopMapperRT()
{



}

void KeyframeLoopMapperRT::processLoop()
{
  ros::Rate r(10);

  while(true)
  {
    queue_mutex_.lock();
    int queue_size = keyframe_queue_.size();
    queue_mutex_.unlock();

    if (queue_size == 0)
    {
      keyframes_mutex_.lock();
      int size = keyframes_.size();
      keyframes_mutex_.unlock();
        
      if (size > n_processed_keyframes_)
      {
        processKeyframe(n_processed_keyframes_);
        n_processed_keyframes_++;
      }  
    }
    r.sleep();
  }
}

void KeyframeLoopMapperRT::processKeyframe(int idx)
{
  keyframes_mutex_.lock();
  RGBDKeyframe keyframe = keyframes_[idx];
  keyframes_mutex_.unlock();

  ros::WallTime start = ros::WallTime::now();;
  findFeatures(keyframe);

  descriptors_vector_.push_back(keyframe.descriptors);

  matcher_.add(descriptors_vector_);
  matcher_.train();

  printf("[%d] LoopMapperRT duration: %.1fms\n", 
    n_processed_keyframes_, getMsDuration(start));
}

void KeyframeLoopMapperRT::findFeatures(RGBDKeyframe& keyframe)
{
  double init_surf_threshold = 400.0;
  double min_surf_threshold = 25;
  int desired_keypoints = 200;

  cv::SurfDescriptorExtractor extractor;
 
  double surf_threshold = init_surf_threshold;

  while (surf_threshold >= min_surf_threshold)
  {
    cv::SurfFeatureDetector detector(surf_threshold);
    keyframe.keypoints.clear();
    detector.detect(keyframe.rgb_img, keyframe.keypoints);

    printf("\t %d keypoints detecting with %.1f\n", 
      (int)keyframe.keypoints.size(), surf_threshold); 

    if ((int)keyframe.keypoints.size() < desired_keypoints)
      surf_threshold /= 2.0;
    else break;
  }

  // TODO: not sure what's best here, reuse frame members, or make new ones

  extractor.compute(keyframe.rgb_img, keyframe.keypoints, keyframe.descriptors);
 
  keyframe.computeDistributions();
}

} // namespace ccny_rgbd
