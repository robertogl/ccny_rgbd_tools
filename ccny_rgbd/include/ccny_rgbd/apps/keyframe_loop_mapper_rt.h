#ifndef CCNY_RGBD_KEYFRAME_LOOP_MAPPER_RT_H
#define CCNY_RGBD_KEYFRAME_LOOP_MAPPER_RT_H

#include <tf/transform_listener.h>
#include <opencv2/nonfree/features2d.hpp>

#include "ccny_rgbd/apps/keyframe_mapper.h"
#include "ccny_rgbd/mapping/keyframe_loop_detector.h"
#include "ccny_rgbd/mapping/keyframe_loop_solver.h"
#include "ccny_rgbd/mapping/keyframe_loop_solver_sba.h"
#include "ccny_rgbd/mapping/keyframe_loop_solver_toro.h"

#include "ccny_rgbd/GenerateAssociations.h"
#include "ccny_rgbd/AddManualAssociation.h"
#include "ccny_rgbd/SolveLoop.h"

namespace ccny_rgbd
{

class KeyframeLoopMapperRT: public KeyframeMapper
{
  public:

    KeyframeLoopMapperRT(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~KeyframeLoopMapperRT();

  private:

    boost::thread process_thread_;

    KeyframeLoopSolver * loop_solver_;

    KeyframeAssociationVector associations_;

    cv::FlannBasedMatcher matcher_;

    int n_processed_keyframes_;

    std::vector<cv::Mat> descriptors_vector_;

    void processLoop();

    void processKeyframe(int idx);
    void findFeatures(RGBDKeyframe& keyframe);
};

} // namespace ccny_rgbd

#endif // CCNY_RGBD_KEYFRAME_LOOP_MAPPER_RT_H
