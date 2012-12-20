#ifndef CCNY_RGBD_KEYFRAME_LOOP_DETECTOR_H
#define CCNY_RGBD_KEYFRAME_LOOP_DETECTOR_H

#include <tf/transform_datatypes.h>
#include <boost/thread/mutex.hpp>
#include <pcl/registration/transformation_estimation_svd.h>
#include <opencv2/nonfree/features2d.hpp>

#include <ccny_gicp/gicp.h>
#include <ccny_gicp/types.h>
#include <ccny_gicp/gicp_align.h>

#include "ccny_rgbd/structures/rgbd_keyframe.h"
#include "ccny_rgbd/structures/keyframe_association.h"
#include "ccny_rgbd/mapping/keyframe_generator.h"

namespace ccny_rgbd
{

typedef pcl::registration::TransformationEstimationSVD<PointFeature, PointFeature> TransformationEstimationSVD; 

class KeyframeLoopDetector
{
  public:

    KeyframeLoopDetector(ros::NodeHandle nh, ros::NodeHandle nh_private);
    virtual ~KeyframeLoopDetector();

    void generateKeyframeAssociations(
      KeyframeVector& keyframes,
      KeyframeAssociationVector& associations);

    bool addManualAssociation(
      int kf_idx_a, int kf_idx_b,
      KeyframeVector& keyframes,
      KeyframeAssociationVector& associations);

   protected:
  
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;

   private:

    // parameters
    int max_ransac_iterations_;

    void prepareFeaturesForRANSAC(KeyframeVector& keyframes);
 
    /* adds associations between consecutive frames
     * in the path, using their relative poses
     */
    void createConsecutiveAssociations(
      const KeyframeVector& keyframes,
      KeyframeAssociationVector& associations);

    void treeAssociations(
      KeyframeVector& keyframes,
      KeyframeAssociationVector& associations);

    void createDescriptorMatcherFLANN(
      const KeyframeVector& keyframes,
      cv::FlannBasedMatcher& matcher);
    
    void simplifiedRingAssociations(  
      KeyframeVector& keyframes,
      KeyframeAssociationVector& associations);

    void ringAssociations(  
      KeyframeVector& keyframes,
      KeyframeAssociationVector& associations);

    /* tries brite force search, but only through keyframes
     * which have been manually added (manually_added= = true)
     */
    void manualBruteForceAssociations(
      KeyframeVector& keyframes,
      KeyframeAssociationVector& associations);

    double distEuclideanSq(const PointFeature& a, const PointFeature& b)
    {
      // calculate squared Euclidean distance
      float dx = a.x - b.x;
      float dy = a.y - b.y;
      float dz = a.z - b.z;
      return dx*dx + dy*dy + dz*dz;
    }

    void getRandomIndices(int k, int n, std::vector<int>& output);

    bool pairwiseAlignGICP(
      RGBDKeyframe& keyframe_a, RGBDKeyframe& keyframe_b,
      const tf::Transform& initial_guess, 
      tf::Transform& transform);

    void pairwiseMatchingRANSAC(
      RGBDFrame& frame_a, RGBDFrame& frame_b,
      double max_eucl_dist_sq, 
      double max_desc_dist,
      double sufficient_inlier_ratio,
      std::vector<cv::DMatch>& all_matches,
      std::vector<cv::DMatch>& best_inlier_matches,
      Eigen::Matrix4f& best_transformation);

    //void bruteForceAssociations();
};

} // namespace ccny_rgbd

#endif // CCNY_RGBD_KEYFRAME_LOOP_DETECTOR_H
