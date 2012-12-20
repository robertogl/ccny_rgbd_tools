#include "ccny_rgbd/mapping/keyframe_loop_detector.h"

namespace ccny_rgbd
{

KeyframeLoopDetector::KeyframeLoopDetector(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), 
  nh_private_(nh_private)
{
  srand(time(NULL));

  // params
  if (!nh_private_.getParam ("loop/max_ransac_iterations", max_ransac_iterations_))
    max_ransac_iterations_ = 2000;
}

KeyframeLoopDetector::~KeyframeLoopDetector()
{

}

void KeyframeLoopDetector::generateKeyframeAssociations(
  KeyframeVector& keyframes,
  KeyframeAssociationVector& associations)
{
  prepareFeaturesForRANSAC(keyframes);

  createConsecutiveAssociations(keyframes, associations);

  //simplifiedRingAssociations(keyframes, associations);
  //ringAssociations(keyframes, associations);
  treeAssociations(keyframes, associations);
  //manualBruteForceAssociations(keyframes, associations);
}

bool KeyframeLoopDetector::addManualAssociation(
  int kf_idx_a, int kf_idx_b,
  KeyframeVector& keyframes,
  KeyframeAssociationVector& associations)
{ 
  RGBDKeyframe& keyframe_a = keyframes[kf_idx_a];
  RGBDKeyframe& keyframe_b = keyframes[kf_idx_b];

  tf::Transform initial_guess = keyframe_b.pose;
  tf::Transform transformation_a2b;

  bool result = pairwiseAlignGICP(keyframe_a, keyframe_b,
    initial_guess, transformation_a2b);

  if (result)
  {
    // create an association object
    KeyframeAssociation association;
    association.kf_idx_a = kf_idx_a;
    association.kf_idx_b = kf_idx_b;
    association.a2b = transformation_a2b;
    associations.push_back(association);
  }

  //keyframe_b.pose = keyframe_a.pose * transformation_a2b;

  return result;
}

bool KeyframeLoopDetector::pairwiseAlignGICP(
  RGBDKeyframe& keyframe_a, RGBDKeyframe& keyframe_b,
  const tf::Transform& initial_guess, 
  tf::Transform& transform)
{
  int nn_count_ = 20;
  double gicp_epsilon_ = 0.004;
  double vgf_res_ = 0.02;

  keyframe_a.constructDensePointCloud();
  keyframe_b.constructDensePointCloud();

  pcl::VoxelGrid<PointT> vgf; //TODO make member
  vgf.setLeafSize (vgf_res_, vgf_res_, vgf_res_);
  vgf.setFilterFieldName ("z");
  vgf.setFilterLimits (0, 5.0);
 
  PointCloudT::Ptr cloud_in_a = 
    boost::shared_ptr<PointCloudT>(new PointCloudT());
  PointCloudT::Ptr cloud_in_b = 
    boost::shared_ptr<PointCloudT>(new PointCloudT());

  *cloud_in_a = keyframe_a.cloud;
  *cloud_in_b = keyframe_b.cloud;

  PointCloudT cloud_a;
  PointCloudT cloud_b;

  vgf.setInputCloud(cloud_in_a);
  vgf.filter(cloud_a);

  vgf.setInputCloud(cloud_in_b);
  vgf.filter(cloud_b);

  pcl::transformPointCloud(cloud_a, cloud_a, eigenFromTf(keyframe_a.pose));
  pcl::transformPointCloud(cloud_b, cloud_b, eigenFromTf(keyframe_b.pose));

  // **** set up registration

  ccny_gicp::GICPAlign reg_;
  GICPParams params;
  params.max_distance = 0.40;
  params.solve_rotation = true;
  params.max_iteration = 20;
  params.max_iteration_inner = 20;
  params.epsilon = 5e-4;
  params.epsilon_rot = 2e-3;
  params.debug = false;
  reg_.setUseColor(false);
  reg_.setParams(params);

  // ***** set up GICP data set ********************************
  printf("Setting up data...\n");

  ccny_gicp::GICPPointSetKd gicp_data;
  gicp_data.setNormalCount(nn_count_);

  for (unsigned int i = 0; i < cloud_b.points.size(); ++i)
  {
    PointGICP p;
    p.x   = cloud_b.points[i].x;
    p.y   = cloud_b.points[i].y;
    p.z   = cloud_b.points[i].z;
    p.rgb = cloud_b.points[i].rgb;
    gicp_data.AppendPoint(p);
  }

  // create kd tree
  KdTree gicp_data_kdtree;
  gicp_data_kdtree.setInputCloud(gicp_data.getCloud());
  gicp_data.setKdTree(&gicp_data_kdtree);
  
  // compute matrices
  gicp_data.SetGICPEpsilon(gicp_epsilon_);
  gicp_data.computeMatrices();

  reg_.setData(&gicp_data);

  // **** set up gicp model set ********************************
  printf("Setting up model...\n");

  ccny_gicp::GICPPointSetKd gicp_model;
  gicp_model.setNormalCount(nn_count_);

  for (unsigned int i = 0; i < cloud_a.points.size(); ++i)
  {
    PointGICP p;
    p.x   = cloud_a.points[i].x;
    p.y   = cloud_a.points[i].y;
    p.z   = cloud_a.points[i].z;
    p.rgb = cloud_a.points[i].rgb;
    gicp_model.AppendPoint(p);
  }

  // create kd tree
  KdTree gicp_model_kdtree;
  gicp_model_kdtree.setInputCloud(gicp_model.getCloud());
  gicp_model.setKdTree(&gicp_model_kdtree);
  
  // compute matrices
  gicp_model.SetGICPEpsilon(gicp_epsilon_);
  gicp_model.computeMatrices();

  reg_.setModel(&gicp_model);

  // **** perform alignment
  printf("Aligning...\n");

  Eigen::Matrix4f corr_eigen;
  reg_.align(corr_eigen); 

  // in the fixed frame
  transform = tfFromEigen(corr_eigen);
  
  // in a's frame
  //keyframe_b.pose = transform * keyframe_b.pose;

  keyframe_b.pose = tfFromEigen(corr_eigen) * keyframe_b.pose;

  return true;
}


void KeyframeLoopDetector::prepareFeaturesForRANSAC(
  KeyframeVector& keyframes)
{
  double init_surf_threshold = 400.0;
  double min_surf_threshold = 25;
  int desired_keypoints = 200;
  bool save = true;

  printf("preparing SURF features for RANSAC associations...\n");  

  cv::SurfDescriptorExtractor extractor;
 
  for (unsigned int kf_idx = 0; kf_idx < keyframes.size(); kf_idx++)
  { 
    RGBDKeyframe& keyframe = keyframes[kf_idx];

    double surf_threshold = init_surf_threshold;

    while (surf_threshold >= min_surf_threshold)
    {
      cv::SurfFeatureDetector detector(surf_threshold);
      keyframe.keypoints.clear();
      detector.detect(keyframe.rgb_img, keyframe.keypoints);

      printf("\t[%d] %d keypoints detecting with %.1f\n", 
        kf_idx, (int)keyframe.keypoints.size(), surf_threshold); 

      if ((int)keyframe.keypoints.size() < desired_keypoints)
        surf_threshold /= 2.0;
      else break;
    }

    if (save)
    {
      cv::Mat kp_img;
      cv::drawKeypoints(keyframe.rgb_img, keyframe.keypoints, kp_img);
      std::stringstream ss1;
      ss1 << "kp_" << kf_idx;
      cv::imwrite("/home/idryanov/ros/images/ransac/" + ss1.str() + ".png", kp_img);
    }

    // TODO: not sure what's best here, reuse frame members, or make new ones

    //printf("[%d] computing descriptors\n", kf_idx);  
    extractor.compute(keyframe.rgb_img, keyframe.keypoints, keyframe.descriptors);
 
    //printf("[%d] computing distributions\n", kf_idx);  
    keyframe.computeDistributions();
    //keyframe.kp_cloud.clear();
  }
}

// produces k random numbers in the range [0, n).
void KeyframeLoopDetector::getRandomIndices(
  int k, int n, std::vector<int>& output)
{
  // TODO: This is Monte-Carlo based random sampling, maybe
  //       not the best way to do this

  while ((int)output.size() < k)
  {
    int random_number = rand() % n;
    bool duplicate = false;    

    for (unsigned int i = 0; i < output.size(); ++i)
    {
      if (output[i] == random_number)
      {
        duplicate = true;
        break;
      }
    }

    if (!duplicate)
      output.push_back(random_number);
  }
}

void KeyframeLoopDetector::manualBruteForceAssociations(
  KeyframeVector& keyframes,
  KeyframeAssociationVector& associations)
{
  // params
  double max_eucl_dist    = 0.05;
  double max_desc_dist    = 10.0;
  double min_inlier_ratio = 0.75;
  double min_inliers      = 20;

  double max_eucl_dist_sq = max_eucl_dist * max_eucl_dist;

  // insert time-consecutive frames
  for (unsigned int kf_idx = 0; kf_idx < keyframes.size()-1; ++kf_idx)
  {
    unsigned int kf_idx_a = kf_idx;
    unsigned int kf_idx_b = kf_idx+1;

    // set up the two keyframe references
    RGBDKeyframe& keyframe_a = keyframes[kf_idx_a];
    RGBDKeyframe& keyframe_b = keyframes[kf_idx_b];

    // create an association object
    KeyframeAssociation association;
    association.kf_idx_a = kf_idx_a;
    association.kf_idx_b = kf_idx_b;
    //association.matches  = inlier_matches;
    association.a2b = keyframe_a.pose.inverse() * keyframe_b.pose;;
    associations.push_back(association);
  }

  // generate a list of all keyframe indices, for which the keyframe
  // is manually added
  std::vector<unsigned int> manual_keyframe_indices;
  
  for (unsigned int kf_idx = 0; kf_idx < keyframes.size(); ++kf_idx)
  {
    const RGBDKeyframe& keyframe = keyframes[kf_idx];
    if(keyframe.manually_added) 
    {
      printf("Manual keyframe: %d\n", kf_idx);
      manual_keyframe_indices.push_back(kf_idx);
    }
  }

  for (unsigned int mn_idx_a = 0; mn_idx_a < manual_keyframe_indices.size(); ++mn_idx_a)
  for (unsigned int mn_idx_b = mn_idx_a+1; mn_idx_b < manual_keyframe_indices.size(); ++mn_idx_b)
  {
    // extract the indices of the manual keyframes
    unsigned int kf_idx_a = manual_keyframe_indices[mn_idx_a];
    unsigned int kf_idx_b = manual_keyframe_indices[mn_idx_b];

    // set up the two keyframe references
    RGBDKeyframe& keyframe_a = keyframes[kf_idx_a];
    RGBDKeyframe& keyframe_b = keyframes[kf_idx_b];

    // perform ransac matching, b onto a
    std::vector<cv::DMatch> all_matches, inlier_matches;
    Eigen::Matrix4f transformation;

    pairwiseMatchingRANSAC(keyframe_a, keyframe_b, 
      max_eucl_dist_sq, max_desc_dist, min_inlier_ratio,
      all_matches, inlier_matches, transformation);

    if (inlier_matches.size() >= min_inliers)
    {
      printf("[RANSAC %d -> %d] OK   (%d / %d)\n", 
        kf_idx_a, kf_idx_b,
        (int)inlier_matches.size(), (int)all_matches.size());

      // create an association object
      KeyframeAssociation association;
      association.kf_idx_a = kf_idx_a;
      association.kf_idx_b = kf_idx_b;
      association.matches  = inlier_matches;
      association.a2b = tfFromEigen(transformation);
      associations.push_back(association);
    }
    else
    {
      printf("[RANSAC %d -> %d] FAIL (%d / %d)\n", 
        kf_idx_a, kf_idx_b,
        (int)inlier_matches.size(), (int)all_matches.size());
    }
  }
}

void KeyframeLoopDetector::simplifiedRingAssociations(
  KeyframeVector& keyframes,
  KeyframeAssociationVector& associations)
{
  printf("Calculating RANSAC associations using ring topology...\n");  

  // params
  double max_eucl_dist    = 0.05;
  double max_desc_dist    = 10.0;
  double min_inlier_ratio = 0.75;
  double min_inliers      = 20;
  bool   save             = true;
  int    n_ring_neighbors = 1;      // each keyframe gets matched to n neighbors
  
  double max_eucl_dist_sq = max_eucl_dist * max_eucl_dist;

  for (unsigned int kf_idx_a = 0; kf_idx_a < keyframes.size(); ++kf_idx_a)
  for (int n = 1; n <= n_ring_neighbors; ++n)
  {   
    // determine second index, with wrap-around
    int kf_idx_b = kf_idx_a + n;
    if (kf_idx_b >= (int)keyframes.size()) 
      kf_idx_b -= (int)keyframes.size(); 

    // set up the two keyframe references
    RGBDKeyframe& keyframe_a = keyframes[kf_idx_a];
    RGBDKeyframe& keyframe_b = keyframes[kf_idx_b];

    // if time consecutive, just use vo odometry 

    if (kf_idx_b - kf_idx_a == 1)
    {
      // create an association object
      KeyframeAssociation association;
      association.kf_idx_a = kf_idx_a;
      association.kf_idx_b = kf_idx_b;
      //association.matches  = inlier_matches;  // TODO
      association.a2b = keyframe_a.pose.inverse() * keyframe_b.pose;
      associations.push_back(association);
    }
    else
    {
      // perform ransac matching, b onto a
      std::vector<cv::DMatch> all_matches, inlier_matches;
      Eigen::Matrix4f transformation;

      pairwiseMatchingRANSAC(keyframe_a, keyframe_b, 
        max_eucl_dist_sq, max_desc_dist, min_inlier_ratio,
        all_matches, inlier_matches, transformation);

      if (inlier_matches.size() >= min_inliers)
      {
        printf("[RANSAC %d -> %d] OK   (%d / %d)\n", 
          kf_idx_a, kf_idx_b,
          (int)inlier_matches.size(), (int)all_matches.size());

        // create an association object
        KeyframeAssociation association;
        association.kf_idx_a = kf_idx_a;
        association.kf_idx_b = kf_idx_b;
        association.matches  = inlier_matches;
        association.a2b = tfFromEigen(transformation);
        associations.push_back(association);
    
        // save image
        if (save)
        {
          cv::Mat img_matches;
          cv::drawMatches(keyframe_b.rgb_img, keyframe_b.keypoints, 
                          keyframe_a.rgb_img, keyframe_a.keypoints, 
                          inlier_matches, img_matches);
          std::stringstream ss1;
          ss1 << kf_idx_a << "_to_" << kf_idx_b;
          cv::imwrite("/home/idryanov/ros/images/ransac/" + ss1.str() + ".png", img_matches);
        }
      }
      else
      {
        printf("[RANSAC %d -> %d] FAIL (%d / %d)\n", 
          kf_idx_a, kf_idx_b,
          (int)inlier_matches.size(), (int)all_matches.size());
   
        // save image
        if (save)
        {
          cv::Mat img_matches;
          cv::drawMatches(keyframe_b.rgb_img, keyframe_b.keypoints, 
                          keyframe_a.rgb_img, keyframe_a.keypoints, 
                          all_matches, img_matches);
          std::stringstream ss1;
          ss1 << kf_idx_a << "_xx_" << kf_idx_b;
          cv::imwrite("/home/idryanov/ros/images/ransac/" + ss1.str() + ".png", img_matches);
        }
      }
    }
  }
}

void KeyframeLoopDetector::ringAssociations(
  KeyframeVector& keyframes,
  KeyframeAssociationVector& associations)
{
  printf("Calculating RANSAC associations using ring topology...\n");  

  // params
  double max_eucl_dist    = 0.10;
  double max_desc_dist    = 10.0;
  double min_inlier_ratio = 0.75;
  double min_inliers      = 20;
  bool   save             = true;
  int    n_ring_neighbors = 3;      // each keyframe gets matched to n neighbors
  
  double max_eucl_dist_sq = max_eucl_dist * max_eucl_dist;

  for (unsigned int kf_idx_a = 0; kf_idx_a < keyframes.size(); ++kf_idx_a)
  for (int n = 1; n <= n_ring_neighbors; ++n)
  {   
    // determine second index, with wrap-around
    int kf_idx_b = kf_idx_a + n;
    if (kf_idx_b >= (int)keyframes.size()) 
      kf_idx_b -= (int)keyframes.size(); 
    
    // set up the two keyframe references
    RGBDKeyframe& keyframe_a = keyframes[kf_idx_a];
    RGBDKeyframe& keyframe_b = keyframes[kf_idx_b];

    // perform ransac matching, b onto a
    std::vector<cv::DMatch> all_matches, inlier_matches;
    Eigen::Matrix4f transformation;

    pairwiseMatchingRANSAC(keyframe_a, keyframe_b, 
      max_eucl_dist_sq, max_desc_dist, min_inlier_ratio,
      all_matches, inlier_matches, transformation);

    if (inlier_matches.size() >= min_inliers)
    {
      printf("[RANSAC %d -> %d] OK   (%d / %d)\n", 
        kf_idx_a, kf_idx_b,
        (int)inlier_matches.size(), (int)all_matches.size());

      // create an association object
      KeyframeAssociation association;
      association.kf_idx_a = kf_idx_a;
      association.kf_idx_b = kf_idx_b;
      association.matches  = inlier_matches;
      association.a2b = tfFromEigen(transformation);
      associations.push_back(association);
  
      // save image
      if (save)
      {
        cv::Mat img_matches;
        cv::drawMatches(keyframe_b.rgb_img, keyframe_b.keypoints, 
                        keyframe_a.rgb_img, keyframe_a.keypoints, 
                        inlier_matches, img_matches);
        std::stringstream ss1;
        ss1 << kf_idx_a << "_to_" << kf_idx_b;
        cv::imwrite("/home/idryanov/ros/images/ransac/" + ss1.str() + ".png", img_matches);
      }
    }
    else
    {
      printf("[RANSAC %d -> %d] FAIL (%d / %d)\n", 
        kf_idx_a, kf_idx_b,
        (int)inlier_matches.size(), (int)all_matches.size());
 
      // save image
      if (save)
      {
        cv::Mat img_matches;
        cv::drawMatches(keyframe_b.rgb_img, keyframe_b.keypoints, 
                        keyframe_a.rgb_img, keyframe_a.keypoints, 
                        all_matches, img_matches);
        std::stringstream ss1;
        ss1 << kf_idx_a << "_xx_" << kf_idx_b;
        cv::imwrite("/home/idryanov/ros/images/ransac/" + ss1.str() + ".png", img_matches);
      }
    }
  }
}

void KeyframeLoopDetector::pairwiseMatchingRANSAC(
  RGBDFrame& frame_a, RGBDFrame& frame_b,
  double max_eucl_dist_sq, 
  double max_desc_dist,
  double sufficient_inlier_ratio,
  std::vector<cv::DMatch>& all_matches,
  std::vector<cv::DMatch>& best_inlier_matches,
  Eigen::Matrix4f& best_transformation)
{
  // constants
  int min_sample_size = 3;

  cv::FlannBasedMatcher matcher_;          // for SURF
  TransformationEstimationSVD svd_;

  // **** build candidate matches ***********************************

  // assumes detectors and distributions are computed
  // establish all matches from b to a
  matcher_.match(frame_b.descriptors, frame_a.descriptors, all_matches);

  //printf("\tAll matches: %d\n", (int)all_matches.size());

  // remove bad matches - too far away in descriptor space,
  //                    - nan, too far, or cov. too big
  std::vector<cv::DMatch> candidate_matches;
  for (unsigned int m_idx = 0; m_idx < all_matches.size(); ++m_idx)
  {
    const cv::DMatch& match = all_matches[m_idx];
    int idx_b = match.queryIdx;
    int idx_a = match.trainIdx; 

    if (match.distance < max_desc_dist && 
        frame_a.kp_valid[idx_a] && 
        frame_b.kp_valid[idx_b])
    {
      candidate_matches.push_back(all_matches[m_idx]);
    }
  }

  int size = candidate_matches.size();
  //printf("\tCandidate matches: %d\n", size);

  // **** build 3D features for SVD ********************************

  PointCloudFeature features_a, features_b;

  features_a.resize(size);
  features_b.resize(size);

  for (int m_idx = 0; m_idx < size; ++m_idx)
  {
    const cv::DMatch& match = candidate_matches[m_idx];
    int idx_b = match.queryIdx;
    int idx_a = match.trainIdx; 

    PointFeature& p_a = features_a[m_idx];
    p_a.x = frame_a.kp_means[idx_a](0,0);
    p_a.y = frame_a.kp_means[idx_a](1,0);
    p_a.z = frame_a.kp_means[idx_a](2,0);

    PointFeature& p_b = features_b[m_idx];
    p_b.x = frame_b.kp_means[idx_b](0,0);
    p_b.y = frame_b.kp_means[idx_b](1,0);
    p_b.z = frame_b.kp_means[idx_b](2,0);
  }

  // **** main RANSAC loop ****************************************

  int best_n_inliers = 0;
  Eigen::Matrix4f transformation; // transformation used inside loop
  
  for (int iteration = 0; iteration < max_ransac_iterations_; ++iteration)
  {
    // generate random indices
    std::vector<int> sample_idx;
    getRandomIndices(min_sample_size, size, sample_idx);

    // build initial inliers from random indices
    std::vector<int> inlier_idx;
    std::vector<cv::DMatch> inlier_matches;

    for (unsigned int s_idx = 0; s_idx < sample_idx.size(); ++s_idx)
    {
      int m_idx = sample_idx[s_idx];
      inlier_idx.push_back(m_idx);
      inlier_matches.push_back(candidate_matches[m_idx]);
    } 

    // estimate transformation from minimum set of random samples
    svd_.estimateRigidTransformation(
      features_b, inlier_idx,
      features_a, inlier_idx,
      transformation);

    // evaluate transformation fitness by checking distance to all points
    PointCloudFeature features_b_tf;
    pcl::transformPointCloud(features_b, features_b_tf, transformation);

    for (int m_idx = 0; m_idx < size; ++m_idx)
    {
      const PointFeature& p_a = features_a[m_idx];
      const PointFeature& p_b = features_b_tf[m_idx];

      float dist_sq = distEuclideanSq(p_a, p_b);
      
      if (dist_sq < max_eucl_dist_sq)
      {
        inlier_idx.push_back(m_idx);
        inlier_matches.push_back(candidate_matches[m_idx]);

        // reestimate transformation from all inliers
        svd_.estimateRigidTransformation(
          features_b, inlier_idx,
          features_a, inlier_idx,
          transformation);
        pcl::transformPointCloud(features_b, features_b_tf, transformation);
      }
    }

    // check if inliers are better than the best model so far
    int n_inliers = inlier_idx.size();

    if (n_inliers > best_n_inliers)
    {
      svd_.estimateRigidTransformation(
        features_b, inlier_idx,
        features_a, inlier_idx,
        transformation);

      best_n_inliers = n_inliers;
      best_transformation = transformation;
      best_inlier_matches = inlier_matches;
    }

    // check if we reached ratio termination criteria
    double inlier_ratio = (double) n_inliers / (double) size;

    if (inlier_ratio > sufficient_inlier_ratio)
      break;
  }

  //printf("\tInlier matches: %d\n", best_n_inliers);
}


void KeyframeLoopDetector::createConsecutiveAssociations(
  const KeyframeVector& keyframes,
  KeyframeAssociationVector& associations)
{
  for (unsigned int kf_idx_a = 0; kf_idx_a < keyframes.size()-1; ++kf_idx_a)
  {
    // determine second index, NO wrap-around
    int kf_idx_b = kf_idx_a + 1;

    // set up the two keyframe references
    const RGBDKeyframe& keyframe_a = keyframes[kf_idx_a];
    const RGBDKeyframe& keyframe_b = keyframes[kf_idx_b];

    // create an association object
    KeyframeAssociation association;
    association.kf_idx_a = kf_idx_a;
    association.kf_idx_b = kf_idx_b;
    //association.matches  = inlier_matches;  // no matches needed
    association.a2b = keyframe_a.pose.inverse() * keyframe_b.pose;
    associations.push_back(association);
  }
}

void KeyframeLoopDetector::createDescriptorMatcherFLANN(
  const KeyframeVector& keyframes,
  cv::FlannBasedMatcher& matcher)
{
  std::vector<cv::Mat> descriptors_vector;

  for (unsigned int kf_idx = 0; kf_idx < keyframes.size(); ++kf_idx)
  {
    const RGBDKeyframe& keyframe = keyframes[kf_idx];
    descriptors_vector.push_back(keyframe.descriptors);
  }
  matcher.add(descriptors_vector);

  matcher.train();
}

void KeyframeLoopDetector::treeAssociations(
  KeyframeVector& keyframes,
  KeyframeAssociationVector& associations)
{
   // RANSAC params
  double max_eucl_dist    = 0.03;
  double max_desc_dist    = 1.0;
  double min_inlier_ratio = 1.0;
  double min_inliers      = 30;
  bool   save             = true;
 
  // tree matching params
  int k_nn = 15;                        // look for X nearest neighbors
  int n_ransac_tests = 15;              // consider the first X frames with highest corr. count
  int min_corresp_for_ransac_test = 30; // if there are fewer corresp. than X, don't bother to RANSAC

  // derived params  
  double max_eucl_dist_sq = max_eucl_dist * max_eucl_dist;
  
  // **** build a tree with deature descriptors
  printf("Building FLANN matcher...\n");

  cv::FlannBasedMatcher matcher;
  createDescriptorMatcherFLANN(keyframes, matcher);

  // **** lookup per frame
  printf("Frame lookups...\n");

  for (unsigned int kf_idx = 0; kf_idx < keyframes.size(); ++kf_idx)
  {
    printf("\tFrame %d of %d: \n", (int)kf_idx, (int)keyframes.size());
    RGBDFrame& keyframe = keyframes[kf_idx];

    // find k nearest matches for each feature in the keyframe
    std::vector<std::vector<cv::DMatch> > matches_vector;
    matcher.knnMatch(keyframe.descriptors, matches_vector, k_nn);

    // create empty bins vector of Pairs <count, image_index>
    std::vector<std::pair<int, int> > bins;
    bins.resize(keyframes.size());
    for (unsigned int b = 0; b < bins.size(); ++b) 
      bins[b] = std::pair<int, int>(0, b);

    // fill out bins with match indices
    for (unsigned int j = 0; j < matches_vector.size(); ++j)
    {
      std::vector<cv::DMatch>& matches = matches_vector[j];
      for (unsigned int k = 0; k < matches.size(); ++k)
      {
        bins[matches[k].imgIdx].first++;
      }
    }
  
    // sort - highest counts first
    std::sort(bins.begin(), bins.end(), std::greater<std::pair<int, int> >());

    // output results
    printf("\t\tbest matches: ");
    for (int b = 0; b < n_ransac_tests; ++b)
      printf("[%d(%d)] ", bins[b].second, bins[b].first);
    printf("\n");

    // **** find top X candidates
    
    printf("\t\tcandidate matches: ");
    std::vector<int> ransac_candidates;
    int n_ransac_candidates_found = 0;
    for (unsigned int b = 0; b < bins.size(); ++b)
    {
      unsigned int index_a = kf_idx;
      unsigned int index_b = bins[b].second;
      int corresp_count = bins[b].first;

      // test for order consistence
      // (+1 to ignore consec. frames)
      // and for minimum number of keypoints
      if (index_b > index_a + 1 && 
          corresp_count >= min_corresp_for_ransac_test)
      {
        ransac_candidates.push_back(index_b);
        ++n_ransac_candidates_found;
        printf("[%d(%d)] ", index_b, corresp_count);
      }

      if (n_ransac_candidates_found >= n_ransac_tests) break;
    }
    printf("\n");

    // **** test top X candidates using RANSAC

    for (unsigned int rc = 0; rc < ransac_candidates.size(); ++rc)
    {
      unsigned int kf_idx_a = kf_idx;
      unsigned int kf_idx_b = ransac_candidates[rc];

      RGBDKeyframe& keyframe_a = keyframes[kf_idx_a];
      RGBDKeyframe& keyframe_b = keyframes[kf_idx_b];

      std::vector<cv::DMatch> all_matches;
      std::vector<cv::DMatch> inlier_matches;

      // perform ransac matching, b onto a
      Eigen::Matrix4f transformation;

      pairwiseMatchingRANSAC(keyframe_a, keyframe_b, 
        max_eucl_dist_sq, max_desc_dist, min_inlier_ratio,
        all_matches, inlier_matches, transformation);

      if (inlier_matches.size() >= min_inliers)
      {
        if (save)
        {
          cv::Mat img_matches;
          cv::drawMatches(keyframe_b.rgb_img, keyframe_b.keypoints, 
                          keyframe_a.rgb_img, keyframe_a.keypoints, 
                          inlier_matches, img_matches);

          std::stringstream ss1;
          ss1 << kf_idx_a << "_to_" << kf_idx_b;
          cv::imwrite("/home/idyanov/ros/images/ransac/" + ss1.str() + ".png", img_matches);
        }

        printf("RANSAC %d -> %d: PASS\n", kf_idx_a, kf_idx_b);

        // create an association object
        KeyframeAssociation association;
        association.kf_idx_a = kf_idx_a;
        association.kf_idx_b = kf_idx_b;
        association.matches  = inlier_matches;
        association.a2b = tfFromEigen(transformation);
        associations.push_back(association);      
      }
      else  
        printf("RANSAC %d -> %d: FAIL\n", kf_idx_a, kf_idx_b);
    }
  }
}

} // namespace ccny_rgbd
