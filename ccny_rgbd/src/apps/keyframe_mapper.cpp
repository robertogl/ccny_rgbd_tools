#include "ccny_rgbd/apps/keyframe_mapper.h"

namespace ccny_rgbd
{

KeyframeMapper::KeyframeMapper(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), 
  nh_private_(nh_private),
  n_keyframes_added_(0),
  manual_add_(false)
{
  ROS_INFO("Starting RGBD Keyframe Mapper");

  // **** parameters

  if (!nh_private_.getParam ("fixed_frame", fixed_frame_))
    fixed_frame_ = "/odom";
  if (!nh_private_.getParam ("kf/kf_dist_eps", kf_dist_eps_))
    kf_dist_eps_  = 0.10;
  if (!nh_private_.getParam ("kf/kf_angle_eps", kf_angle_eps_))
    kf_angle_eps_  = 10.0 * M_PI / 180.0;

  // **** init params

  keyframes_pub_ = nh_.advertise<PointCloudT>(
    "keyframes", 1);
  poses_pub_ = nh_.advertise<visualization_msgs::Marker>( 
    "keyframe_poses", 1);
  edges_pub_ = nh_.advertise<visualization_msgs::Marker>( 
    "keyframe_edges", 1);

  process_queue_thread_ = boost::thread(&KeyframeMapper::processQueueLoop, this);  

  // **** services

  add_manual_keyframe_service_ = nh_.advertiseService(
    "add_manual_keyframe", &KeyframeMapper::addManualKeyframeSrvCallback, this);
  pub_frame_service_ = nh_.advertiseService(
    "publish_keyframe", &KeyframeMapper::publishKeyframeSrvCallback, this);
  pub_frames_service_ = nh_.advertiseService(
    "publish_keyframes", &KeyframeMapper::publishAllKeyframesSrvCallback, this);
  recolor_service_ = nh_.advertiseService(
    "recolor", &KeyframeMapper::recolorSrvCallback, this);
  save_kf_service_ = nh_.advertiseService(
    "save_keyframes", &KeyframeMapper::saveKeyframesSrvCallback, this);
  save_kf_ff_service_ = nh_.advertiseService(
    "save_keyframes_ff", &KeyframeMapper::saveKeyframesFFSrvCallback, this);
 load_kf_service_ = nh_.advertiseService(
    "load_keyframes", &KeyframeMapper::loadKeyframesSrvCallback, this);
  save_full_service_ = nh_.advertiseService(
    "save_full_map", &KeyframeMapper::saveFullSrvCallback, this);

  // **** subscribers

  image_transport::ImageTransport rgb_it(nh_);
  image_transport::ImageTransport depth_it(nh_);

  sub_depth_.subscribe(depth_it, "/rgbd/depth", 1);
  sub_rgb_.subscribe(rgb_it, "/rgbd/rgb", 1);
  sub_info_.subscribe(nh_, "/rgbd/info", 1);

  // Synchronize inputs. Topic subscriptions happen on demand in the connection callback.
  int queue_size = 5;
  sync_.reset(new Synchronizer(SyncPolicy(queue_size), sub_depth_, sub_rgb_, sub_info_));
  sync_->registerCallback(boost::bind(&KeyframeMapper::RGBDCallback, this, _1, _2, _3));  
}

KeyframeMapper::~KeyframeMapper()
{

}


void KeyframeMapper::processQueueLoop()
{
  ros::Rate r(20);
  
  while(true)
  {
    processQueue(); 
    r.sleep();
  }
}

void KeyframeMapper::processQueue()
{
  queue_mutex_.lock();
  keyframes_mutex_.lock();

  if (!keyframe_queue_.empty())
  {
    printf("[-] transferring frame, queue size is %d\n", 
      (int)keyframe_queue_.size());

    keyframes_.push_back(keyframe_queue_.front());
    keyframe_queue_.pop();
    publishKeyframeData(keyframes_.size() - 1);
  }

  keyframes_mutex_.unlock();
  queue_mutex_.unlock();
}

void KeyframeMapper::RGBDCallback(
  const ImageMsg::ConstPtr& depth_msg,
  const ImageMsg::ConstPtr& rgb_msg,
  const CameraInfoMsg::ConstPtr& info_msg)
{
  tf::StampedTransform transform;

  const ros::Time& time = rgb_msg->header.stamp;

  try{
    tf_listener_.waitForTransform(
      fixed_frame_, rgb_msg->header.frame_id, time, ros::Duration(0.1));
    tf_listener_.lookupTransform(
      fixed_frame_, rgb_msg->header.frame_id, time, transform);  
  }
  catch(...)
  {
    return;
  }

  // create a new frame, and check whether we need to add a keyframe
  RGBDFrame frame(rgb_msg, depth_msg, info_msg);

  if (newKeyframeNeeded(transform))
    addKeyframe(frame, transform);

  //if (result) publishKeyframeData(keyframes_.size() - 1);
}

bool KeyframeMapper::processFrame(
  const RGBDFrame& frame, 
  const tf::Transform& pose)
{
  if (newKeyframeNeeded(pose))
  {
    addKeyframe(frame, pose);
    return true;
  }
  else return false;
}

bool KeyframeMapper::newKeyframeNeeded(const tf::Transform& pose)
{
  bool result;

  keyframes_mutex_.lock();

  if(n_keyframes_added_ == 0)
    result = true;
  else if (manual_add_)
    result = true;
  else
  {
    double dist, angle;
    getTfDifference(pose, last_keyframe_pose_, dist, angle);

    if (dist > kf_dist_eps_ || angle > kf_angle_eps_)
      result = true;
    else 
      result = false;
  }
  
  keyframes_mutex_.unlock();

  return result;
}

void KeyframeMapper::addKeyframe(
  const RGBDFrame& frame, 
  const tf::Transform& pose)
{
  RGBDKeyframe keyframe(frame);
  keyframe.pose = pose;
  keyframe.constructDensePointCloud();

  if (manual_add_)
  {
    ROS_INFO("Adding frame manually");
    manual_add_ = false;
    keyframe.manually_added = true;
  }

  keyframes_mutex_.lock();  
  n_keyframes_added_++;
  last_keyframe_pose_ = pose;
  keyframes_mutex_.unlock();

  queue_mutex_.lock(); 
  printf("[+] adding frame, queue size is %d\n", (int)keyframe_queue_.size());
  keyframe_queue_.push(keyframe);
  queue_mutex_.unlock(); 
}


bool KeyframeMapper::publishKeyframeSrvCallback(
  PublishKeyframe::Request& request,
  PublishKeyframe::Response& response)
{
  keyframes_mutex_.lock();

  if (request.id < 0 || request.id >= (int)keyframes_.size())
  {
    ROS_ERROR("request.id %d out of bounds (%d keyframes)", (int)request.id, (int)keyframes_.size());
    keyframes_mutex_.unlock();    
    return false;
  }

  publishKeyframeData(request.id);
  publishKeyframePose(request.id);
  usleep(25000);

  return true;
}

bool KeyframeMapper::recolorSrvCallback(
  Recolor::Request&  request,
  Recolor::Response& response)
{
  keyframes_mutex_.lock();

  srand(time(NULL));

  for (unsigned int kf_idx = 0; kf_idx < keyframes_.size(); ++kf_idx)
  {
    ROS_INFO("Recoloring frame %d", kf_idx);
    RGBDKeyframe& keyframe = keyframes_[kf_idx];

    int r = rand() % 255;
    int g = rand() % 255;
    int b = rand() % 255;

    for (unsigned int pt_idx = 0; pt_idx < keyframe.cloud.points.size(); ++pt_idx)
    {
      PointT& p = keyframe.cloud.points[pt_idx];
      
      p.r = r;
      p.g = g;
      p.b = b;
    }
  }

  keyframes_mutex_.unlock();

  return true;
}

bool KeyframeMapper::publishAllKeyframesSrvCallback(
  PublishAllKeyframes::Request&  request,
  PublishAllKeyframes::Response& response)
{
  if (request.step <= 0)
  {
    ROS_ERROR("request.step has to be >= 1");
    return false;
  }

  keyframes_mutex_.lock();

  for (int i = 0; i < (int)keyframes_.size(); i += request.step)
  {
    publishKeyframeData(i);
    publishKeyframePose(i);
    usleep(25000);
  }

  publishEdges();

  keyframes_mutex_.unlock();

  return true;
}

void KeyframeMapper::publishKeyframeData(int i)
{
  RGBDKeyframe& keyframe = keyframes_[i];

  // data transformed to the fixed frame
  PointCloudT keyframe_data_ff; 
  pcl::transformPointCloud(
    keyframe.cloud, keyframe_data_ff, eigenFromTf(keyframe.pose));

  keyframe_data_ff.header.frame_id = fixed_frame_;

  keyframes_pub_.publish(keyframe_data_ff);
}

void KeyframeMapper::publishEdges()
{
  visualization_msgs::Marker marker_edge;
  marker_edge.header.stamp = ros::Time::now();
  marker_edge.header.frame_id = fixed_frame_;
  marker_edge.ns = "consecutive";
  marker_edge.id = 0;
  marker_edge.type = visualization_msgs::Marker::LINE_STRIP;
  marker_edge.action = visualization_msgs::Marker::ADD;

  marker_edge.points.resize(keyframes_.size());
  marker_edge.scale.x = 0.001;

  marker_edge.color.a = 1.0;
  marker_edge.color.r = 1.0;
  marker_edge.color.g = 1.0;
  marker_edge.color.b = 0.0;

  for (unsigned int i = 0; i < keyframes_.size(); ++i)
  {
    RGBDKeyframe& keyframe = keyframes_[i];

    // start point for the edge
    marker_edge.points[i].x = keyframe.pose.getOrigin().getX();  
    marker_edge.points[i].y = keyframe.pose.getOrigin().getY();
    marker_edge.points[i].z = keyframe.pose.getOrigin().getZ();
  }

  edges_pub_.publish(marker_edge);
}

void KeyframeMapper::publishKeyframePose(int i)
{
  boost::mutex::scoped_lock(keyframes_mutex_);

  RGBDKeyframe& keyframe = keyframes_[i];

  // **** publish camera pose

  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = fixed_frame_;
  marker.ns = "keyframe_poses";
  marker.id = i;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;

  marker.points.resize(2);

  // start point for the arrow
  marker.points[0].x = keyframe.pose.getOrigin().getX();
  marker.points[0].y = keyframe.pose.getOrigin().getY();
  marker.points[0].z = keyframe.pose.getOrigin().getZ();

  // end point for the arrow
  tf::Transform ep; 
  ep.setIdentity();
  ep.setOrigin(tf::Vector3(0.00, 0.00, 0.12)); // z = arrow length
  ep = keyframe.pose * ep;

  marker.points[1].x = ep.getOrigin().getX();
  marker.points[1].y = ep.getOrigin().getY();
  marker.points[1].z = ep.getOrigin().getZ(); 
  
  marker.scale.x = 0.02; // shaft radius
  marker.scale.y = 0.05; // head radius

  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  poses_pub_.publish(marker);

  // **** publish frame index text

  visualization_msgs::Marker marker_text;
  marker_text.header.stamp = ros::Time::now();
  marker_text.header.frame_id = fixed_frame_;
  marker_text.ns = "keyframe_indexes";
  marker_text.id = i;
  marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker_text.action = visualization_msgs::Marker::ADD;

  tf::poseTFToMsg(keyframe.pose, marker_text.pose);

  marker_text.pose.position.z -= 0.05;

  char label[6];
  sprintf(label, "%d", i);
  marker_text.text = label;

  marker_text.color.a = 1.0;
  marker_text.color.r = 1.0;
  marker_text.color.g = 1.0;
  marker_text.color.b = 0.0;

  marker_text.scale.z = 0.05; // shaft radius

  poses_pub_.publish(marker_text);
}

bool KeyframeMapper::saveKeyframesSrvCallback(
  Save::Request& request,
  Save::Response& response)
{
  boost::mutex::scoped_lock(keyframes_mutex_);

  ROS_INFO("Saving keyframes...");
  std::string path = request.filename;
  return saveKeyframes(keyframes_, path);
}

bool KeyframeMapper::saveKeyframesFFSrvCallback(
  Save::Request& request,
  Save::Response& response)
{
  boost::mutex::scoped_lock(keyframes_mutex_);

  ROS_INFO("Saving keyframes (in fixed frame)...");
  std::string path = request.filename;
  return saveKeyframes(keyframes_, path, true);
}

bool KeyframeMapper::loadKeyframesSrvCallback(
  Load::Request& request,
  Load::Response& response)
{
  boost::mutex::scoped_lock(keyframes_mutex_);

  ROS_INFO("Loading keyframes...");
  std::string path = request.filename;
  return loadKeyframes(keyframes_, path);
}

bool KeyframeMapper::saveFullSrvCallback(
  Save::Request& request,
  Save::Response& response)
{
  boost::mutex::scoped_lock(keyframes_mutex_);

  ROS_INFO("Saving full map...");
  std::string path = request.filename;
  return saveFullMap(path);
}

bool KeyframeMapper::saveFullMap(const std::string& path)
{
  double vgf_res = 0.01;

  PointCloudT::Ptr full_map(new PointCloudT());
  full_map->header.frame_id = fixed_frame_;

  // aggregate all frames into single cloud
  for (unsigned int kf_idx = 0; kf_idx < keyframes_.size(); ++kf_idx)
  {
    const RGBDKeyframe& keyframe = keyframes_[kf_idx];

    PointCloudT cloud_tf;
    pcl::transformPointCloud(keyframe.cloud, cloud_tf, eigenFromTf(keyframe.pose));
    cloud_tf.header.frame_id = fixed_frame_;

    *full_map += cloud_tf;
  }

  // filter cloud
  PointCloudT full_map_f;
  pcl::VoxelGrid<PointT> vgf;
  vgf.setInputCloud(full_map);
  vgf.setLeafSize(vgf_res, vgf_res, vgf_res);
  vgf.filter(full_map_f);

  // write out
  pcl::PCDWriter writer;
  int result_pcd = writer.writeBinary<PointT>(path + ".pcd", full_map_f);  

  return result_pcd;
}

bool KeyframeMapper::addManualKeyframeSrvCallback(
  AddManualKeyframe::Request& request,
  AddManualKeyframe::Response& response)
{
  manual_add_ = true;

  return true;
}

} // namespace ccny_rgbd
