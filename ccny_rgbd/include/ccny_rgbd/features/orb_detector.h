/**
 *  @file orb_detector.h
 *  @author Ivan Dryanovski <ivan.dryanovski@gmail.com>
 * 
 *  @section LICENSE
 * 
 *  Copyright (C) 2013, City University of New York
 *  CCNY Robotics Lab <http://robotics.ccny.cuny.edu>
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef CCNY_RGBD_ORB_DETECTOR_H
#define CCNY_RGBD_ORB_DETECTOR_H

#include "ccny_rgbd/rgbd_util.h"
#include "ccny_rgbd/features/feature_detector.h"

namespace ccny_rgbd {

/** @brief ORB detector
*/  
class OrbDetector: public FeatureDetector
{
  public:

    /** @brief Constructor from ROS nodehandles
     * @param nh the public nodehandle
     * @param nh_private the private nodehandle
     */    
    OrbDetector(const ros::NodeHandle& nh, 
                const ros::NodeHandle& nh_private);
  
    /** @brief Default destructor
     */    
    ~OrbDetector();

    /** @brief Implementation of the feature detector.
     * @param frame the input frame
     * @param input_img the image for feature detection, derived from the
     *        RGB image of the frame after (optional) blurring
     */ 
    void findFeatures(RGBDFrame& frame, const cv::Mat& input_img);
    
    /** @brief Sets the detection threshold
     * @param threshold the detection threshold
     */
    void setThreshold(int threshold);
    
    /** @brief Sets the number of desired features
     * @param n_features number of desired features
     */
    void setNFeatures(int n_features);
    
    /** @brief Returns the detection threshold
     * @return the detection threshold
     */
    inline int getNFeatures() const;
    
    /** @brief Returns the number of desired features
     * @return the number of desired features
     */
    inline int getThreshold() const;

  private:

    boost::mutex mutex_;    ///< mutex to lock detector during async calls
    int n_features_;        ///< number of desired features
    double threshold_;      ///< threshold for detection
    
    cv::OrbDescriptorExtractor orb_descriptor_; ///< OpenCV feature detector object
    cv::OrbFeatureDetector * orb_detector_;     ///< OpenCV descriptor extractor object
};

} //namespace ccny_rgbd

#endif // CCNY_RGBD_ORB_DETECTOR_H
