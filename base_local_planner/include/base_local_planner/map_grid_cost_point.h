/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Eric Perko
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Eric Perko nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#ifndef MAP_GRID_COST_POINT_H_
#define MAP_GRID_COST_POINT_H_

#include <sensor_msgs/point_cloud2_iterator.h>

namespace base_local_planner {
  inline
  void createMapGridCostPointCloud(boost::shared_ptr<sensor_msgs::PointCloud2>& cloud,
                                   boost::shared_ptr<sensor_msgs::PointCloud2Modifier>& modifier)
  {
    if (!cloud)
      cloud.reset(new sensor_msgs::PointCloud2());
    modifier.reset(new sensor_msgs::PointCloud2Modifier(*cloud));
    modifier->setPointCloud2Fields(7, "x", 1, sensor_msgs::PointField::FLOAT32,
                                   "y", 1, sensor_msgs::PointField::FLOAT32,
                                   "z", 1, sensor_msgs::PointField::FLOAT32,
                                   "path_cost", 1, sensor_msgs::PointField::FLOAT32,
                                   "goal_cost", 1, sensor_msgs::PointField::FLOAT32,
                                   "occ_cost", 1, sensor_msgs::PointField::FLOAT32,
                                   "total_cost", 1, sensor_msgs::PointField::FLOAT32);
  };
}

#endif
