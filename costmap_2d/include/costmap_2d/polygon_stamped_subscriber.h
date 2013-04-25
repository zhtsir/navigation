/*
 * Copyright (c) 2013, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef POLYGON_STAMPED_SUBSCRIBER_H
#define POLYGON_STAMPED_SUBSCRIBER_H

#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>
#include <geometry_msgs/PolygonStamped.h>
#include <string>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

namespace tf
{
class TransformListener;
}

namespace costmap_2d
{

/** @brief Subscriber class for geometry_msgs::PolygonStamped which
 * uses a tf::MessageFilter to transform the polygon into a desired
 * frame. */
class PolygonStampedSubscriber
{
public:
  PolygonStampedSubscriber( ros::NodeHandle nh,
                            std::string topic_name,
                            int buffer_length,
                            std::string target_frame,
                            tf::TransformListener* tf,
                            const boost::function<void (const geometry_msgs::Polygon&)>& callback );
private:
  void handleMessage( const boost::shared_ptr<const geometry_msgs::PolygonStamped>& msg );

  tf::TransformListener* tf_;
  std::string target_frame_;
  message_filters::Subscriber<geometry_msgs::PolygonStamped> subscriber_;
  tf::MessageFilter<geometry_msgs::PolygonStamped> tf_filter_;
  geometry_msgs::Polygon polygon_;
  boost::function<void (const geometry_msgs::Polygon&)> callback_;  
};

} // end namespace costmap_2d

#endif // POLYGON_STAMPED_SUBSCRIBER_H
