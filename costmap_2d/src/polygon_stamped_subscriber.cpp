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
#include <costmap_2d/polygon_stamped_subscriber.h>

namespace costmap_2d
{

PolygonStampedSubscriber::PolygonStampedSubscriber( ros::NodeHandle nh,
                                                    std::string topic_name,
                                                    int buffer_length,
                                                    std::string target_frame,
                                                    tf::TransformListener* tf,
                                                    const boost::function<void (const geometry_msgs::Polygon&)>& callback )
  : tf_( tf )
  , target_frame_( target_frame )
  , tf_filter_( subscriber_, *tf_, target_frame, buffer_length )
  , callback_( callback )
{
  subscriber_.subscribe( nh, topic_name, buffer_length );
  tf_filter_.registerCallback( boost::bind( &PolygonStampedSubscriber::handleMessage, this, _1 ));
}

void PolygonStampedSubscriber::handleMessage( const boost::shared_ptr<const geometry_msgs::PolygonStamped>& msg )
{
  polygon_.points.resize( msg->polygon.points.size() );

  tf::StampedTransform transform;
  try
  {
    tf_->lookupTransform( target_frame_, msg->header.frame_id, msg->header.stamp, transform );
  }
  catch( tf::TransformException ex )
  {
    ROS_ERROR( "PolygonStampedSubscriber: failed to lookup transform from %s to %s when called from tf::MessageFilter.  Exception: %s",
               msg->header.frame_id.c_str(), target_frame_.c_str(), ex.what() );
    return;
  }
  tf::Point point_in, point_out;
  for( unsigned int i = 0; i < msg->polygon.points.size(); i++ )
  {
    point_in.setX( msg->polygon.points[ i ].x );
    point_in.setY( msg->polygon.points[ i ].y );
    point_in.setZ( msg->polygon.points[ i ].z );

    point_out = transform * point_in;

    polygon_.points[ i ].x = point_out.x();
    polygon_.points[ i ].y = point_out.y();
    polygon_.points[ i ].z = point_out.z();
  }

  if( callback_ )
  {
    callback_( polygon_ );
  }
}

} // end namespace costmap_2d
