/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013-2015, Team ViGIR ( TORC Robotics LLC, TU Darmstadt, Virginia Tech, Oregon State University, Cornell University, and Leibniz University Hanover )
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
 *   * Neither the name of Team ViGIR, TORC Robotics, nor the names of its
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
//@TODO_ADD_AUTHOR_INFO
#include "ros/ros.h"

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "interactive_markers/interactive_marker_server.h"

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

interactive_markers::InteractiveMarkerServer* server;

void makeMarker( )
{
  // create an interactive marker for our server
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "/rviz_logo";
  int_marker.name = "R";

  int_marker.pose.orientation.x = 0.0;
  int_marker.pose.orientation.y = 0.0;
  int_marker.pose.orientation.z = 1.0;
  int_marker.pose.orientation.w = 1.0;
  int_marker.pose.position.y = - 2.0;
  int_marker.scale = 2.3;

  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;

  tf::quaternionTFToMsg( tf::Quaternion( tf::Vector3(0, 0, 1), 0.2 ), marker.pose.orientation );

  marker.pose.position.x = 0;
  marker.pose.position.y = -0.22;
  marker.pose.position.z = 0;
  marker.scale.x = marker.scale.y = marker.scale.z = 4;
  marker.color.r = 0;
  marker.color.g = 0;
  marker.color.b = 0;
  marker.color.a = 0;
  marker.mesh_resource = "package://rviz/image_src/R.stl";
  marker.mesh_use_embedded_materials = true;
  marker.id = 0;

  // create a non-interactive control which contains the box
  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( marker );

  // add the control to the interactive marker
  int_marker.controls.push_back( control );

  visualization_msgs::InteractiveMarkerControl linear_control;
  linear_control.name = "rotate_z";
  linear_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  linear_control.orientation.y = 1;
  linear_control.orientation.w = 1;

  // add the control to the interactive marker
  int_marker.controls.push_back(linear_control);

  server->insert(int_marker);

  int_marker.name = "Viz";

  marker.mesh_resource = "package://rviz/image_src/Viz.stl";
  marker.pose.position.x = 3.3;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  control.markers.clear();
  control.markers.push_back( marker );

  // add the control to the interactive marker
  int_marker.controls.clear();
  int_marker.controls.push_back( control );

  server->insert(int_marker);


  server->applyChanges();
}

void publishCallback(tf::TransformBroadcaster& tf_broadcaster, const ros::TimerEvent&)
{
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(3, 1, 0) );
  transform.setRotation( tf::createQuaternionFromRPY(0, 0, M_PI * 0.9) );
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "rviz_logo"));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rviz_logo_marker");
  ros::NodeHandle n;
  tf::TransformBroadcaster tf_broadcaster;

  ros::Timer publish_timer = n.createTimer(ros::Duration(0.1), boost::bind(&publishCallback,tf_broadcaster,_1));

  server = new interactive_markers::InteractiveMarkerServer("rviz_logo");
  makeMarker( );

  ros::spin();
}
