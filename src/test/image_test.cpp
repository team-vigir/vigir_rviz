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

#include "sensor_msgs/Image.h"

int main( int argc, char** argv )
{
  ros::init( argc, argv, "image_test" );

  ros::NodeHandle n;

  ros::Publisher rgb_pub = n.advertise<sensor_msgs::Image>( "red_image", 0 );

  ros::Duration(0.1).sleep();

  sensor_msgs::Image red_image;
  red_image.header.frame_id = "/base_link";
  red_image.header.stamp = ros::Time::now();
  red_image.height = 100;
  red_image.width = 100;
  red_image.encoding = "rgb8";
  red_image.step = 3 * red_image.height;

  red_image.data.resize(3 * red_image.height * red_image.width);
  for (uint32_t i = 0; i < 3 * red_image.height * red_image.width; ++i)
  {
    if (i % 3 == 0)
    {
      red_image.data[i] = 255;
    }
    else
    {
      red_image.data[i] = 0;
    }
  }

  int i = 0;
  while (n.ok())
  {
    ROS_INFO("Publishing");

    rgb_pub.publish(red_image);

    ++i;

    ros::Duration(1.0).sleep();
  }
}
