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
#include <OgreVector3.h>
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <rviz/ogre_helpers/shape.h>

#include <ros/ros.h>

#include "point_visual.h"

namespace rviz
{

    PointStampedVisual::PointStampedVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node )
    {
	scene_manager_ = scene_manager;

	// Ogre::SceneNode s form a tree, with each node storing the
	// transform (position and orientation) of itself relative to its
	// parent.  Ogre does the math of combining those transforms when it
	// is time to render.
	//
	// Here we create a node to store the pose of the Point's header frame
	// relative to the RViz fixed frame.
	frame_node_ = parent_node->createChildSceneNode();

	// We create the arrow object within the frame node so that we can
	// set its position and direction relative to its header frame.
	point_ = new rviz::Shape( rviz::Shape::Sphere, scene_manager_, frame_node_ );
    }

    PointStampedVisual::~PointStampedVisual()
    {
	// Delete the arrow to make it disappear.
	delete point_;

	// Destroy the frame node since we don't need it anymore.
	scene_manager_->destroySceneNode( frame_node_ );
    }


void PointStampedVisual::setMessage( const geometry_msgs::PointStamped::ConstPtr& msg )
    {
        Ogre::Vector3 scale( radius_, radius_, radius_ );
        point_->setScale( scale );

        // Set the orientation of the arrow to match the direction of the
        // acceleration vector.
        Ogre::Vector3 point( msg->point.x, msg->point.y, msg->point.z);
        point_->setPosition( point );
    }

    // Position and orientation are passed through to the SceneNode.
    void PointStampedVisual::setFramePosition( const Ogre::Vector3& position )
    {
	frame_node_->setPosition( position );
    }

    void PointStampedVisual::setFrameOrientation( const Ogre::Quaternion& orientation )
    {
	frame_node_->setOrientation( orientation );
    }

    // Color is passed through to the rviz object.
    void PointStampedVisual::setColor( float r, float g, float b, float a )
    {
	point_->setColor( r, g, b, a );
    }

    void PointStampedVisual::setRadius( float r )
    {
        radius_ = r;
    }
} // end namespace rviz

