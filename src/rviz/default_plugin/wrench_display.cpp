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
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <rviz/visualization_manager.h>
#include <rviz/frame_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/parse_color.h>
#include <rviz/validate_floats.h>

#include <boost/foreach.hpp>

#include "wrench_visual.h"

#include "wrench_display.h"

namespace rviz
{

    WrenchStampedDisplay::WrenchStampedDisplay()
    {
	force_color_property_ =
	    new rviz::ColorProperty( "Force Color", QColor( 204, 51, 51 ),
                                     "Color to draw the force arrows.",
                                     this, SLOT( updateColorAndAlpha() ));

	torque_color_property_ =
	    new rviz::ColorProperty( "Torque Color", QColor( 204, 204, 51),
                                     "Color to draw the torque arrows.",
                                     this, SLOT( updateColorAndAlpha() ));

	alpha_property_ =
            new rviz::FloatProperty( "Alpha", 1.0,
                                     "0 is fully transparent, 1.0 is fully opaque.",
                                     this, SLOT( updateColorAndAlpha() ));

	force_scale_property_ =
            new rviz::FloatProperty( "Force Arrow Scale", 2.0,
                                     "force arrow scale",
                                     this, SLOT( updateColorAndAlpha() ));

        torque_scale_property_ =
            new rviz::FloatProperty( "Torque Arrow Scale", 2.0,
                                     "torque arrow scale",
                                     this, SLOT( updateColorAndAlpha() ));            

	width_property_ =
            new rviz::FloatProperty( "Arrow Width", 0.5,
                                     "arrow width",
                                     this, SLOT( updateColorAndAlpha() ));


	history_length_property_ =
	    new rviz::IntProperty( "History Length", 1,
                                   "Number of prior measurements to display.",
                                   this, SLOT( updateHistoryLength() ));

        history_length_property_->setMin( 1 );
        history_length_property_->setMax( 100000 );
    }

    void WrenchStampedDisplay::onInitialize()
    {
        MFDClass::onInitialize();
	updateHistoryLength( );
    }

    WrenchStampedDisplay::~WrenchStampedDisplay()
    {
    }

    // Override rviz::Display's reset() function to add a call to clear().
    void WrenchStampedDisplay::reset()
    {
        MFDClass::reset();
	visuals_.clear();
    }

    void WrenchStampedDisplay::updateColorAndAlpha()
    {
        float alpha = alpha_property_->getFloat();
        float force_scale = force_scale_property_->getFloat();
        float torque_scale = torque_scale_property_->getFloat();
        float width = width_property_->getFloat();
        Ogre::ColourValue force_color = force_color_property_->getOgreColor();
        Ogre::ColourValue torque_color = torque_color_property_->getOgreColor();

        for( size_t i = 0; i < visuals_.size(); i++ )
	{
            visuals_[i]->setForceColor( force_color.r, force_color.g, force_color.b, alpha );
            visuals_[i]->setTorqueColor( torque_color.r, torque_color.g, torque_color.b, alpha );
            visuals_[i]->setForceScale( force_scale );
            visuals_[i]->setTorqueScale( torque_scale );
            visuals_[i]->setWidth( width );
	}
    }

    // Set the number of past visuals to show.
    void WrenchStampedDisplay::updateHistoryLength()
    {
        visuals_.rset_capacity(history_length_property_->getInt());
    }

    bool validateFloats( const geometry_msgs::WrenchStamped& msg )
    {
        return rviz::validateFloats(msg.wrench.force) && rviz::validateFloats(msg.wrench.torque) ;
    }

    // This is our callback to handle an incoming message.
    void WrenchStampedDisplay::processMessage( const geometry_msgs::WrenchStamped::ConstPtr& msg )
    {
        if( !validateFloats( *msg ))
            {
                setStatus( rviz::StatusProperty::Error, "Topic", "Message contained invalid floating point values (nans or infs)" );
                return;
            }

	// Here we call the rviz::FrameManager to get the transform from the
	// fixed frame to the frame in the header of this Imu message.  If
	// it fails, we can't do anything else so we return.
	Ogre::Quaternion orientation;
	Ogre::Vector3 position;
	if( !context_->getFrameManager()->getTransform( msg->header.frame_id,
                                                        msg->header.stamp,
                                                        position, orientation ))
	  {
	    ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'",
		       msg->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
	    return;
	  }

        // We are keeping a circular buffer of visual pointers.  This gets
        // the next one, or creates and stores it if the buffer is not full
        boost::shared_ptr<WrenchStampedVisual> visual;
        if( visuals_.full() )
            {
                visual = visuals_.front();
            }
        else
            {
                visual.reset(new WrenchStampedVisual( context_->getSceneManager(), scene_node_ ));
            }

	// Now set or update the contents of the chosen visual.
	visual->setMessage( msg );
	visual->setFramePosition( position );
	visual->setFrameOrientation( orientation );
        float alpha = alpha_property_->getFloat();
        float force_scale = force_scale_property_->getFloat();
        float torque_scale = torque_scale_property_->getFloat();
        float width = width_property_->getFloat();
        Ogre::ColourValue force_color = force_color_property_->getOgreColor();
        Ogre::ColourValue torque_color = torque_color_property_->getOgreColor();
	visual->setForceColor( force_color.r, force_color.g, force_color.b, alpha );
	visual->setTorqueColor( torque_color.r, torque_color.g, torque_color.b, alpha );
        visual->setForceScale( force_scale );
        visual->setTorqueScale( torque_scale );
        visual->setWidth( width );

        // And send it to the end of the circular buffer
        visuals_.push_back(visual);
    }

} // end namespace rviz

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rviz::WrenchStampedDisplay, rviz::Display )



