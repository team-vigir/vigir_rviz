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
#ifndef POINT_VISUAL_H
#define POINT_VISUAL_H

#include <geometry_msgs/PointStamped.h>

namespace Ogre
{
    class Vector3;
    class Quaternion;
}

namespace rviz
{
    class Shape;
}

namespace rviz
{


// Each instance of PointStampedVisual represents the visualization of a single
// sensor_msgs::Point message.  Currently it just shows an arrow with
// the direction and magnitude of the acceleration vector, but could
// easily be expanded to include more of the message data.
class PointStampedVisual
{
public:
    // Constructor.  Creates the visual stuff and puts it into the
    // scene, but in an unconfigured state.
    PointStampedVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node );

    // Destructor.  Removes the visual stuff from the scene.
    virtual ~PointStampedVisual();

    // set rainbow color
    void getRainbowColor(float value, Ogre::ColourValue& color);
    // Configure the visual to show the data in the message.
    void setMessage( const geometry_msgs::PointStamped::ConstPtr& msg  );

    // Set the pose of the coordinate frame the message refers to.
    // These could be done inside setMessage(), but that would require
    // calls to FrameManager and error handling inside setMessage(),
    // which doesn't seem as clean.  This way PointStampedVisual is only
    // responsible for visualization.
    void setFramePosition( const Ogre::Vector3& position );
    void setFrameOrientation( const Ogre::Quaternion& orientation );

    // Set the color and alpha of the visual, which are user-editable
    // parameters and therefore don't come from the Point message.
    void setColor( float r, float g, float b, float a );

    void setRadius( float r );

private:
    // The object implementing the point circle
    rviz::Shape* point_;

    // A SceneNode whose pose is set to match the coordinate frame of
    // the Point message header.
    Ogre::SceneNode* frame_node_;

    // The SceneManager, kept here only so the destructor can ask it to
    // destroy the ``frame_node_``.
    Ogre::SceneManager* scene_manager_;

    float radius_;

};

} // end namespace rviz

#endif // POINT_VISUAL_H
