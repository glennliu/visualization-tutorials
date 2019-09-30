/*
 * Copyright (c) 2011, Willow Garage, Inc.
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


#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>

#include <math.h>

#include <tf/LinearMath/Vector3.h>


using namespace visualization_msgs;

boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;

std::vector< tf::Vector3 > positions;

void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  switch ( feedback->event_type )
  {
    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
    {
      //compute difference vector for this cube

      tf::Vector3 fb_pos(feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);
      unsigned index = atoi( feedback->marker_name.c_str() );

      if ( index > positions.size() )
      {
        return;
      }
      tf::Vector3 fb_delta = fb_pos - positions[index];

      // move all markers in that direction
      for ( unsigned i=0; i<positions.size(); i++ )
      {
        float d = fb_pos.distance( positions[i] );
        float t = 1 / (d*5.0+1.0) - 0.2;
        if ( t < 0.0 ) t=0.0;

        positions[i] += t * fb_delta;

        if ( i == index ) {
          ROS_INFO_STREAM( d );
          positions[i] = fb_pos;
        }

        geometry_msgs::Pose pose;
        pose.position.x = positions[i].x();
        pose.position.y = positions[i].y();
        pose.position.z = positions[i].z();

        std::stringstream s;
        s << i;
        server->setPose( s.str(), pose );
      }


      break;
    }
  }
  server->applyChanges();
}

InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg )
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.orientation_mode = InteractiveMarkerControl::VIEW_FACING;
  control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
  control.independent_marker_orientation = true;

  Marker marker;

  marker.type = Marker::CUBE;
  marker.scale.x = msg.scale;
  marker.scale.y = msg.scale;
  marker.scale.z = msg.scale;
  marker.color.r = 5;//+0.7*msg.pose.position.x;
  marker.color.g = 0.1;//+0.7*msg.pose.position.y;
  marker.color.b = 0.1;//+0.7*msg.pose.position.z;
  marker.color.a = 1.0;
//  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

  control.markers.push_back( marker );
  msg.controls.push_back( control );

  return msg.controls.back();
}


void makeCube( )
{
    int side_length = 10;
    float step = 1.0/ (float)side_length;
    int count = 4;

    positions.reserve( side_length*side_length*side_length );


    std::vector  <InteractiveMarker> objects;

    InteractiveMarker int_marker;
    int_marker.header.frame_id = "world";
    int_marker.scale = 0.3;
    positions.push_back( tf::Vector3(1,1,1) );
    std::stringstream s;

//      objects 1
    s << 0;
    int_marker.name = s.str();
    int_marker.description = "1";

    int_marker.pose.position.x = 1.98;
    int_marker.pose.position.y = -2.15;
    int_marker.pose.position.z = 0.8;

    objects.push_back(int_marker);
    makeBoxControl(objects[0]);
    server->insert( objects[0] );
    server->setCallback( objects[0].name, &processFeedback );

//      object 2
    s <<1;
    int_marker.name = s.str();
    int_marker.description = "2";
    int_marker.pose.position.x = 4.9;
    int_marker.pose.position.y = -2.3;
    int_marker.pose.position.z = 1.5;

    objects.push_back(int_marker);
    makeBoxControl(objects[1]);
    server->insert( objects[1] );
    server->setCallback( objects[1].name, &processFeedback );

    //      object 3
    s <<2;
    int_marker.name = s.str();
    int_marker.description = "3";
    int_marker.pose.position.x = 3.9;
    int_marker.pose.position.y = -0.46;
    int_marker.pose.position.z = 0.8;

    objects.push_back(int_marker);
    makeBoxControl(objects[2]);
    server->insert( objects[2] );
    server->setCallback( objects[2].name, &processFeedback );

    //      object 4
    s <<3;
    int_marker.name = s.str();
    int_marker.description = "4";
    int_marker.pose.position.x = 6.3;
    int_marker.pose.position.y = 0.55;
    int_marker.pose.position.z = 1.5;

    objects.push_back(int_marker);
    makeBoxControl(objects[3]);
    server->insert( objects[3] );
    server->setCallback( objects[3].name, &processFeedback );

    //      object 5
    s <<4;
    int_marker.name = s.str();
    int_marker.description = "5";
    int_marker.pose.position.x = 0.65;
    int_marker.pose.position.y = 1.86;
    int_marker.pose.position.z = 1.2;

    objects.push_back(int_marker);
    makeBoxControl(objects[4]);
    server->insert( objects[4] );
    server->setCallback( objects[4].name, &processFeedback );


}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cube");

  server.reset( new interactive_markers::InteractiveMarkerServer("cube") );

  ros::Duration(0.1).sleep();

  ROS_INFO("initializing..");
  makeCube();
  server->applyChanges();
  ROS_INFO("ready.");

  ros::spin();

  server.reset();
}
