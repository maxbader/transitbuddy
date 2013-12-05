/*
* Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* * Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.
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

#include "ros/ros.h"
#include <transitbuddy_robot_publisher/robot_publisher_node.h>

int main(int argc, char **argv)
{
        
    ros::init ( argc, argv, "robot_publisher" );
    ros::NodeHandle n;
    RobotPublisherNode bridge ( n );
    ros::Rate rate ( bridge.frequency() );
    while ( ros::ok() ) {
        bridge.publishRobotPose();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

RobotPublisherNode::RobotPublisherNode(ros::NodeHandle & n) : n_ ( n ), n_param_ ( "~" ), frequency_ ( DEFAUTL_FRQ), publish_(false), frame_id_(DEFAULT_FRAME_ID) {
        
    n_param_.getParam ( "frequency", frequency_ );
    ROS_INFO ( "frequency: %5.2f", frequency_ );
    
    n_param_.getParam ( "publish", publish_ );
    ROS_INFO ( "publish: %s", (publish_?"true":"false") );
    
    n_param_.getParam ( "frame_id", frame_id_ );
    ROS_INFO ( "frame_id: %s", frame_id_.c_str() );
    
    sub_ = n_.subscribe( NAME_SUB, 1, &RobotPublisherNode::callbackHumanPose, this );
    pub_ = n_param_.advertise<transitbuddy_msgs::PoseWithIDArray> ( NAME_PUB, 1 );
}

RobotPublisherNode::~RobotPublisherNode(){
}

void RobotPublisherNode::callbackHumanPose(const transitbuddy_msgs::PoseWithIDArray::ConstPtr& msg){
    ROS_INFO ( "robotPoseCallback");
}

void RobotPublisherNode::publishRobotPose(){
    if(publish_ == false) return;
    ROS_INFO ( "publishHumanPose");
    transitbuddy_msgs::PoseWithIDArray poses;
    poses.header.stamp = ros::Time::now();
    poses.header.frame_id = frame_id_;
    poses.poses.resize(2);
    poses.poses[0].id = 10;
    poses.poses[0].valid = true;
    poses.poses[0].pose.position.x = -12.0;
    poses.poses[1].id = 12;
    poses.poses[1].valid = false;
    poses.poses[1].pose.position.x = 24.0;
    pub_.publish(poses);
}
