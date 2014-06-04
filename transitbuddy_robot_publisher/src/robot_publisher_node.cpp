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

RobotPublisherNode::RobotPublisherNode(ros::NodeHandle & n) : n_ ( n ), n_param_ ( "~" ), frequency_ ( DEFAUTL_FRQ), publish_(true), frame_id_(DEFAULT_FRAME_ID) {

    offset_map_.position.x = OFFSET_X;
    offset_map_.position.y = OFFSET_Y;

    if(!n_param_.getParam ( "ox", offset_map_.position.x )) {
        n_param_.setParam ( "ox", offset_map_.position.x);
    }
    ROS_INFO ( "ox: %5.2f", offset_map_.position.x);
    if(!n_param_.getParam ( "oy", offset_map_.position.y )) {
        n_param_.setParam ( "oy", offset_map_.position.y);
    }
    ROS_INFO ( "oy: %5.2f", offset_map_.position.y);
    n_param_.getParam ( "frequency", frequency_ );
    ROS_INFO ( "frequency: %5.2f", frequency_ );

    n_param_.getParam ( "publish", publish_ );
    ROS_INFO ( "publish: %s", (publish_?"true":"false") );

    n_param_.getParam ( "frame_id", frame_id_ );
    ROS_INFO ( "frame_id: %s", frame_id_.c_str() );

    robotPoses_.header.seq = 0;
    sub_ = n_.subscribe( NAME_SUB_HUMANS, 1, &RobotPublisherNode::callbackHumanPose, this );
    sub_ = n_.subscribe( NAME_SUB_GAZEBO_MODELS, 1, &RobotPublisherNode::callbackGazeboModel, this );
    pub_ = n_param_.advertise<transitbuddy_msgs::PoseWithIDArray> ( NAME_PUB_ROBOTS, 1 );
}

RobotPublisherNode::~RobotPublisherNode() {
}

void RobotPublisherNode::callbackHumanPose(const transitbuddy_msgs::PoseWithIDArray::ConstPtr& msg) {
    ROS_INFO ( "%i human pose received: ", (int) msg->poses.size());
}

void RobotPublisherNode::callbackGazeboModel(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    n_param_.getParam ( "ox", offset_map_.position.x );
    n_param_.getParam ( "oy", offset_map_.position.x );
    robotPoses_.poses.clear();
    for(int i = 0; i < msg->name.size(); i++) {
        std::string name( msg->name[i]);
        transitbuddy_msgs::PoseWithID robot;
        robot.pose.position.x =  msg->pose[i].position.x + offset_map_.position.x;
        robot.pose.position.y =  msg->pose[i].position.y + offset_map_.position.y;
        robot.pose.position.z = 0;
        robot.valid = true;
        if (name.compare("r1") == 0) {
            robot.id = 1;
            ROS_INFO ( "robot: %s, %f,%f,%f", name.c_str(), robot.pose.position.x, robot.pose.position.y, robot.pose.position.z );
            robotPoses_.poses.push_back(robot);
        }
        if (name.compare("r2") == 0) {
            robot.id = 2;
            ROS_INFO ( "robot: %s, %f,%f,%f", name.c_str(), robot.pose.position.x, robot.pose.position.y, robot.pose.position.z );
            robotPoses_.poses.push_back(robot);
        }
    }

}
void RobotPublisherNode::publishRobotPose() {
    if(publish_ == false) return;
    robotPoses_.header.seq++;
    if(robotPoses_.header.seq % 10 == 0) ROS_INFO ( "publish robot pose %i", robotPoses_.header.seq);
    robotPoses_.header.stamp = ros::Time::now();
    robotPoses_.header.frame_id = frame_id_;
    pub_.publish(robotPoses_);
}
