/*
 * Copyright (c) 2010, Daniel Hewlett, Antons Rebguns
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the <organization> nor the
 *      names of its contributors may be used to endorse or promote products
 *      derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
 *  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **/

/*
 * \file  gazebo_ros_diff_drive.h
 *
 * \brief A differential drive plugin for gazebo. Based on the diffdrive plugin 
 * developed for the erratic robot (see copyright notice above). The original
 * plugin can be found in the ROS package gazebo_erratic_plugins.
 *
 * \author  Piyush Khandelwal (piyushk@gmail.com)
 *
 * $ Id: 06/21/2013 11:23:40 AM piyushk $
 */

#ifndef DIFFDRIVE_PLUGIN_HH
#define DIFFDRIVE_PLUGIN_HH

#include <map>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

// ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/JointState.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

#include <transitbuddy_msgs/PoseWithIDArray.h>
#include <std_msgs/String.h>
namespace gazebo {

  class Joint;
  class Entity;

  class GazeboRosFactory : public WorldPlugin {

    public:
      GazeboRosFactory();
      ~GazeboRosFactory();
      void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf);
      void poseCallback(const transitbuddy_msgs::PoseWithIDArrayConstPtr& msg);
      void commandCallback(const std_msgs::String& msg);

    protected:
      
    private:

      physics::WorldPtr world_;
      boost::shared_ptr<ros::NodeHandle> rosnode_;
      boost::shared_ptr<ros::NodeHandle> n_param_;
      std::string robot_namespace_;
      ros::Subscriber subHumanPose_;
      ros::Subscriber subCommand_;
      void add(int id, const math::Pose& pose);
      boost::interprocess::interprocess_mutex mutex_;
      std::map<int, math::Pose> humansToAdd_;
      std::list<int> humansToRemove_;
      std::list<int> humansInWorld_;
      std::vector<int> humansRemoved_;
      boost::shared_ptr<boost::thread> addThread_;
      boost::shared_ptr<boost::thread> removeThead_;
      boost::shared_ptr<boost::thread> updateThead_;
      void addTheadFnc();
      void removeTheadFnc();
      void updateHumansFnc();
      double offsetX_, offsetY_, offsetAlpha_;
      transitbuddy_msgs::PoseWithIDArray msgHumans;

      physics::ModelPtr getHuman(int id);

      std::string idToName(int id){
        char text[0xF];
        sprintf(text, "h%03i", id);
        return text;
      }
      
  };

}

#endif

