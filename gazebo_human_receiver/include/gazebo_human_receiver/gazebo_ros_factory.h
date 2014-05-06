#ifndef GAZEBO_ROS_HUMAN_RECEIVER_HH
#define GAZEBO_ROS_HUMAN_RECEIVER_HH

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
#include <dynamic_reconfigure/server.h>

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

#include <gazebo_human_receiver/human_receiverConfig.h>

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
      void callbackParameters ( gazebo_human_receiver::human_receiverConfig &config, uint32_t level );

    protected:
      
    private:

      physics::WorldPtr world_;
      boost::shared_ptr<ros::NodeHandle> rosnode_;
      boost::shared_ptr<ros::NodeHandle> n_param_;
      std::string robot_namespace_;
      ros::Subscriber subHumanPose_;
      ros::Subscriber subCommand_;
      dynamic_reconfigure::Server<gazebo_human_receiver::human_receiverConfig> reconfigureServer_;
      dynamic_reconfigure::Server<gazebo_human_receiver::human_receiverConfig>::CallbackType reconfigureFnc_;  
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
      double map_offset_x_, map_offset_y_, map_offset_angle_;
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
