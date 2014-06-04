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
#include <transitbuddy_msgs/LineWithIDArray.h>
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
      void humanCallback(const transitbuddy_msgs::PoseWithIDArrayConstPtr& msg);
      void wallsCallback(const transitbuddy_msgs::LineWithIDArrayConstPtr& msg);
      void commandCallback(const std_msgs::String& msg);
      void callbackParameters ( gazebo_human_receiver::human_receiverConfig &config, uint32_t level );

    protected:
      
    private:

      physics::WorldPtr world_;
      boost::shared_ptr<ros::NodeHandle> rosnode_;
      boost::shared_ptr<ros::NodeHandle> n_param_;
      std::string robot_namespace_;
      ros::Subscriber subHumanPose_;
      ros::Subscriber subWall_;
      ros::Subscriber subCommand_;
      dynamic_reconfigure::Server<gazebo_human_receiver::human_receiverConfig> reconfigureServer_;
      dynamic_reconfigure::Server<gazebo_human_receiver::human_receiverConfig>::CallbackType reconfigureFnc_;  
      void createHuman(const std::string &name, const math::Pose& pose);
      void addWall(int id, double x0, double y0, double x1, double y1);
      void addRoomModel(const transitbuddy_msgs::LineWithIDArray &walls);
      void addWallModel(const transitbuddy_msgs::LineWithID &wall);
      boost::interprocess::interprocess_mutex mutexHumans_;
      boost::interprocess::interprocess_mutex mutexWall_;
      std::vector<physics::ModelPtr> humansInactive_;
      std::map<int,physics::ModelPtr> humansActive_;
      transitbuddy_msgs::LineWithIDArray walls_;
      boost::shared_ptr<boost::thread> updateThead_;
      boost::shared_ptr<boost::thread> wallThead_;
      boost::shared_ptr<boost::thread> createHumansThead_;
      void updateHumansFnc();
      void wallTheadFnc();
      void createHumansFnc();
      double map_offset_x_, map_offset_y_, map_offset_angle_;
      int max_humans_;
      double min_distance_between_humans_;
      std::string human_template_file_;
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

