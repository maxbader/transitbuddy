
#include <gazebo_human_receiver/gazebo_ros_factory.h>
#include <gazebo_human_receiver/gazebo_model_templates.h>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

namespace gazebo
{

// Constructor
GazeboRosFactory::GazeboRosFactory(): reconfigureServer_(ros::NodeHandle("GazeboRosFactory")) {}

// Destructor
GazeboRosFactory::~GazeboRosFactory() {}

void GazeboRosFactory::callbackParameters ( gazebo_human_receiver::human_receiverConfig &config, uint32_t level ) {
    map_offset_x_ = config.map_offset_x;
    map_offset_y_ = config.map_offset_y;
    map_offset_angle_ = config.map_offset_angle;
}

void GazeboRosFactory::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
    this->world_ = _parent;
    rosnode_ = boost::shared_ptr<ros::NodeHandle> ( new ros::NodeHandle () );
    n_param_ = boost::shared_ptr<ros::NodeHandle>(new ros::NodeHandle("~"));

    this->max_humans_=100;
    if (!_sdf->HasElement("max_humans")) {
        ROS_WARN("GazeboRosFactory Plugin (ns = %s) missing <max_humans>, defaults to %i",
                 this->robot_namespace_.c_str(), this->max_humans_);
    } else {
        this->max_humans_ = _sdf->GetElement("max_humans")->Get<int>();
    }
    this->min_distance_between_humans_=0.5;
    if (!_sdf->HasElement("min_distance_between_humans")) {
        ROS_WARN("GazeboRosFactory Plugin (ns = %s) missing <min_distance_between_humans>, defaults to %f",
                 this->robot_namespace_.c_str(), this->min_distance_between_humans_);
    } else {
        this->min_distance_between_humans_ = _sdf->GetElement("min_distance_between_humans")->Get<double>();
    }
    
    if (!_sdf->HasElement("human_template_file")) {
        ROS_ERROR("GazeboRosFactory Plugin (ns = %s) missing <human_template_file_>",
                 this->robot_namespace_.c_str());
    } else {
        this->human_template_file_ = _sdf->GetElement("human_template_file")->Get<std::string>();
    }
    
    ROS_INFO ( "GazeboRosFactory");
    subHumanPose_ = rosnode_->subscribe( "/human_publisher/human_pose", 1, &GazeboRosFactory::humanCallback, this );
    subCommand_ = rosnode_->subscribe( "/transitbuddy_dummy/command", 1, &GazeboRosFactory::commandCallback, this );
    // subWall_ = rosnode_->subscribe( "/human_publisher/walls", 1, &GazeboRosFactory::wallsCallback, this );

   updateThead_ = boost::shared_ptr<boost::thread>(new boost::thread(&GazeboRosFactory::updateHumansFnc, this));
    createHumansThead_ = boost::shared_ptr<boost::thread>(new boost::thread(&GazeboRosFactory::createHumansFnc, this));
    reconfigureFnc_ = boost::bind(&GazeboRosFactory::callbackParameters, this,  _1, _2);
    reconfigureServer_.setCallback(reconfigureFnc_);

    wallThead_ = boost::shared_ptr<boost::thread>(new boost::thread(&GazeboRosFactory::wallTheadFnc, this));
}

void GazeboRosFactory::createHumansFnc() {
    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(mutexHumans_);
    sleep(2);
    int cols =  ceil(sqrt(max_humans_));
    for(std::size_t id = 0; id < max_humans_; id++) {
        int r = id / cols;
        int c = id % cols;
        math::Pose pose(min_distance_between_humans_*r, min_distance_between_humans_*c, 0, 0, 0, 0);
        std::string name = idToName(id);
        createHuman(name, pose);
    }
    ROS_INFO ( "create  %i humans:", max_humans_);
    sleep(2);
    humansInactive_.resize(max_humans_);
    for(std::size_t id = 0; id < humansInactive_.size(); id ++) {
        std::string name = idToName(id);
        physics::ModelPtr p  = this->world_->GetModel(name);
        if(p) {
            humansInactive_[id] = p;
        } else {
            ROS_INFO ( "createHumansFnc: Could not get pointer to %s", name.c_str());
        }
    }
    ROS_INFO ( "indexed humans:");
}

void GazeboRosFactory::wallTheadFnc() {
    double loop = true;
    while(loop) {
        sleep(2);
        if(walls_.lines.size() > 0)
        {
            loop = false;
            boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock();

            // addRoomModel(walls_);
            for(unsigned int i = 0; i < walls_.lines.size(); i++) {
                addWallModel(walls_.lines[i]);
            }
            ROS_INFO ( "wallTheadFnc");
        }
    }
}

void GazeboRosFactory::addWallModel(const transitbuddy_msgs::LineWithID &_wall) {
    std::stringstream ss;
    double dx = _wall.p[1].x - _wall.p[0].x;
    double dy = _wall.p[1].y - _wall.p[0].y;
    double l = sqrt(dx*dx + dy*dy);
    double a = atan2(dy,dx);
    double w = 0.2;
    double h = 1.5;
    double h2 = h*2;
    double x = _wall.p[0].x + dx/2.;
    double y = _wall.p[0].y + dy/2.;
    std::string modelStr("\
          <sdf version ='1.4'>\n\
            <model name ='${model}'>\n\
              <static>true</static>\n\
              <link name ='${link}'>\n\
              <pose>${x} ${y} ${h} 0 0 ${a}</pose>\n\
              <collision name ='collision'>\n\
                <pose>0 0 0 0 0 0</pose>\
                <geometry>\n\
                  <box><size>${l} ${w} ${h2}</size></box>\n\
                </geometry>\n\
              </collision>\n\
              <visual name ='${visual}'>\n\
                <pose>0 0 0 0 0 0</pose>\n\
                <geometry>\n\
                  <box><size>${l} ${w} ${h2}</size></box>\n\
                </geometry>\n\
              </visual>\n\
            </link>\n\
          </model>\n\
        </sdf>\n");
    boost::replace_all(modelStr, "${model}", std::string("w") + boost::lexical_cast<std::string>(_wall.id));
    boost::replace_all(modelStr, "${link}",  std::string("l") + boost::lexical_cast<std::string>(_wall.id));
    boost::replace_all(modelStr, "${visual}",  std::string("v") + boost::lexical_cast<std::string>(_wall.id));
    boost::replace_all(modelStr, "${x}", boost::lexical_cast<std::string>(x));
    boost::replace_all(modelStr, "${y}", boost::lexical_cast<std::string>(y));
    boost::replace_all(modelStr, "${dx}", boost::lexical_cast<std::string>(dx));
    boost::replace_all(modelStr, "${dy}", boost::lexical_cast<std::string>(dy));
    boost::replace_all(modelStr, "${h}", boost::lexical_cast<std::string>(h));
    boost::replace_all(modelStr, "${l}", boost::lexical_cast<std::string>(l));
    boost::replace_all(modelStr, "${w}", boost::lexical_cast<std::string>(w));
    boost::replace_all(modelStr, "${h2}", boost::lexical_cast<std::string>(h2));
    boost::replace_all(modelStr, "${a}", boost::lexical_cast<std::string>(a));

    sdf::SDF sdfModel;
    sdfModel.SetFromString(modelStr);
    this->world_->InsertModelSDF(sdfModel);
}

void GazeboRosFactory::addRoomModel(const transitbuddy_msgs::LineWithIDArray &_walls) {
    std::stringstream ss;
    ss << "<sdf version ='1.4'>" << std::endl;
    ss << "  <model name ='walls'>" << std::endl;
    ss << "    <static>true</static>" << std::endl;
    ss << "    <link name ='walls'>" << std::endl;
    ss << "      <pose>0 0 .1 0 0 0</pose>" << std::endl;
    for(unsigned int i = 0; i < _walls.lines.size(); i++) {
        double dx = _walls.lines[i].p[1].x - _walls.lines[i].p[0].x;
        double dy = _walls.lines[i].p[1].y - _walls.lines[i].p[0].y;
        double l = sqrt(dx*dx + dy*dy);
        double a = atan2(dy,dx);
        double w = 0.2;
        double h = 1.5;
        double h2 = h*2;
        double x = _walls.lines[i].p[0].x + dx/2.;
        double y = _walls.lines[i].p[0].y + dy/2.;
        std::string modelStr("\n\
              <collision name ='collision'>\n\
                <pose>${x} ${y} ${h} 0 0 ${a}</pose>\n\
                <geometry>\n\
                  <box><size>${l} ${w} ${h2}</size></box>\n\
                </geometry>\n\
              </collision>\n\
              <visual name ='${visual}'>\n\
                <pose>${x} ${y} ${h} 0 0 ${a}</pose>\n\
                <geometry>\n\
                  <box><size>${l} ${w} ${h2}</size></box>\n\
                </geometry>\n\
              </visual>\n");
        boost::replace_all(modelStr, "${joint}", std::string("j") + boost::lexical_cast<std::string>(_walls.lines[i].id));
        boost::replace_all(modelStr, "${link}",  std::string("l") + boost::lexical_cast<std::string>(_walls.lines[i].id));
        boost::replace_all(modelStr, "${visual}",  std::string("v") + boost::lexical_cast<std::string>(_walls.lines[i].id));
        boost::replace_all(modelStr, "${x}", boost::lexical_cast<std::string>(x));
        boost::replace_all(modelStr, "${y}", boost::lexical_cast<std::string>(y));
        boost::replace_all(modelStr, "${dx}", boost::lexical_cast<std::string>(dx));
        boost::replace_all(modelStr, "${dy}", boost::lexical_cast<std::string>(dy));
        boost::replace_all(modelStr, "${h}", boost::lexical_cast<std::string>(h));
        boost::replace_all(modelStr, "${l}", boost::lexical_cast<std::string>(l));
        boost::replace_all(modelStr, "${w}", boost::lexical_cast<std::string>(w));
        boost::replace_all(modelStr, "${h2}", boost::lexical_cast<std::string>(h2));
        boost::replace_all(modelStr, "${a}", boost::lexical_cast<std::string>(a));
        ss << modelStr;
    }
    ss << "    </link>" << std::endl;
    ss << "  </model>" << std::endl;
    ss << "</sdf>" << std::endl;
    sdf::SDF sdfModel;
    std::string model = ss.str();
    std::ofstream myfile;
    myfile.open ("/tmp/model.xml");
    myfile << model;
    myfile.close();
    sdfModel.SetFromString(model);
    this->world_->InsertModelSDF(sdfModel);
}

void GazeboRosFactory::createHuman(const std::string &name, const math::Pose &pose) {
    double radius = 0.2, length_viusal = 1.8, length_collision = 0.4, mass = 10;
    double half_length_visual = length_viusal/2.0;
    double half_length_collision = length_collision/2.0;
    std::string modelStr = GazeboModelTemplates::cylinderTemplate(human_template_file_, name, pose, radius, mass, length_viusal,half_length_collision);
    sdf::SDF sdfModel;
    sdfModel.SetFromString(modelStr);
    this->world_->InsertModelSDF(sdfModel);
}

void GazeboRosFactory::commandCallback(const std_msgs::String& msg) {
    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(mutexHumans_);
    if(msg.data.compare("clear") == 0) {
        /*
        BOOST_FOREACH(int id, humansInWorld_) {
          std::string name = idToName(id);
          ROS_INFO ( "clearing %s", name.c_str());
          physics::ModelPtr p  = this->world_->GetModel(name);
          if(p) {
              ROS_INFO ( "Fini() start", name.c_str());
              sleep(1);
              p->SetLinearVel(math::Vector3::Zero);
              p->SetLinearAccel(math::Vector3::Zero);
              p->Fini();
              ROS_INFO ( "Fini() end", name.c_str());
          } else {
              ROS_INFO ( "clear failed %s", name.c_str());
          }
          sleep(10);
        }
          */
    } else {
        ROS_INFO ( "commandCallback");
        //physics::ModelPtr p  = this->world_->GetModel("c1");
        //p->Fini();
    }
}

physics::ModelPtr GazeboRosFactory::getHuman(int id) {
    std::string name = idToName(id);
    return world_->GetModel(name);
}


void GazeboRosFactory::wallsCallback(const transitbuddy_msgs::LineWithIDArrayConstPtr& msg) {
    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(mutexWall_);
    walls_ = *msg;
}


void GazeboRosFactory::humanCallback(const transitbuddy_msgs::PoseWithIDArrayConstPtr& msg) {
    if(mutexHumans_.try_lock()) {
        msgHumans = *msg;
        mutexHumans_.unlock();
    }

}

void GazeboRosFactory::updateHumansFnc() {
    int seq = msgHumans.header.seq;
    while(true) {
        usleep(100000);
        if(seq != msgHumans.header.seq) {
            seq = msgHumans.header.seq;
            std::vector <int> logAdded;
            std::vector <int> logMoved;
            std::vector <int> logRemoved;
            boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(mutexHumans_);
            math::Pose pose(0,0,0,0,0,0);
            double ca = cos(map_offset_angle_), sa = sin(map_offset_angle_);
            physics::ModelPtr p;
            std::map<int,physics::ModelPtr>::iterator it;
            std::map<int, physics::ModelPtr> lasthumansActive = humansActive_;
            humansActive_.clear();
            for(std::size_t i = 0; i < msgHumans.poses.size(); i++) {
                if (msgHumans.poses[i].valid == false) continue;
                const geometry_msgs::Point &pos = msgHumans.poses[i].pose.position;
                double xw = ca * pos.x - sa * pos.y + map_offset_x_;
                double yw = sa * pos.x + ca * pos.y + map_offset_y_;
                double zw = 0.0;
                xw = pos.x;
                yw = pos.y;
                zw = 0;
                math::Pose poseModel(xw, yw, zw, 0, 0, 0);
                it = lasthumansActive.find(msgHumans.poses[i].id);
                if(it == lasthumansActive.end()) {
                    if(humansInactive_.empty()) {
                        ROS_INFO ( "updateHumansFnc: out of humans");
                        continue;
                    } else {
                        p = humansInactive_.back();
                        humansInactive_.pop_back();
                        p->SetWorldPose(poseModel);
                        p->SetStatic(false);
                    }
                } else {
                    p = it->second;
                    lasthumansActive.erase(it);
                }
                humansActive_[msgHumans.poses[i].id] = p;
                const math::Pose &current =  p->GetWorldPose();
                math::Vector3 diff = poseModel.pos - current.pos;
                p->SetLinearVel(math::Vector3(diff.x, diff.y, -0));
                p->SetAngularVel(math::Vector3::Zero);
                p->SetAngularAccel(math::Vector3::Zero);
             }
            int cols =  ceil(sqrt(max_humans_));
            for (it = lasthumansActive.begin(); it!=lasthumansActive.end(); ++it) {
                p = it->second;
                int id = humansInactive_.size();
                humansInactive_.push_back(p);
                int r = id / cols;
                int c = id % cols;
                math::Pose pose(min_distance_between_humans_*r, min_distance_between_humans_*c, 0, 0, 0, 0);
                p->SetStatic(true);
                p->SetAngularVel(math::Vector3::Zero);
                p->SetAngularAccel(math::Vector3::Zero);
                p->SetLinearVel(math::Vector3::Zero);
                p->SetLinearAccel(math::Vector3::Zero);
                p->SetWorldPose(pose);
            }
            
            ROS_INFO("GazeboRosFactory Plugin (ns = %s) active humans  %i",
                 this->robot_namespace_.c_str(), (int) humansActive_.size());
  
        }
    }
}

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(GazeboRosFactory)
}

