
#include <gazebo_human_receiver/gazebo_ros_factory.h>
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

    ROS_INFO ( "GazeboRosFactory");
    //add("c1", math::Pose::Zero);
    subHumanPose_ = rosnode_->subscribe( "/human_publisher/human_pose", 1, &GazeboRosFactory::humanCallback, this );
    subCommand_ = rosnode_->subscribe( "/transitbuddy_dummy/command", 1, &GazeboRosFactory::commandCallback, this );
    // subWall_ = rosnode_->subscribe( "/human_publisher/walls", 1, &GazeboRosFactory::wallsCallback, this );

    addThread_ = boost::shared_ptr<boost::thread>(new boost::thread(&GazeboRosFactory::addTheadFnc, this));
    removeThead_ = boost::shared_ptr<boost::thread>(new boost::thread(&GazeboRosFactory::removeTheadFnc, this));
    updateThead_ = boost::shared_ptr<boost::thread>(new boost::thread(&GazeboRosFactory::updateHumansFnc, this));

    reconfigureFnc_ = boost::bind(&GazeboRosFactory::callbackParameters, this,  _1, _2);
    reconfigureServer_.setCallback(reconfigureFnc_);

    //wallThead_ = boost::shared_ptr<boost::thread>(new boost::thread(&GazeboRosFactory::wallTheadFnc, this));
    // addWall(10, 200, 0, 200, 200);
/*
    walls_.lines.resize(4);
    walls_.lines[0].id = 0;
    walls_.lines[0].p.resize(2);
    walls_.lines[0].p[0].x = -10;
    walls_.lines[0].p[0].y = -10;
    walls_.lines[0].p[1].x =  10;
    walls_.lines[0].p[1].y = -10;
    walls_.lines[1].id = 1;
    walls_.lines[1].p.resize(2);
    walls_.lines[1].p[0].x =  10;
    walls_.lines[1].p[0].y = -10;
    walls_.lines[1].p[1].x =  10;
    walls_.lines[1].p[1].y =  10;
    
    walls_.lines[2].id = 2;
    walls_.lines[2].p.resize(2);
    walls_.lines[2].p[0].x =  10;
    walls_.lines[2].p[0].y =  10;
    walls_.lines[2].p[1].x = -10;
    walls_.lines[2].p[1].y =  10;
    walls_.lines[3].id = 3;
    walls_.lines[3].p.resize(2);
    walls_.lines[3].p[0].x = -10;
    walls_.lines[3].p[0].y =  10;
    walls_.lines[3].p[1].x = -10;
    walls_.lines[3].p[1].y = -10;
    */
    
}

void GazeboRosFactory::wallTheadFnc() {
    double loop = true;
    while(loop) {
        sleep(2);
        if(walls_.lines.size() > 0)
        {
            loop = false;
            boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(mutexWall_);
            addWallModel();
        }
    }
}

void GazeboRosFactory::addWallModel() {
    std::stringstream ss;
    ss << "<sdf version ='1.4'>" << std::endl;
    ss << "  <link name='world'/>" << std::endl;
    ss << "  <model name ='walls'>" << std::endl;
    for(unsigned int i = 0; i < walls_.lines.size(); i++) {
        double dx = walls_.lines[i].p[0].x - walls_.lines[i].p[1].x;
        double dy = walls_.lines[i].p[0].y - walls_.lines[i].p[1].y;
        double l = sqrt(dx*dx + dy*dy);
        double a = atan2(dy, dx);
        double w = 0.2;
        double h = 1.5;
        double h2 = h*2;
        double x = walls_.lines[i].p[0].x + dx/2.;
        double y = walls_.lines[i].p[0].y + dy/2.;
        std::string modelStr(
          
            "<joint name='${joint}' type='fixed'>\
              <parent link='world'/>\
              <child link='${link}'/>\
            </joint>\
            <link name ='${link}'>\
            <pose>{x} ${y} ${h} 0 0 0</pose>\
            <collision name ='collision'>\
              <geometry>\
                <box><size>${l} ${w} ${h2}</size></box>\
              </geometry>\
            </collision>\
            <visual name ='visual'>\
              <geometry>\
                <box><size>${l} ${w} ${h2}</size></box>\
              </geometry>\
            </visual>\
          </link>");
        boost::replace_all(modelStr, "${joint}", std::string("j") + boost::lexical_cast<std::string>(walls_.lines[i].id));
        boost::replace_all(modelStr, "${link}",  std::string("l") + boost::lexical_cast<std::string>(walls_.lines[i].id));
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
    ss << "  </model>" << std::endl;
    ss << "</sdf>" << std::endl;
    sdf::SDF sdfModel;
    std::string model = ss.str();
    sdfModel.SetFromString(model);
    this->world_->InsertModelSDF(sdfModel);
}

void GazeboRosFactory::addWall(int id, double x0, double y0, double x1, double y1) {
    std::vector<int>::iterator it = std::find (wallsInWorld_.begin(), wallsInWorld_.end(), 30);
    if(it != wallsInWorld_.end()) return;
    double dx = x1 - x0;
    double dy = y1 - y0;
    double l = sqrt(dx*dx + dy*dy);
    double a = atan2(dy, dx);
    double w = 0.2;
    double h = 1.5;
    double h2 = h*2;
    double x = x0 + dx/2.;
    double y = y0 + dy/2.;
    std::string modelStr(
        "<sdf version ='1.4'>\
          <model name ='${name}'>\
          <link name ='${link}'>\
            <pose>0 0 ${h} 0 0 0</pose>\
            <collision name ='collision'>\
              <geometry>\
                <box><size>${l} ${w} ${h2}</size></box>\
              </geometry>\
            </collision>\
            <visual name ='visual'>\
              <geometry>\
                <box><size>${l} ${w} ${h2}</size></box>\
              </geometry>\
            </visual>\
          </link>\
        </model>\
        </sdf>");

    boost::replace_all(modelStr, "${name}", std::string("w") + boost::lexical_cast<std::string>(id));
    boost::replace_all(modelStr, "${joint}", std::string("j") + boost::lexical_cast<std::string>(id));
    boost::replace_all(modelStr, "${link}", std::string("l") + boost::lexical_cast<std::string>(id));
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
    wallsInWorld_.push_back(id);
}

void GazeboRosFactory::add(int id, const math::Pose &pose) {
    double radius = 0.2, length_viusal = 1.8, length_collision = 0.4, mass = 10;
    double half_length_visual = length_viusal/2.0;
    double half_length_collision = length_collision/2.0;
    std::string modelStr(
        "<sdf version ='1.4'>\
          <model name ='${name}'>\
            <pose>${pose}</pose>\
            <link name ='link'>\
              <pose>0 0 0 0 0 0</pose>\
              <collision name ='collision'>\
              <pose>0 0 ${half_length_collision} 0 0 0</pose>\
                <geometry>\
                   <box><size>${radius} ${radius} ${length_viusal}</size></box>\
                </geometry>\
                <surface>\
                  <friction>\
                    <ode>\
                      <mu>0.0</mu>\
                      <mu2>0.0</mu2>\
                    </ode>\
                  </friction>\
                </surface>\
              </collision>\
              <visual name='visual'>\
              <pose>0 0 ${half_length_visual} 0 0 0</pose>\
                <geometry>\
                   <box><size>${radius} ${radius} ${length_viusal}</size></box>\
                </geometry>\
              </visual>\
              <inertial>\
                <mass>${mass}</mass>\
                <pose>0 0 -0.7 0 0 0</pose>\
                <inertia>\
                  <ixx>${ixx}</ixx>\
                  <ixy>0.0</ixy>\
                  <ixz>0.0</ixz>\
                  <iyy>${iyy}</iyy>\
                  <iyz>0.0</iyz>\
                  <izz>${izz}</izz>\
                </inertia>\
              </inertial>\
            </link>\
          </model>\
        </sdf>");
    /*
 std::string modelStr(
        "<sdf version ='1.4'>\
          <model name ='${name}'>\
            <pose>${pose}</pose>\
            <link name ='link'>\
              <pose>0 0 0 0 0 0</pose>\
              <collision name ='collision'>\
              <pose>0 0 -0.7 0 0 0</pose>\
                <geometry>\
                  <cylinder>\
                    <radius>${radius}</radius>\
                    <length>${length_collision}</length>\
                  </cylinder>\
                </geometry>\
                <surface>\
                  <friction>\
                    <ode>\
                      <mu>0.0</mu>\
                      <mu2>0.0</mu2>\
                    </ode>\
                  </friction>\
                </surface>\
              </collision>\
              <visual name='visual'>\
                <geometry>\
                  <cylinder>\
                    <radius>${radius}</radius>\
                    <length>${length_viusal}</length>\
                  </cylinder>\
                </geometry>\
              </visual>\
              <inertial>\
                <mass>${mass}</mass>\
                <pose>0 0 -0.7 0 0 0</pose>\
                <inertia>\
                  <ixx>${ixx}</ixx>\
                  <ixy>0.0</ixy>\
                  <ixz>0.0</ixz>\
                  <iyy>${iyy}</iyy>\
                  <iyz>0.0</iyz>\
                  <izz>${izz}</izz>\
                </inertia>\
              </inertial>\
            </link>\
          </model>\
        </sdf>");
 */
    std::stringstream poseStr;
    double ixx = 0.0833333 * mass * (3 * radius * radius + length_collision * length_collision);
    double iyy = 0.0833333 * mass * (3 * radius * radius + length_collision * length_collision);
    double izz = 0.5 * mass * radius * radius;
    poseStr << pose.pos.x << " " << pose.pos.y << " " << pose.pos.z << " 0 0 0";
    boost::replace_all(modelStr, "${name}", idToName(id));
    boost::replace_all(modelStr, "${pose}", poseStr.str());
    boost::replace_all(modelStr, "${radius}", boost::lexical_cast<std::string>(radius));
    boost::replace_all(modelStr, "${half_length_visual}", boost::lexical_cast<std::string>(half_length_visual));
    boost::replace_all(modelStr, "${half_length_collision}", boost::lexical_cast<std::string>(half_length_collision));
    
    boost::replace_all(modelStr, "${length_viusal}", boost::lexical_cast<std::string>(length_viusal));
    boost::replace_all(modelStr, "${length_collision}", boost::lexical_cast<std::string>(length_collision));
    boost::replace_all(modelStr, "${mass}", boost::lexical_cast<std::string>(mass));
    boost::replace_all(modelStr, "${ixx}", boost::lexical_cast<std::string>( ixx));
    boost::replace_all(modelStr, "${iyy}", boost::lexical_cast<std::string>( iyy));
    boost::replace_all(modelStr, "${izz}", boost::lexical_cast<std::string>(izz));
    sdf::SDF sdfModel;
    sdfModel.SetFromString(modelStr);
    this->world_->InsertModelSDF(sdfModel);
    humansInWorld_.push_back(id);
}

void GazeboRosFactory::commandCallback(const std_msgs::String& msg) {
    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(mutex_);
    if(msg.data.compare("clear") == 0) {
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
        humansInWorld_.clear();
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

void GazeboRosFactory::addTheadFnc() {
    while(true) {
        sleep(2);
        boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(mutex_);
        std::stringstream ss;
        for (std::map<int, math::Pose>::iterator it=humansToAdd_.begin(); it!=humansToAdd_.end(); ++it) {
            int id = it->first;
            math::Pose &pose = it->second;
            physics::ModelPtr p  = getHuman(id);
            if(!p) {
                add(id, pose);
                ss << id << ", ";
            }
        }
        std::string listOfHumansAdded = ss.str();
         ROS_INFO ( "added %s", listOfHumansAdded.c_str());
        humansToAdd_.clear();

    }
}
void GazeboRosFactory::removeTheadFnc() {
    while(true) {

        sleep(2);
        boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(mutex_);
        std::stringstream ss;
        for (std::list<int>::iterator it=humansToRemove_.begin(); it!=humansToRemove_.end(); ++it) {
            int id = *it;
            ss << id << ", ";
            physics::ModelPtr p  = getHuman(id);
            if(p) {
                ROS_INFO ( "remove %02i start", id);
                double x = humansRemoved_.size()%20 + 300;
                double y = -((double) humansRemoved_.size()/(double) 20.0) + 300;
                p->SetLinearVel(math::Vector3::Zero);
                p->SetLinearAccel(math::Vector3::Zero);
                p->SetWorldPose(math::Pose(x, y, 0, 0,0,0));
                humansInWorld_.remove(id);
                humansRemoved_.push_back(id);
                ROS_INFO ( "remove %02i done -> %4.2f,  %4.2f", id, x, y);
            } else {
                ROS_INFO ( "remove %02i failed", id);
            }
        }
        std::string listOfHumansToRemive = ss.str();
        ROS_INFO ( "remove %s", listOfHumansToRemive.c_str());

    }
}


void GazeboRosFactory::wallsCallback(const transitbuddy_msgs::LineWithIDArrayConstPtr& msg) {
    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(mutexWall_);
    walls_ = *msg;
}


void GazeboRosFactory::humanCallback(const transitbuddy_msgs::PoseWithIDArrayConstPtr& msg) {
    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(mutex_);
    msgHumans = *msg;
}

void GazeboRosFactory::updateHumansFnc() {
    while(true) {
        usleep(10000);
        boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(mutex_);
        math::Pose pose(0,0,0,0,0,0);
        double ca = cos(map_offset_angle_), sa = sin(map_offset_angle_);
        std::list<int> unused = humansInWorld_;
        for(std::size_t i = 0; i < msgHumans.poses.size(); i++) {
            int id = msgHumans.poses[i].id;
            unused.remove(id);
            const geometry_msgs::Point &pos = msgHumans.poses[i].pose.position;
            double xw = ca * pos.x - sa * pos.y + map_offset_x_;
            double yw = sa * pos.x + ca * pos.y + map_offset_y_;
            double zw = 0.0;
            math::Pose poseModel(xw, yw, zw, 0, 0,0);
            physics::ModelPtr p  = getHuman(id);
            if(!p) {
                humansToAdd_[id] = poseModel;
                //p = this->world_->GetModel(modelName);
            } else {
                const math::Pose &current =  p->GetWorldPose();
                math::Vector3 diff = poseModel.pos - current.pos;
                /*
                if(fabs(current.rot.x) + fabs(current.rot.y) + fabs(current.rot.z) < 0.01) {
                    p->SetLinearVel(math::Vector3(diff.x, diff.y, -0));
                } else {
                    p->SetWorldPose(math::Pose(poseModel.pos.x, poseModel.pos.y, 0, 0,0,0));
                    p->SetLinearVel(math::Vector3(diff.x, diff.y, -0));
                }
                */
                p->SetWorldPose(math::Pose(poseModel.pos.x, poseModel.pos.y, 0, 0,0,0));
            }
            //p->SetWorldPose(poseModel);
        }
        humansToRemove_ = unused;
        //p->SetLinearVel(math::Vector3(.03, 0, 0));
    }
}

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(GazeboRosFactory)
}
