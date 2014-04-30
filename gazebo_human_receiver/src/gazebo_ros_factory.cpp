
#include <v4r_gazebo/gazebo_ros_factory.h>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

namespace gazebo
{

// Constructor
GazeboRosFactory::GazeboRosFactory() {}

// Destructor
GazeboRosFactory::~GazeboRosFactory() {}


void GazeboRosFactory::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
    this->world_ = _parent;

    rosnode_ = boost::shared_ptr<ros::NodeHandle> ( new ros::NodeHandle () );
    n_param_ = boost::shared_ptr<ros::NodeHandle>(new ros::NodeHandle("~"));

    ROS_INFO ( "GazeboRosFactory");
    //add("c1", math::Pose::Zero);
    subHumanPose_ = rosnode_->subscribe( "/human_publisher/human_pose", 1, &GazeboRosFactory::poseCallback, this );
    subCommand_ = rosnode_->subscribe( "/transitbuddy_dummy/command", 1, &GazeboRosFactory::commandCallback, this );

    n_param_->param<double> ( "offsetX", offsetX_, 133.0 );
    n_param_->param<double> ( "offsetY", offsetY_, 109.0 );
    n_param_->param<double> ( "offsetAlpha", offsetAlpha_, 0.0 );
    ROS_INFO ( "offset: %6.3f m, %6.3f m, %6.3f rad", offsetX_, offsetY_, offsetAlpha_ );
    //n_param_->setParam( "offsetX", offsetX_ );
    //n_param_->setParam( "offsetY", offsetY_ );
    //n_param_->setParam( "offsetAlpha", offsetAlpha_ );

    addThread_ = boost::shared_ptr<boost::thread>(new boost::thread(&GazeboRosFactory::addTheadFnc, this));
    removeThead_ = boost::shared_ptr<boost::thread>(new boost::thread(&GazeboRosFactory::removeTheadFnc, this));
    updateThead_ = boost::shared_ptr<boost::thread>(new boost::thread(&GazeboRosFactory::updateHumansFnc, this));

    double offsetX, offsetY, offsetAlpa;
}


void GazeboRosFactory::add(int id, const math::Pose &pose) {
    double radius = 0.3, length = 1.8, mass = 10;
    std::string modelStr(
        "<sdf version ='1.4'>\
          <model name ='${name}'>\
            <pose>${pose}</pose>\
            <link name ='link'>\
              <pose>0 0 0 0 0 0</pose>\
              <collision name ='collision'>\
                <geometry>\
                  <cylinder>\
                    <radius>${radius}</radius>\
                    <length>${length}</length>\
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
                    <length>${length}</length>\
                  </cylinder>\
                </geometry>\
              </visual>\
              <inertial>\
                <mass>${mass}</mass>\
                <pose>0 0 0 0 0 0</pose>\
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

    std::stringstream poseStr;
    double ixx = 0.0833333 * mass * (3 * radius * radius + length * length);
    double iyy = 0.0833333 * mass * (3 * radius * radius + length * length);
    double izz = 0.5 * mass * radius * radius;
    poseStr << pose.pos.x << " " << pose.pos.y << " " << pose.pos.z << " 0 0 0";
    boost::replace_all(modelStr, "${name}", idToName(id));
    boost::replace_all(modelStr, "${pose}", poseStr.str());
    boost::replace_all(modelStr, "${radius}", boost::lexical_cast<std::string>(radius));
    boost::replace_all(modelStr, "${length}", boost::lexical_cast<std::string>(length));
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

        n_param_->getParam( "offsetX", offsetX_);
        n_param_->getParam("offsetY", offsetY_);
        n_param_->getParam("offsetAlpha", offsetAlpha_);
        ROS_INFO ( "offset: %6.3fm, %6.3fm, %6.3frad", offsetX_, offsetY_, offsetAlpha_ );


        sleep(2);
        boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(mutex_);
        std::stringstream ss;
        for (std::list<int>::iterator it=humansToRemove_.begin(); it!=humansToRemove_.end(); ++it) {
            int id = *it;
            ss << id << ", ";
            physics::ModelPtr p  = getHuman(id);
            if(p) {
                ROS_INFO ( "remove %02i start", id);
                double x = humansRemoved_.size()%20 + 100;
                double y = -humansRemoved_.size()/20;
                p->SetWorldPose(math::Pose(x, y, 0, 0,0,0));
                p->SetLinearVel(math::Vector3::Zero);
                p->SetLinearAccel(math::Vector3::Zero);
                humansInWorld_.remove(id);
                humansRemoved_.push_back(id);
                ROS_INFO ( "remove %02i done", id);
            } else {
                ROS_INFO ( "remove %02i failed", id);
            }
        }
        std::string listOfHumansToRemive = ss.str();
        ROS_INFO ( "remove %s", listOfHumansToRemive.c_str());

    }
}



void GazeboRosFactory::poseCallback(const transitbuddy_msgs::PoseWithIDArrayConstPtr& msg) {
    boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(mutex_);
    msgHumans = *msg;
}

void GazeboRosFactory::updateHumansFnc() {
    while(true) {
        usleep(10000);
        boost::interprocess::scoped_lock<boost::interprocess::interprocess_mutex> lock(mutex_);
        math::Pose pose(0,0,0,0,0,0);
        double ca = cos(offsetAlpha_), sa = sin(offsetAlpha_);
        std::list<int> unused = humansInWorld_;
        for(std::size_t i = 0; i < msgHumans.poses.size(); i++) {
            int id = msgHumans.poses[i].id;
            unused.remove(id);
            const geometry_msgs::Point &pos = msgHumans.poses[i].pose.position;
            double xw = ca * pos.x - sa * pos.y + offsetX_;
            double yw = sa * pos.x + ca * pos.y + offsetY_;
            double zw = 0.0;
            math::Pose poseModel(xw, yw, zw, 0, 0,0);
            physics::ModelPtr p  = getHuman(id);
            if(!p) {
                humansToAdd_[id] = poseModel;
                //p = this->world_->GetModel(modelName);
            } else {
                const math::Pose &current =  p->GetWorldPose();
                if(fabs(current.rot.x) + fabs(current.rot.y) + fabs(current.rot.z) < 0.01) {
                    math::Vector3 diff = poseModel.pos - current.pos;
                    p->SetLinearVel(math::Vector3(diff.x, diff.y, -0));
                } else {
                    p->SetWorldPose(math::Pose(poseModel.pos.x, poseModel.pos.y, 0, 0,0,0));
                    p->SetLinearVel(math::Vector3::Zero);
                    p->SetLinearAccel(math::Vector3::Zero);
                }
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
