#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

namespace gazebo
{
class FactoryAndFini : public WorldPlugin
{
    physics::WorldPtr world_;
    boost::shared_ptr<boost::thread> removeThead_;
    boost::shared_ptr<boost::thread> moveThead_;
public:
    void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
        this->world_ = _parent;
        add("object_static", math::Pose(0,0,0,0,0,0), /*radius*/ 0.3, /*height*/ 2.0, /*mass*/ 10, /*mu*/ 0.0, /*mu2*/ 0.0 );
        std::cout << "added object_static" << std::endl;
        add("object_moving_no_mu", math::Pose(1,0,0,0,0,0), /*radius*/ 0.3, /*height*/ 2.0, /*mass*/ 10, /*mu*/ 0.0, /*mu2*/ 0.0 );
        std::cout << "added object_moving_no_mu" << std::endl;
        add("object_moving_with_mu", math::Pose(2,0,0,0,0,0), /*radius*/ 0.3, /*height*/ 2.0, /*mass*/ 10, /*mu*/ 0.1, /*mu2*/ 0.0 );
        std::cout << "added object_moving_with_mu" << std::endl;
        
        
        removeThead_ = boost::shared_ptr<boost::thread>(new boost::thread(&FactoryAndFini::removeTheadFnc, this));
        moveThead_ = boost::shared_ptr<boost::thread>(new boost::thread(&FactoryAndFini::moveTheadFnc, this));
    };

    void moveTheadFnc() {
        sleep(1);
        physics::ModelPtr p;
        p = world_->GetModel("object_moving_no_mu");
        if(p) {
          std::cout << "moving object_moving_no_mu" << std::endl;
          p->SetLinearVel(math::Vector3(0.1, 0, 0));
        }   
        p = world_->GetModel("object_moving_with_mu");
        if(p) {
          std::cout << "moving object_moving_with_mu" << std::endl;
          p->SetLinearVel(math::Vector3(0.1, 0, 0));
        }      
    }
    void removeTheadFnc() {

        sleep(5);
        physics::ModelPtr p;
        std::cout << "removing object_static" << std::endl;
        p =  world_->GetModel("object_static");
        if(p){
          p->Fini();
          std::cout << "done removing object_static" << std::endl;
        }
        sleep(1);
        std::cout << "removing object_moving_no_mu" << std::endl;
        p =  world_->GetModel("object_moving_no_mu");
        if(p){
          p->Fini();
          std::cout << "done removing object_moving_no_mu" << std::endl;
        }
        sleep(1);
        std::cout << "removing object_moving_with_mu" << std::endl;
        p =  world_->GetModel("object_moving_with_mu");
        if(p){
          p->Fini();
          std::cout << "done removing object_moving_with_mu" << std::endl;
        }
    }

    void add(std::string name, const math::Pose &pose, double radius, double length, double mass, double mu, double mu2) {
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
                      <mu>${mu}</mu>\
                      <mu2>${mu2}</mu2>\
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
        boost::replace_all(modelStr, "${name}", name);
        boost::replace_all(modelStr, "${pose}", poseStr.str());
        boost::replace_all(modelStr, "${radius}", boost::lexical_cast<std::string>(radius));
        boost::replace_all(modelStr, "${length}", boost::lexical_cast<std::string>(length));
        boost::replace_all(modelStr, "${mass}", boost::lexical_cast<std::string>(mass));
        boost::replace_all(modelStr, "${ixx}", boost::lexical_cast<std::string>( ixx));
        boost::replace_all(modelStr, "${iyy}", boost::lexical_cast<std::string>( iyy));
        boost::replace_all(modelStr, "${izz}", boost::lexical_cast<std::string>(izz));
        boost::replace_all(modelStr, "${mu}", boost::lexical_cast<std::string>( mu));
        boost::replace_all(modelStr, "${mu2}", boost::lexical_cast<std::string>(mu2));
        sdf::SDF sdfModel;
        sdfModel.SetFromString(modelStr);
        this->world_->InsertModelSDF(sdfModel);
    }
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(FactoryAndFini)
}
