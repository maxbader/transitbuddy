#ifndef GAZEBO_MODEL_TEMPLATE_HH
#define GAZEBO_MODEL_TEMPLATE_HH

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/replace.hpp>

namespace gazebo {

class GazeboModelTemplates {
public:
    static std::string boxTemplate(const std::string &name, const math::Pose &pose, double width, double height, double mass, double length_visual, double length_collision) {
        std::string modelStr(
        "<sdf version ='1.4'>\
          <model name ='${name}'>\
            <pose>${pose}</pose>\
            <link name ='link'>\
              <pose>0 0 0 0 0 0</pose>\
              <collision name ='collision'>\
              <pose>0 0 ${half_length_collision} 0 0 0</pose>\
                <geometry>\
                   <box><size>${width} ${height} ${length_viusal}</size></box>\
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
                   <box><size>${width} ${height} ${length_viusal}</size></box>\
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
        std::stringstream poseStr;
        double ixx = 0.0833333 * mass * (width * width + length_collision * length_collision);
        double iyy = 0.0833333 * mass * (height * height + length_collision * length_collision);
        double izz = 0.0833333 * mass * (height * height + width * width);
        poseStr << pose.pos.x << " " << pose.pos.y << " " << pose.pos.z << " 0 0 0";
        boost::replace_all(modelStr, "${name}", name);
        boost::replace_all(modelStr, "${pose}", poseStr.str());
        boost::replace_all(modelStr, "${width}", boost::lexical_cast<std::string>(width));
        boost::replace_all(modelStr, "${height}", boost::lexical_cast<std::string>(height));
        boost::replace_all(modelStr, "${half_length_visual}", boost::lexical_cast<std::string>(length_visual/2.0));
        boost::replace_all(modelStr, "${half_length_collision}", boost::lexical_cast<std::string>(length_collision/2.0));

        boost::replace_all(modelStr, "${length_viusal}", boost::lexical_cast<std::string>(length_visual));
        boost::replace_all(modelStr, "${length_collision}", boost::lexical_cast<std::string>(length_collision));
        boost::replace_all(modelStr, "${mass}", boost::lexical_cast<std::string>(mass));
        boost::replace_all(modelStr, "${ixx}", boost::lexical_cast<std::string>( ixx));
        boost::replace_all(modelStr, "${iyy}", boost::lexical_cast<std::string>( iyy));
        boost::replace_all(modelStr, "${izz}", boost::lexical_cast<std::string>(izz));
        return modelStr;
    }
    static std::string cylinderTemplate(const std::string &tmplateFile, const std::string &name, const math::Pose &pose, double radius, double mass, double length_visual, double length_collision) {
        std::ifstream ifs(tmplateFile.c_str());
        std::string modelStr( (std::istreambuf_iterator<char>(ifs) ),  (std::istreambuf_iterator<char>()    ) );
        ifs.close();
        std::stringstream poseStr;
        double ixx = 0.0833333 * mass * (3 * radius * radius + length_collision * length_collision);
        double iyy = 0.0833333 * mass * (3 * radius * radius + length_collision * length_collision);
        double izz = 0.5 * mass * radius * radius;
        poseStr << pose.pos.x << " " << pose.pos.y << " " << pose.pos.z << " 0 0 0";
        boost::replace_all(modelStr, "${name}", name);
        boost::replace_all(modelStr, "${pose}", poseStr.str());
        boost::replace_all(modelStr, "${radius}", boost::lexical_cast<std::string>(radius));
        boost::replace_all(modelStr, "${half_length_visual}", boost::lexical_cast<std::string>(length_visual/2.0));
        boost::replace_all(modelStr, "${half_length_collision}", boost::lexical_cast<std::string>(length_collision/2.0));

        boost::replace_all(modelStr, "${length_viusal}", boost::lexical_cast<std::string>(length_visual));
        boost::replace_all(modelStr, "${length_collision}", boost::lexical_cast<std::string>(length_collision));
        boost::replace_all(modelStr, "${mass}", boost::lexical_cast<std::string>(mass));
        boost::replace_all(modelStr, "${ixx}", boost::lexical_cast<std::string>( ixx));
        boost::replace_all(modelStr, "${iyy}", boost::lexical_cast<std::string>( iyy));
        boost::replace_all(modelStr, "${izz}", boost::lexical_cast<std::string>(izz));
        return modelStr;
    }
};

}

#endif

