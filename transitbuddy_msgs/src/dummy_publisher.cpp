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
#include <random_numbers/random_numbers.h>
#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <transitbuddy_msgs/PoseWithIDArray.h>

typedef boost::numeric::ublas::vector<double> uvec;
typedef boost::numeric::ublas::matrix<double> umat;

class DummyPublisher {
public:
    DummyPublisher(): n_(), n_param_("~"), frequency_(20.0), people_(10) {

        n_param_.getParam ( "frequency", frequency_ );
        ROS_INFO ( "frequency: %5.2f", frequency_ );
        
        n_param_.getParam ( "people", people_ );
        ROS_INFO ( "people: %5i", people_ );

        poses_.header.seq = 0;
        poses_.header.stamp = ros::Time::now();
        poses_.header.frame_id = "map";
        poses_.poses.resize(people_);

        velocities_ = umat(people_,2);
        for(std::size_t i = 0; i < poses_.poses.size(); i++) {
            poses_.poses[i].id = i;
            poses_.poses[i].pose.position.x = i;
            poses_.poses[i].pose.position.y = 0;
            poses_.poses[i].pose.position.z = 0;
            velocities_(i,0) = rand_numbers.uniformReal(-1.2, 1.2);
            velocities_(i,1) = rand_numbers.uniformReal(-1.2, 1.2);
        }

        pub_ = n_param_.advertise<transitbuddy_msgs::PoseWithIDArray> ( "human_pose", 1 );
    }
    ~DummyPublisher() {
    }
    void run() {
        rate_ = boost::shared_ptr<ros::Rate>(new ros::Rate(frequency_));
        while ( ros::ok() ) {
            publish();
            ros::spinOnce();
            rate_->sleep();
        }
    }
private:
    ros::NodeHandle n_;
    ros::NodeHandle n_param_;
    boost::shared_ptr<ros::Rate> rate_;
    double frequency_;
    int people_;
    ros::Publisher pub_;
    transitbuddy_msgs::PoseWithIDArray poses_;
    umat velocities_;
    random_numbers::RandomNumberGenerator rand_numbers; 
    


    void publish() {
        pub_.publish(poses_);
        umat d = velocities_ / frequency_;
        for(std::size_t i = 0; i < poses_.poses.size(); i++) {
          poses_.poses[i].pose.position.x = poses_.poses[i].pose.position.x + d(i,0);
          poses_.poses[i].pose.position.y = poses_.poses[i].pose.position.y + d(i,1);
        }
    }


};

int main(int argc, char **argv)
{
    ros::init ( argc, argv, "transitbuddy_dummy" );
    DummyPublisher dummy;
    dummy.run();
    return 0;
}
