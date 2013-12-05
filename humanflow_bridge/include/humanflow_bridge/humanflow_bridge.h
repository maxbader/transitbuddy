/*
 * Copyright (C) 2013.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
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

#ifndef HUMANFLOW_BRIDGE_H
#define HUMANFLOW_BRIDGE_H

#include "ros/ros.h"
#include <humanflow_bridge/humanflow_bridge_defaults.h>
#include <humanflow_bridge/PoseWithIDArray.h>


class HumanflowBridgeNode {
	
	public:
		HumanflowBridgeNode(ros::NodeHandle & n);
		~HumanflowBridgeNode();
		void robotPoseCallback(const humanflow_bridge::PoseWithIDArrayConstPtr& msg);
		double frequency() {
			return frequency_;
		}
  		void publishHumanPose();
	private:
		ros::NodeHandle n_;
		ros::NodeHandle n_param_;
		double frequency_;
		bool publish_;
		std::string frame_id_;
  		ros::Subscriber sub_;
		ros::Publisher pub_;
  		

};

#endif // HUMANFLOW_BRIDGE_H
