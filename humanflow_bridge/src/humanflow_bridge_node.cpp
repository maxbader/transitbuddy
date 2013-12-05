/*
 * Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
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

#include <sstream>
#include <iostream>
#include <fstream>
#include <hash_map>
#include <algorithm>

#include "ros/ros.h"
#include <humanflow_bridge/humanflow_bridge.h>

#include "mpedplus/MpedMessage.h"
#include "mpedplus/MpedClientAPI.h"

int main(int argc, char **argv)
{
	cout << " starting module! " << endl;

	// Init MpedPlus
	mped::MpedClientAPI mpedController;
	// no params means debug mode
	if(argc == 1) {
		string modDescrString = "";
		ifstream ifs("module.xml");
		string temp;
		while (getline(ifs, temp))
			modDescrString += temp;

		// init API controller
		cout << " run in debug mode " << endl;
		mpedController.debugRun(MPED_URL, modDescrString, MPED_INSTANCE_ID, "");
	} else if (argc >= 3) { // only instance uid (& framework url) is given
		cout << argv[1] << " " << argv[2] << endl;

		// run
		mpedController.run(argv[1], argv[2], "");
	} else {
		cout << "Invalid number or arguments: Must be 0 or 2" << endl;
		exit(EXIT_FAILURE);
	}
	
	// Without a ROS_MASTER, we'll segfault! We probably should just do that
	// if the user hasn't set a ROS_MASTER_URI himself
	//_putenv("ROS_MASTER_URI=http://192.168.56.101:11311/");
	
	// Init ROS
	cout << "Init ROS. Make sure you have an environment variable like ROS_MASTER_URI=http://192.168.56.101:11311/ or the program will segfault!" << endl;
	char* masterUri = getenv("ROS_MASTER_URI");
	if (masterUri == NULL)
		cerr << "No ROS_MASTER_URI found!!" << endl;
	else
		cerr << "ROS_MASTER_URI: " << masterUri << endl;

    ros::init ( argc, argv, "humanflow_bridge" );
	cout << "Creating Handle" << endl;
	ros::NodeHandle n;
	cout << "Creating Bridge" << endl;
    HumanflowBridgeNode bridge ( n, mpedController );
    ros::Rate rate ( bridge.frequency() );
	cout << "Starting main loop" << endl;

	// Run the main loop
    while ( ros::ok() ) {

		// Handle messages from the mped framework
		mped::MpedMessage *msg = mpedController.waitForNextMessage();
		if (msg == NULL)
			break;

		bridge.handleMpedMessage(*msg);
		mpedController.sendDoneMessage();
		delete msg;

        ros::spinOnce();
    }
    return 0;
}

HumanflowBridgeNode::HumanflowBridgeNode(ros::NodeHandle & n, mped::MpedClientAPI &controller)
		: n_ ( n ),
		n_param_ ( "~" ),
		frequency_ ( DEFAULT_FRQ),
		publish_(true),
		frame_id_(DEFAULT_FRAME_ID),
		mpedController(mpedController) {
	
    n_param_.getParam ( "frequency", frequency_ );
    ROS_INFO ( "frequency: %5.2f", frequency_ );
    
    n_param_.getParam ( "publish", publish_ );
    ROS_INFO ( "publish: %s", (publish_?"true":"false") );
    
    n_param_.getParam ( "frame_id", frame_id_ );
    ROS_INFO ( "frame_id: %s", frame_id_.c_str() );
    
	sub_ = n_.subscribe( NAME_SUB, 1, &HumanflowBridgeNode::robotPoseCallback, this );
	pub_ = n_param_.advertise<humanflow_bridge::PoseWithIDArray> ( "human_pose", 1 );
}

HumanflowBridgeNode::~HumanflowBridgeNode(){
	}

void HumanflowBridgeNode::robotPoseCallback(const humanflow_bridge::PoseWithIDArray::ConstPtr& msg){
    ROS_INFO ( "robotPoseCallback");
	
	for (auto iter = msg->poses.begin(); iter != msg->poses.end(); iter++)
	{
		// TODO: Ignore agents from other section
		long id = iter->id;
		if (std::find(robotIds.begin(), robotIds.end(), id) == robotIds.end()) {
			// No robot added yet, do it now
			ROS_INFO("adding robot: %d", id);
			mped::MpedMessage *msg = mpedController.createMessage("ADD_AGENTS");
			vector<long> newIds;
			newIds.push_back(id);
			msg->setLongArray("ids", newIds);
			stringstream section;
			section << "s" << id;
			msg->setLong(section.str(), sectionId);
			stringstream type;
			type << "t" << id;
			msg->setString(type.str(), "robot");
			mpedController.sendMessage(msg);
			delete msg;

			// save the id
			robotIds.push_back(id);
		}

		// Update position
		mped::MpedMessage *msg = mpedController.createMessage("UPDATE_AGENTS");
		vector<long> newIds;
		newIds.push_back(id);
		msg->setLongArray("ids", newIds);
		//stringstream section;
		//section << "s" << id;
		//msg->setLong(section.str(), sectionId);
		//stringstream type;
		//type << "t" << id;
		//msg->setString(type.str(), "robot");
		stringstream posKey;
		posKey << "p" << id;
		vector<double> pos;
		pos.push_back(iter->pose.position.x);
		pos.push_back(iter->pose.position.y);
		pos.push_back(iter->pose.position.z);
		msg->setDoubleArray(posKey.str(), pos);
		mpedController.sendMessage(msg);
		delete msg;
	}

	for (auto iter = robotIds.begin(); iter != robotIds.end(); iter++) {
		bool hasPose = false;
		for (auto posesIter = msg->poses.begin(); posesIter != msg->poses.end(); posesIter++) {
			if (posesIter->id == *iter) {
				hasPose = true;
				break;
			}
		}

		if (hasPose == false) {
			// Robot does not exist anymore, remove it from the simulation
			ROS_INFO("removing robot: %d", *iter);
			mped::MpedMessage *msg = mpedController.createMessage("REMOVE_AGENTS");
			vector<long> newIds;
			newIds.push_back(*iter);
			msg->setLongArray("ids", newIds);
			mpedController.sendMessage(msg);
			delete msg;

			// TODO: Remove value from robotIds so it can be used again
		}
	}
}

void HumanflowBridgeNode::handleMpedMessage(mped::MpedMessage &msg){
	if (msg.isInitMessage())
	{
		//onInit(msg);
		sectionId = mpedController.getAssociatedSection();
		if (sectionId == 0)
			mpedController.warn("Module is not associated with a section! It should be moved to the section where the robot is created");
	}
	else if (msg.isNextStepMessage())
	{
		//onNextStep(msg->getLong("step"), msg->getDouble("stepDuration"), msg->getDouble("simulationDuration"));
		long step = msg.getLong("step");
		// and send the info to the ROS server
		if (step % 10 == 0)
			publishHumanPose();
	}
	else if (msg.isShutdownMessage())
	{
		cout << "Received Shutdown Message" << endl;
	}
	else if (msg.isKillMessage())
	{
		cout << "Received Kill Message" << endl;
		exit(EXIT_SUCCESS);
	}
	/*else if (msg.equals("ADD_AGENTS"))
	{
		vector<long> ids = msg->getLongArray("ids");
		for (vector<long>::iterator iter = ids.begin(); iter != ids.end(); iter++)
		{
			// TODO: Ignore agents from other section
			Agent *agent = new Agent(*iter);
			stringstream ss; ss << (*iter);
			std::vector<double> pos = msg.getDoubleArray("p" + ss.str());
			if (pos.size() >= 2)
				agent->position = new Point(pos[0], pos[1]);

			agents[*iter] = agent;
		}

		// set wachter value -> number of processed pedestrians
		char watcherValueChar[10];
		sprintf(watcherValueChar, "%d", agents.size());
		string watcherValueStr = watcherValueChar;
		controller.setWatcherValue("Num of c++ agents", watcherValueStr);
	}*/
	else if (msg.equals("UPDATE_AGENTS"))
	{
		vector<long> ids = msg.getLongArray("ids");
		poses.poses.resize(ids.size());
		int i = 0;
		for (vector<long>::iterator iter = ids.begin(); iter != ids.end(); iter++)
		{
			poses.poses[i].id = *iter;
			stringstream ss; ss << (*iter); 
			std::vector<double> pos = msg.getDoubleArray("p" + ss.str());
			if (pos.size() >= 2)
			{
				poses.poses[i].valid = true;
				poses.poses[i].pose.position.x = pos[0];
				poses.poses[i].pose.position.y = pos[1];
			} else {
				poses.poses[i].valid = false;
			}
			i++;
			/*AgentIterator agentIter = agents.find(*iter);
			if (agentIter == agents.end())
				continue; // agent not in our list

			stringstream ss; ss << (*iter); 
			std::vector<double> pos = msg.getDoubleArray("p" + ss.str());
			if (pos.size() >= 2)
			{
				if (agentIter->second->position == NULL)
					agentIter->second->position = new Point();

				agentIter->second->position->x = pos[0];
				agentIter->second->position->y = pos[1];
			}*/
		}

		// cout << "Ignoring update agents for now" << endl;
	}
	/*else if (msg.equals("REMOVE_AGENTS"))
	{
		vector<long> ids = msg.getLongArray("ids");
		cout << "Ignoring remove agents for now" << endl;
	}*/
		
	
}


void HumanflowBridgeNode::publishHumanPose() {
	if(publish_ == false) return;
	ROS_DEBUG("publishHumanPose: %d humans", poses.poses.size());
    poses.header.stamp = ros::Time::now();
    poses.header.frame_id = frame_id_;
	pub_.publish(poses);
}
