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
#include "mpedplus/Section.h"
#include "mpedplus/Region.h"
#include "mpedplus/Shape.h"

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
		std::string url(argv[1]);
		std::string instanceName(argv[2]);
		cout << url << " " << instanceName << endl;
		// run
		mpedController.run(url, instanceName, "");
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

    ros::init ( argc, argv, "human_publisher" );
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
		//rate.sleep();
    }
    return 0;
}

HumanflowBridgeNode::HumanflowBridgeNode(ros::NodeHandle & n, mped::MpedClientAPI &controller)
		: n_ ( n ),
		n_param_ ( "~" ),
		frequency_ ( DEFAULT_FRQ),
		publish_(true),
		frame_id_(DEFAULT_FRAME_ID) {

	mpedController = controller;
	
    n_param_.getParam ( "frequency", frequency_ );
    ROS_INFO ( "frequency: %5.2f", frequency_ );
    
    n_param_.getParam ( "publish", publish_ );
    ROS_INFO ( "publish: %s", (publish_?"true":"false") );
    
    n_param_.getParam ( "frame_id", frame_id_ );
    ROS_INFO ( "frame_id: %s", frame_id_.c_str() );
    
	sub_ = n_.subscribe( NAME_SUB, 1, &HumanflowBridgeNode::robotPoseCallback, this );
	pubHuman_ = n_param_.advertise<transitbuddy_msgs::PoseWithIDArray> ( "human_pose", 1 );
	pubWalls_ = n_param_.advertise<transitbuddy_msgs::LineWithIDArray> ( "walls", 1 );
}

HumanflowBridgeNode::~HumanflowBridgeNode(){
	}

void HumanflowBridgeNode::robotPoseCallback(const transitbuddy_msgs::PoseWithIDArrayConstPtr& msg){
    ROS_INFO ( "robotPoseCallback");
	
	//for (auto iter = msg->poses.begin(); iter != msg->poses.end(); iter++)
	for (unsigned int i=0; i < msg->poses.size(); i++)
	{
		// TODO: Ignore agents from other section
		long id = msg->poses[i].id + ROBOT_ID_OFFSET;
		if (std::find(robotIds.begin(), robotIds.end(), id) == robotIds.end()) {
			// No robot added yet, do it now
			ROS_INFO("adding robot: %d", id);
			mped::MpedMessage *addAgentsMessage = mpedController.createMessage("ADD_AGENTS");
			vector<long> newIds;
			newIds.push_back(id);
			addAgentsMessage->setLongArray("ids", newIds);
			stringstream section;
			section << "s" << id;
			addAgentsMessage->setLong(section.str(), sectionId);
			stringstream type;
			type << "t" << id;
			addAgentsMessage->setString(type.str(), "robot");
			stringstream radius;
			radius << "ra" << id;
			addAgentsMessage->setDouble(radius.str(), 0.25);
			stringstream color;
			color << "color" << id;
			addAgentsMessage->setString(color.str(), "#0000FF");
			mpedController.sendMessage(addAgentsMessage);
			delete addAgentsMessage;

			// save the id
			robotIds.push_back(id);
		}
		
		// Update position
		mped::MpedMessage *updateAgentMsg = mpedController.createMessage("UPDATE_AGENTS");
		vector<long> newIds;
		newIds.push_back(id);
		updateAgentMsg->setLongArray("ids", newIds);
		//stringstream section;
		//section << "s" << id;
		//msg->setLong(section.str(), sectionId);
		//stringstream type;
		//type << "t" << id;
		//msg->setString(type.str(), "robot");
		stringstream posKey;
		posKey << "p" << id;
		vector<double> pos;
		pos.push_back(msg->poses[i].pose.position.x);
		pos.push_back(msg->poses[i].pose.position.y);
		pos.push_back(msg->poses[i].pose.position.z);
		updateAgentMsg->setDoubleArray(posKey.str(), pos);
		mpedController.sendMessage(updateAgentMsg);
		ROS_INFO("Update robot: %d <%5.2f, %5.2f, %5.2f>", id, msg->poses[i].pose.position.x, msg->poses[i].pose.position.y, msg->poses[i].pose.position.z);
		delete updateAgentMsg;
	}

	for (unsigned int i=0; i < robotIds.size(); i++) {
		long robotId = robotIds[i];
		bool hasPose = false;
		for (auto posesIter = msg->poses.begin(); posesIter != msg->poses.end(); posesIter++) {
			if (posesIter->id + ROBOT_ID_OFFSET == robotId) {
				hasPose = true;
				break;
			}
		}

		if (hasPose == false) {
			// Robot does not exist anymore, remove it from the simulation
			ROS_INFO("removing robot: %d", robotId);
			mped::MpedMessage *removeAgentsMsg = mpedController.createMessage("REMOVE_AGENTS");
			vector<long> newIds;
			newIds.push_back(robotId);
			removeAgentsMsg->setLongArray("ids", newIds);
			mpedController.sendMessage(removeAgentsMsg);
			delete removeAgentsMsg;

			// TODO: Remove value from robotIds so it can be used again
		}
	}
}

void HumanflowBridgeNode::handleMpedMessage(mped::MpedMessage &msg){
	
	cout << msg.getMessageType() << endl;
	if (msg.isInitMessage())
	{
		//onInit(msg);
		sectionId = mpedController.getAssociatedSection();
		ROS_DEBUG("Module associated with section %d", sectionId);
		if (sectionId == 0)
			mpedController.warn("Module is not associated with a section! It should be moved to the section where the robot is created");

		publishInfrastructurePose();
	}
	else if (msg.isNextStepMessage())
	{
		//onNextStep(msg->getLong("step"), msg->getDouble("stepDuration"), msg->getDouble("simulationDuration"));
		long step = msg.getLong("step");
		// and send the info to the ROS server
		if (step % 10 == 0) publishHumanPose(); // Every x time			
		// if (step % 200 == 0) publishInfrastructurePose(); // Every x th time
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
		cout << "UPDATE_AGENTS" << endl;
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
	pubHuman_.publish(poses);
}

/**
 * Publish all obstruction lines which helps synchronizing the infrastructure
 */
void HumanflowBridgeNode::publishInfrastructurePose() {
	if(publish_ == false) return;
    lines.header.stamp = ros::Time::now();
    lines.header.frame_id = frame_id_;
	
	if (lines.lines.empty()) // only calculate the first time
	{
		long sectionId = mpedController.getAssociatedSection();
		// Add all obstruction lines here
		mped::Section *section = mpedController.getInfrastructureSection(sectionId);
		if (section == NULL)
			return; // No associated section

		long lineId = 1000;
		std::vector<long> obstructionRegions = section->getObstructionRegions();
		for (auto regionId = obstructionRegions.begin(); regionId != obstructionRegions.end(); regionId++)
		{
			mped::Region* region = mpedController.getInfrastructureRegion(*regionId);
			if (region != NULL)
			{
				std::vector<long> shapeIds = region->getShapes();
				for (auto shapeId = shapeIds.begin(); shapeId != shapeIds.end(); shapeId++)
				{
					mped::Shape* shape = mpedController.getInfrastructureShape(*shapeId);
					if (shape != NULL) {
						std::vector<double> x = shape->getX();
						std::vector<double> y = shape->getY();
						double z = shape->getZ();
						for (unsigned int i=0; i < x.size(); i++) {
							int endIndex=i+1;
							transitbuddy_msgs::LineWithID line;
							geometry_msgs::Point start;
							geometry_msgs::Point end;
							if (i == x.size()-1) // The last point: For polygons, wrap around
							{
								if (shape->getType() == "polygon")
									endIndex = 0;
								else
									break;
							}
						
							start.x = x[i];
							start.y = y[i];
							start.z = z;
							end.x = x[endIndex];
							end.y = y[endIndex];
							end.z = z;

							line.id = lineId++;
							line.p.push_back(start);
							line.p.push_back(end);
							lines.lines.push_back(line);
							// cout << "x:" << x.at(i) << " ";
							// cout << "y:" << y.at(i) << " ";
							// cout << endl;
						}
						delete shape;
					}
				}
			}
			delete region;
		}
		delete section;
	}

	ROS_DEBUG("publishInfrastructurePose: %d lines", lines.lines.size());
	printf("publishInfrastructurePose: %d lines\n", lines.lines.size());
	fflush(stdout);
	pubWalls_.publish(lines);
}
