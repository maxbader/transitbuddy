#pragma once

#ifdef MPED_API_BUILD
	#include <activemq/library/ActiveMQCPP.h>
	#include <decaf/lang/Thread.h>
	#include <decaf/lang/Runnable.h>
	#include <decaf/util/concurrent/CountDownLatch.h>
	#include <decaf/lang/Integer.h>
	#include <decaf/lang/Long.h>
	#include <decaf/lang/System.h>
	#include <activemq/core/ActiveMQConnectionFactory.h>
	#include <activemq/util/Config.h>
	#include <cms/Connection.h>
	#include <cms/Session.h>
	#include <cms/TextMessage.h>
	#include <cms/BytesMessage.h>
	#include <cms/MapMessage.h>
	#include <cms/ExceptionListener.h>
	#include <cms/MessageListener.h>
	#include <stdlib.h>
	#include <stdio.h>
	#include <iostream>
	#include <memory>
	#include <vector>
	#include <string>
	#include <exception>

	using namespace activemq::core;
	using namespace decaf::util::concurrent;
	using namespace decaf::util;
	using namespace decaf::lang;
#else // just use some forward declaration for declared pointers
	class Connection;
	class Session;
	class Destination;
	class MessageProducer;
	class MessageConsumer;
#endif

#include "MpedMessage.h"

#include "AgentDefinition.h"
#include "Basin.h"
#include "BasinConnection.h"
#include "CellDefinition.h"
#include "Facility.h"
#include "Link.h"
#include "Node.h"
#include "ODEntry.h"
#include "Parameter.h"
#include "Region.h"
#include "Scenario.h"
#include "Section.h"
#include "Shape.h"
#include "Tool.h"
#include "ToolingRegion.h"

#include "StringParameter.h"
#include "LongParameter.h"
#include "DoubleParameter.h"
#include "BooleanParameter.h"

#ifdef MPED_API_BUILD
using namespace cms;
#endif

using namespace std;

namespace mped {
	class MpedClientAPI
	{
	private:
		static const string DYNAMIC_MODULE_REGISTRATION_QUEUE;

		static const string DYNAMIC_REGISTRATION_INIT_MESSAGE_TYPE;
		static const string DYNAMIC_REGISTRATION_EXIT_MESSAGE_TYPE;
		static const string DYNAMIC_REGISTRATION_DESCRIPTOR_PARAMETER_NAME;
		static const string DYNAMIC_REGISTRATION_INSTANCE_UID_PARAMETER_NAME;
		static const string DYNAMIC_REGISTRATION_SECTION_UID_PARAMETER_NAME;

		static const string RESULT_QUEUE;
		static const string LOGGING_DEBUG_QUEUE;
		static const string LOGGING_INFO_QUEUE;
		static const string LOGGING_WARN_QUEUE;
		static const string LOGGING_ERROR_QUEUE;
		static const string WATCHER_QUEUE_NAME;

		static const string GET_NEXT_GUID_MESSAGE_TYPE;

		static const string GET_MODULE_SECTION_ID_MESSAGE_TYPE;

		static const string GET_BOOLEAN_PARAMETER_MESSAGE_TYPE;
		static const string GET_STRING_PARAMETER_MESSAGE_TYPE;
		static const string GET_LONG_PARAMETER_MESSAGE_TYPE;
		static const string GET_DOUBLE_PARAMETER_MESSAGE_TYPE;

		static const string GET_STRING_PARAMETER_VALUE_ATT;
		static const string GET_STRING_PARAMETER_PATTERN_ATT;
		static const string GET_STRING_PARAMETER_MIN_ATT;
		static const string GET_STRING_PARAMETER_MAX_ATT;
		static const string GET_LONG_PARAMETER_VALUE_ATT;
		static const string GET_LONG_PARAMETER_MIN_ATT;
		static const string GET_LONG_PARAMETER_MAX_ATT;
		static const string GET_DOUBLE_PARAMETER_VALUE_ATT;
		static const string GET_DOUBLE_PARAMETER_MIN_INCL_ATT;
		static const string GET_DOUBLE_PARAMETER_MAX_INCL_ATT;
		static const string GET_DOUBLE_PARAMETER_MIN_EXCL_ATT;
		static const string GET_DOUBLE_PARAMETER_MAX_EXCL_ATT;
		static const string GET_BOOLEAN_PARAMETER_VALUE_ATT;

		static const string GET_INFRASTRUCTURE_IS_NULL_ATTRIBUTE;
		static const string GET_INFRASTRUCTURE_AGENT_MESSAGE_TYPE;
		static const string GET_INFRASTRUCTURE_BASIN_MESSAGE_TYPE;
		static const string GET_INFRASTRUCTURE_BASIN_CONNECTION_MESSAGE_TYPE;
		static const string GET_INFRASTRUCTURE_CELL_MESSAGE_TYPE;
		static const string GET_INFRASTRUCTURE_FACILITY_MESSAGE_TYPE;
		static const string GET_INFRASTRUCTURE_LINK_MESSAGE_TYPE;
		static const string GET_INFRASTRUCTURE_NODE_MESSAGE_TYPE;
		static const string GET_INFRASTRUCTURE_ODENTRY_MESSAGE_TYPE;
		static const string GET_INFRASTRUCTURE_PARAMETER_MESSAGE_TYPE;
		static const string GET_INFRASTRUCTURE_REGION_MESSAGE_TYPE;
		static const string GET_INFRASTRUCTURE_SCENARIO_MESSAGE_TYPE;
		static const string GET_INFRASTRUCTURE_SECTION_MESSAGE_TYPE;
		static const string GET_INFRASTRUCTURE_SHAPE_MESSAGE_TYPE;
		static const string GET_INFRASTRUCTURE_TOOL_MESSAGE_TYPE;
		static const string GET_INFRASTRUCTURE_TOOLING_REGION_MESSAGE_TYPE;

	private:
		static const int STATE_NULL = 0;
		static const int STATE_INIT = 1;
		static const int STATE_NEXT_STEP = 2;
		static const int STATE_SHUTDOWN = 3;

	private:
		string			uid;
		string			sectionName;
		bool			dynamicRegistrationMode;

		int				 state;
		//MpedMessage*     lastReceivedMessage;

		Connection*      connection;
		Session*         session;

		Destination*     dynamicRegistrationQueueDestination;
		MessageProducer* dynamicRegistrationQueueMessageProducer;
		
		Destination*     resultQueueDestination;
		MessageProducer* resultQueueMessageProducer;

		Destination*     clientUidDestination;
		MessageConsumer* clientUidConsumer;

		Destination*     clientUidDestinationSynchron;
		MessageConsumer* clientUidConsumerSynchron;

		Destination*     loggingQueueDebugDestination;
		MessageProducer* loggingQueueDebugMessageProducer;

		Destination*     loggingQueueInfoDestination;
		MessageProducer* loggingQueueInfoMessageProducer;

		Destination*     loggingQueueWarnDestination;
		MessageProducer* loggingQueueWarnMessageProducer;

		Destination*     loggingQueueErrorDestination;
		MessageProducer* loggingQueueErrorMessageProducer;

		Destination*     watcherQueueDestination;
		MessageProducer* watcherQueueMessageProducer;

	public:
		__declspec(dllexport) MpedClientAPI();
		__declspec(dllexport) ~MpedClientAPI(void);

		__declspec(dllexport) void debugRun(const string& frameworkUrl, const string& moduleDescrString, const string& instanceUid, const string& sectionName);
		__declspec(dllexport) void run(const string& frameworkUrl, const string& instanceUid, const string& sectionName);

		__declspec(dllexport) bool isDynamicRegistrationMode();

		__declspec(dllexport) void debug(const string& logMsg);
		__declspec(dllexport) void info(const string& logMsg);
		__declspec(dllexport) void warn(const string& logMsg);
		__declspec(dllexport) void error(const string& logMsg);

		__declspec(dllexport) void setWatcherValue(const string& key, const string& value);

		__declspec(dllexport) Parameter* getParameter(const string& key);
		__declspec(dllexport) StringParameter* getStringParameter(const string& key);
		__declspec(dllexport) LongParameter* getLongParameter(const string& key);
		__declspec(dllexport) DoubleParameter* getDoubleParameter(const string& key);
		__declspec(dllexport) BooleanParameter* getBooleanParameter(const string& key);

		__declspec(dllexport) long getNextGUID();
		__declspec(dllexport) MpedMessage* waitForNextMessage();

	private:
		void updateState(MpedMessage *message);

	public:

		__declspec(dllexport) long getAssociatedSection();

		__declspec(dllexport) AgentDefinition* getInfrastructureAgentDefinition(const long id);
		__declspec(dllexport) Basin* getInfrastructureBasin(const long id);
		__declspec(dllexport) BasinConnection* getInfrastructureBasinConnection(const long id);
		__declspec(dllexport) CellDefinition* getInfrastructureCellDefinition(const long id);
		__declspec(dllexport) Facility* getInfrastructureFacility(const long id);
		__declspec(dllexport) Link* getInfrastructureLink(const long id);
		__declspec(dllexport) Node* getInfrastructureNode(const long id);
		__declspec(dllexport) ODEntry* getInfrastructureODEntry(const long id);
		__declspec(dllexport) Parameter* getInfrastructureParameter(const long id);
		__declspec(dllexport) Region* getInfrastructureRegion(const long id);
		__declspec(dllexport) Scenario* getInfrastructure();
		__declspec(dllexport) Section* getInfrastructureSection(const long id);
		__declspec(dllexport) Shape* getInfrastructureShape(const long id);
		__declspec(dllexport) Tool* getInfrastructureTool(const long id);
		__declspec(dllexport) ToolingRegion* getInfrastructureToolingRegion(const long id);

		__declspec(dllexport) MpedMessage* createMessage(const string& messageType);
		__declspec(dllexport) void sendMessage(MpedMessage *message);

		__declspec(dllexport) void sendDoneMessage();

	private:
		void init(Session *session, const string& instanceUid, const string& sectionUid);
	};
}

#ifdef MPED_CLIENT_API_H_CONSTANTS
	using namespace mped;
	const string MpedClientAPI::DYNAMIC_MODULE_REGISTRATION_QUEUE = "DYNREG_QUEUE";

	const string MpedClientAPI::DYNAMIC_REGISTRATION_INIT_MESSAGE_TYPE = "DYNREG_INIT";
	const string MpedClientAPI::DYNAMIC_REGISTRATION_EXIT_MESSAGE_TYPE = "DYNREG_EXIT";
	const string MpedClientAPI::DYNAMIC_REGISTRATION_DESCRIPTOR_PARAMETER_NAME = "DYNREG_DESCR";
	const string MpedClientAPI::DYNAMIC_REGISTRATION_INSTANCE_UID_PARAMETER_NAME = "DYNREG_INSTANCE";
	const string MpedClientAPI::DYNAMIC_REGISTRATION_SECTION_UID_PARAMETER_NAME = "DYNREG_SECTION";

	const string MpedClientAPI::RESULT_QUEUE = "RESULT_QUEUE";
	const string MpedClientAPI::LOGGING_DEBUG_QUEUE = "LOGGING_DEBUG_QUEUE";
	const string MpedClientAPI::LOGGING_INFO_QUEUE = "LOGGING_INFO_QUEUE";
	const string MpedClientAPI::LOGGING_WARN_QUEUE = "LOGGING_WARN_QUEUE";
	const string MpedClientAPI::LOGGING_ERROR_QUEUE = "LOGGING_ERROR_QUEUE";
	const string MpedClientAPI::WATCHER_QUEUE_NAME = "WATCHER_QUEUE";

	const string MpedClientAPI::GET_NEXT_GUID_MESSAGE_TYPE = "GET_NEXT_GUID";

	const string MpedClientAPI::GET_MODULE_SECTION_ID_MESSAGE_TYPE = "GET_MODULE_SECTION_ID";

	const string MpedClientAPI::GET_BOOLEAN_PARAMETER_MESSAGE_TYPE = "GET_BOOLEAN_PARAM";
	const string MpedClientAPI::GET_STRING_PARAMETER_MESSAGE_TYPE = "GET_STRING_PARAM";
	const string MpedClientAPI::GET_LONG_PARAMETER_MESSAGE_TYPE = "GET_LONG_PARAM";
	const string MpedClientAPI::GET_DOUBLE_PARAMETER_MESSAGE_TYPE = "GET_DOUBLE_PARAM";

    const string MpedClientAPI::GET_STRING_PARAMETER_VALUE_ATT                   = "STRING_PARAM_VALUE";
    const string MpedClientAPI::GET_STRING_PARAMETER_PATTERN_ATT                 = "STRING_PARAM_PATTERN";
	const string MpedClientAPI::GET_STRING_PARAMETER_MIN_ATT                     = "STRING_PARAM_MIN";
	const string MpedClientAPI::GET_STRING_PARAMETER_MAX_ATT                     = "STRING_PARAM_MAX";
	const string MpedClientAPI::GET_LONG_PARAMETER_VALUE_ATT                     = "LONG_PARAM_VALUE";
	const string MpedClientAPI::GET_LONG_PARAMETER_MIN_ATT                       = "LONG_PARAM_MIN";
	const string MpedClientAPI::GET_LONG_PARAMETER_MAX_ATT                       = "LONG_PARAM_MAX";
	const string MpedClientAPI::GET_DOUBLE_PARAMETER_VALUE_ATT                   = "DOUBLE_PARAM_VALUE";
	const string MpedClientAPI::GET_DOUBLE_PARAMETER_MIN_INCL_ATT                = "DOUBLE_PARAM_MIN_INCL";
	const string MpedClientAPI::GET_DOUBLE_PARAMETER_MAX_INCL_ATT                = "DOUBLE_PARAM_MAX_INCL";
	const string MpedClientAPI::GET_DOUBLE_PARAMETER_MIN_EXCL_ATT                = "DOUBLE_PARAM_MIN_EXCL";
	const string MpedClientAPI::GET_DOUBLE_PARAMETER_MAX_EXCL_ATT                = "DOUBLE_PARAM_MAX_EXCL";
	const string MpedClientAPI::GET_BOOLEAN_PARAMETER_VALUE_ATT                  = "BOOLEAN_PARAMETER_VALUE";

	const string MpedClientAPI::GET_INFRASTRUCTURE_IS_NULL_ATTRIBUTE             = "GET_INFRASTRUCTURE_IS_NULL";
    const string MpedClientAPI::GET_INFRASTRUCTURE_AGENT_MESSAGE_TYPE            = "GET_INFRASTRUCTURE_AGENT";
	const string MpedClientAPI::GET_INFRASTRUCTURE_BASIN_MESSAGE_TYPE            = "GET_INFRASTRUCTURE_BASIN";
	const string MpedClientAPI::GET_INFRASTRUCTURE_BASIN_CONNECTION_MESSAGE_TYPE = "GET_INFRASTRUCTURE_BASIN_CONNECTION";
	const string MpedClientAPI::GET_INFRASTRUCTURE_CELL_MESSAGE_TYPE             = "GET_INFRASTRUCTURE_CELL";
	const string MpedClientAPI::GET_INFRASTRUCTURE_FACILITY_MESSAGE_TYPE         = "GET_INFRASTRUCTURE_FACILITY";
	const string MpedClientAPI::GET_INFRASTRUCTURE_LINK_MESSAGE_TYPE             = "GET_INFRASTRUCTURE_LINK";
	const string MpedClientAPI::GET_INFRASTRUCTURE_NODE_MESSAGE_TYPE             = "GET_INFRASTRUCTURE_NODE";
	const string MpedClientAPI::GET_INFRASTRUCTURE_ODENTRY_MESSAGE_TYPE          = "GET_INFRASTRUCTURE_ODENTRY";
	const string MpedClientAPI::GET_INFRASTRUCTURE_PARAMETER_MESSAGE_TYPE        = "GET_INFRASTRUCTURE_PARAMETER";
	const string MpedClientAPI::GET_INFRASTRUCTURE_REGION_MESSAGE_TYPE           = "GET_INFRASTRUCTURE_REGION";
	const string MpedClientAPI::GET_INFRASTRUCTURE_SCENARIO_MESSAGE_TYPE         = "GET_INFRASTRUCTURE_SCENARIO";
	const string MpedClientAPI::GET_INFRASTRUCTURE_SECTION_MESSAGE_TYPE          = "GET_INFRASTRUCTURE_SECTION";
	const string MpedClientAPI::GET_INFRASTRUCTURE_SHAPE_MESSAGE_TYPE            = "GET_INFRASTRUCTURE_SHAPE";
	const string MpedClientAPI::GET_INFRASTRUCTURE_TOOL_MESSAGE_TYPE             = "GET_INFRASTRUCTURE_TOOL";
	const string MpedClientAPI::GET_INFRASTRUCTURE_TOOLING_REGION_MESSAGE_TYPE   = "GET_INFRASTRUCTURE_TOOLING_REGION";
#endif