#pragma once

#ifdef MPED_API_BUILD
	#include <exception>
	#include <stdlib.h>
	#include <activemq/library/ActiveMQCPP.h>
	#include <cms/MapMessage.h>
#else // just do some forward declarations for declared pointers
	class MapMessage;
#endif

#include <string>
#include <iostream>
#include <vector>

using namespace std;

#ifdef MPED_API_BUILD
	using namespace cms;
#endif


namespace mped {
	class MpedMessage
	{
	public:
		static const string ARRAY_KEY_PREFIX;

		static const string INIT_STEP_MESSAGE_TYPE;
		static const string INIT_STEP_DONE_MESSAGE_TYPE;
		static const string NEXT_STEP_MESSAGE_TYPE;
		static const string NEXT_STEP_DONE_MESSAGE_TYPE;
		static const string SHUTDOWN_MESSAGE_TYPE;
		static const string SHUTDOWN_DONE_MESSAGE_TYPE;
		static const string KILL_MESSAGE_TYPE;

	private:
		MapMessage* jmsMessage;

	public:
		__declspec(dllexport) MpedMessage(MapMessage* jmsMessage);
		__declspec(dllexport) ~MpedMessage(void);

	public:

		__declspec(dllexport) MapMessage* getCmsMessage();

		__declspec(dllexport) void setGlobalFlag();

		__declspec(dllexport) string getMessageType();
		__declspec(dllexport) void setMessageType(const string& messageType);

		__declspec(dllexport) string getSection();

		__declspec(dllexport) void setMessageTypeInitDone();
		__declspec(dllexport) void setMessageTypeNextStepDone();

		__declspec(dllexport) bool equals(const string& messageType);
		__declspec(dllexport) bool isInitMessage();
		__declspec(dllexport) bool isNextStepMessage();
		__declspec(dllexport) bool isShutdownMessage();
		__declspec(dllexport) bool isKillMessage();

		__declspec(dllexport) vector<string> getKeys();

		__declspec(dllexport) string getString( const string& key);
		__declspec(dllexport) void setString( const string& key, const string value);

		__declspec(dllexport) vector<string> getStringArray( const string& key);
		__declspec(dllexport) void setStringArray (const string& key, const vector<string>& value);

		__declspec(dllexport) long getLong( const string& key );
		__declspec(dllexport) void setLong( const string& key, const long value);

		__declspec(dllexport) vector<long> getLongArray( const string& key );
		__declspec(dllexport) void setLongArray (const string& key, const vector<long>& value);
		// If we also want to support an overloaded set*Array() with C-arrays, we need to make it for ALL data types or for none.
		//__declspec(dllexport) void setLongArray (const string &key, const long *value, const long &size);

		__declspec(dllexport) double getDouble( const string& key );
		__declspec(dllexport) void setDouble( const string& key, const double value);

		__declspec(dllexport) vector<double> getDoubleArray( const string& key );
		__declspec(dllexport) void setDoubleArray (const string& key, const vector<double>& value);

		__declspec(dllexport) bool getBool( const string& key );
		__declspec(dllexport) void setBool( const string& key, const bool value);

		__declspec(dllexport) vector<bool> getBoolArray( const string& key );
		__declspec(dllexport) void setBoolArray (const string& key, const vector<bool>& value);
		
		__declspec(dllexport) vector<unsigned char> getBytes( const string& key );
		__declspec(dllexport) void setBytes( const string& key, const vector<unsigned char>& value);
	};
}

#ifdef MPED_MESSAGE_H_CONSTANTS
	using namespace mped;
	const string MpedMessage::ARRAY_KEY_PREFIX = "ARR";
	const string MpedMessage::INIT_STEP_MESSAGE_TYPE = "INIT";
	const string MpedMessage::INIT_STEP_DONE_MESSAGE_TYPE = "INIT_DONE";
	const string MpedMessage::NEXT_STEP_MESSAGE_TYPE = "NEXT_STEP";
	const string MpedMessage::NEXT_STEP_DONE_MESSAGE_TYPE = "NEXT_STEP_DONE";
	const string MpedMessage::SHUTDOWN_MESSAGE_TYPE = "SHUTDOWN";
	const string MpedMessage::SHUTDOWN_DONE_MESSAGE_TYPE = "SHUTDOWN_DONE";
	const string MpedMessage::KILL_MESSAGE_TYPE = "KILL";
#endif