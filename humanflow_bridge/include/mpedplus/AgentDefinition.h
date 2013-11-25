#pragma once

#include <string>
#include <vector>

using namespace std;

namespace mped {
	class AgentDefinition {

	private:
		long id;
		string name;
		string type;
		vector<long> odEntries;
		vector<long> parameters;

	public:
		__declspec(dllexport) AgentDefinition();
		__declspec(dllexport) ~AgentDefinition(void);
		
		__declspec(dllexport) long getId();
		__declspec(dllexport) void setId( const long& id);

		__declspec(dllexport) string getName();
		__declspec(dllexport) void setName( const string& name);

		__declspec(dllexport) string getType();
		__declspec(dllexport) void setType( const string& type);

		__declspec(dllexport) vector<long> getOdEntries();
		__declspec(dllexport) void setOdEntries( const vector<long>& odEntries);

		__declspec(dllexport) vector<long> getParameters();
		__declspec(dllexport) void setParameters( const vector<long>& parameters);

	};
}