#pragma once

#include <string>
#include <vector>

using namespace std;

namespace mped {
	class Scenario {

	private:
		vector<long> sections;
		vector<long> basins;
		vector<long> facilities;
		vector<long> tools;
		vector<long> basinConnections;
		vector<long> agents;
		vector<long> parameters;

	public:
		__declspec(dllexport) Scenario();
		__declspec(dllexport) ~Scenario(void);
		
		__declspec(dllexport) vector<long> getSections();
		__declspec(dllexport) void setSections( const vector<long>& sections);

		__declspec(dllexport) vector<long> getBasins();
		__declspec(dllexport) void setBasins( const vector<long>& basins);

		__declspec(dllexport) vector<long> getFacilities();
		__declspec(dllexport) void setFacilities( const vector<long>& facilities);

		__declspec(dllexport) vector<long> getTools();
		__declspec(dllexport) void setTools( const vector<long>& tools);

		__declspec(dllexport) vector<long> getBasinConnections();
		__declspec(dllexport) void setBasinConnections( const vector<long>& basinConnections);

		__declspec(dllexport) vector<long> getAgents();
		__declspec(dllexport) void setAgents( const vector<long>& agents);

		__declspec(dllexport) vector<long> getParameters();
		__declspec(dllexport) void setParameters( const vector<long>& parameters);

	};
}