#pragma once

#include <string>
#include <vector>

using namespace std;

namespace mped {
	class ToolingRegion {

	private:
		long id;
		string name;
		vector<long> tools;
		vector<long> parameters;

	public:
		__declspec(dllexport) ToolingRegion();
		__declspec(dllexport) ~ToolingRegion(void);
		
		__declspec(dllexport) long getId();
		__declspec(dllexport) void setId( const long& id);

		__declspec(dllexport) string getName();
		__declspec(dllexport) void setName( const string& name);

		__declspec(dllexport) vector<long> getTools();
		__declspec(dllexport) void setTools( const vector<long>& tools);

		__declspec(dllexport) vector<long> getParameters();
		__declspec(dllexport) void setParameters( const vector<long>& parameters);

	};
}