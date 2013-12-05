#pragma once

#include <string>
#include <vector>

using namespace std;

namespace mped {
	class Link {

	private:
		long id;
		long start;
		long end;
		double distance;
		vector<long> parameters;

	public:
		__declspec(dllexport) Link();
		__declspec(dllexport) ~Link(void);
		
		__declspec(dllexport) long getId();
		__declspec(dllexport) void setId( const long& id);

		__declspec(dllexport) long getStart();
		__declspec(dllexport) void setStart( const long& start);

		__declspec(dllexport) long getEnd();
		__declspec(dllexport) void setEnd( const long& end);

		__declspec(dllexport) double getDistance();
		__declspec(dllexport) void setDistance( const double& id);

		__declspec(dllexport) vector<long> getParameters();
		__declspec(dllexport) void setParameters( const vector<long>& parameters);

	};
}