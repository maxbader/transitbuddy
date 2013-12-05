#pragma once

#include <string>
#include <vector>

using namespace std;

namespace mped {
	class Node {

	private:
		long id;
		double x;
		double y;
		double z;
		vector<long> links;
		vector<long> parameters;

	public:
		__declspec(dllexport) Node();
		__declspec(dllexport) ~Node(void);
		
		__declspec(dllexport) long getId();
		__declspec(dllexport) void setId( const long& id);

		__declspec(dllexport) double getX();
		__declspec(dllexport) void setX( const double& x);

		__declspec(dllexport) double getY();
		__declspec(dllexport) void setY( const double& y);

		__declspec(dllexport) double getZ();
		__declspec(dllexport) void setZ( const double& z);

		__declspec(dllexport) vector<long> getLinks();
		__declspec(dllexport) void setLinks( const vector<long>& links);

		__declspec(dllexport) vector<long> getParameters();
		__declspec(dllexport) void setParameters( const vector<long>& parameters);

	};
}