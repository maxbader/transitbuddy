#pragma once

#include <string>
#include <vector>

using namespace std;

namespace mped {
	class Shape {

	private:
		long id;
		string type;
		string style;
		vector<double> x;
		vector<double> y;
		double z;
		double height;

	public:
		__declspec(dllexport) Shape();
		__declspec(dllexport) ~Shape(void);
		
		__declspec(dllexport) long getId();
		__declspec(dllexport) void setId( const long& id);

		__declspec(dllexport) string getType();
		__declspec(dllexport) void setType( const string& type);

		__declspec(dllexport) string getStyle();
		__declspec(dllexport) void setStyle( const string& style);

		__declspec(dllexport) vector<double> getX();
		__declspec(dllexport) void setX( const vector<double>& x);

		__declspec(dllexport) vector<double> getY();
		__declspec(dllexport) void setY( const vector<double>& y);

		__declspec(dllexport) double getZ();
		__declspec(dllexport) void setZ( const double& z);

		__declspec(dllexport) double getHeight();
		__declspec(dllexport) void setHeight( const double& height);

	};
}