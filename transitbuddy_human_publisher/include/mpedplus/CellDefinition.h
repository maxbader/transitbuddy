#pragma once

#include <string>
#include <vector>

using namespace std;

namespace mped {
	class CellDefinition {

	private:
		long id;
		long rows;
		long columns;
		double width;
		double height;
		vector<double> origin;

	public:
		__declspec(dllexport) CellDefinition();
		__declspec(dllexport) ~CellDefinition(void);
		
		__declspec(dllexport) long getId();
		__declspec(dllexport) void setId( const long& id);

		__declspec(dllexport) long getRows();
		__declspec(dllexport) void setRows( const long& rows);

		__declspec(dllexport) long getColumns();
		__declspec(dllexport) void setColumns( const long& columns);

		__declspec(dllexport) double getWidth();
		__declspec(dllexport) void setWidth( const double& width);

		__declspec(dllexport) double getHeight();
		__declspec(dllexport) void setHeight( const double& height);

		__declspec(dllexport) vector<double> getOrigin();
		__declspec(dllexport) void setOrigin( const vector<double>& origin);

	};
}