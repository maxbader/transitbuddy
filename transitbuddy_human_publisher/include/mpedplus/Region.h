#pragma once

#include <string>
#include <vector>

using namespace std;

namespace mped {
	class Region {

	private:
		long id;
		long item;
		vector<long> shapes;
		vector<long> cells;
		vector<long> nodes;

	public:
		__declspec(dllexport) Region();
		__declspec(dllexport) ~Region(void);
		
		__declspec(dllexport) long getId();
		__declspec(dllexport) void setId( const long& id);

		__declspec(dllexport) long getItem();
		__declspec(dllexport) void setItem( const long& item);

		__declspec(dllexport) vector<long> getShapes();
		__declspec(dllexport) void setShapes( const vector<long>& shapes);

		__declspec(dllexport) vector<long> getCells();
		__declspec(dllexport) void setCells( const vector<long>& cells);

		__declspec(dllexport) vector<long> getNodes();
		__declspec(dllexport) void setNodes( const vector<long>& nodes);

	};
}