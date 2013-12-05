#pragma once

#include <string>
#include <vector>

using namespace std;

namespace mped {
	class Section {

	private:
		long id;
		string name;
		vector<double> origin;
		vector<double> size;
		double rotation;
		vector<long> shapes;
		long cell;
		vector<long> nodes;
		vector<long> links;
		long boundingRegion;
		vector<long> obstructionRegions;
		vector<long> basinRegions;
		vector<long> facilityRegions;
		vector<long> toolingRegions;
		vector<long> parameters;

	public:
		__declspec(dllexport) Section();
		__declspec(dllexport) ~Section(void);

		__declspec(dllexport) long getId();
		__declspec(dllexport) void setId( const long& id);

		__declspec(dllexport) string getName();
		__declspec(dllexport) void setName( const string& name);
		
		__declspec(dllexport) vector<double> getOrigin();
		__declspec(dllexport) void setOrigin( const vector<double>& origin);

		__declspec(dllexport) vector<double> getSize();
		__declspec(dllexport) void setSize( const vector<double>& size);

		__declspec(dllexport) double getRotation();
		__declspec(dllexport) void setRotation( const double& rotation);

		__declspec(dllexport) vector<long> getShapes();
		__declspec(dllexport) void setShapes( const vector<long>& shapes);

		__declspec(dllexport) long getCell();
		__declspec(dllexport) void setCell( const long& cell);

		__declspec(dllexport) vector<long> getNodes();
		__declspec(dllexport) void setNodes( const vector<long>& nodes);

		__declspec(dllexport) vector<long> getLinks();
		__declspec(dllexport) void setLinks( const vector<long>& links);

		__declspec(dllexport) long getBoundingRegion();
		__declspec(dllexport) void setBoundingRegion( const long& boundingRegion);
		
		__declspec(dllexport) vector<long> getObstructionRegions();
		__declspec(dllexport) void setObstructionRegions( const vector<long>& obstructionRegions);

		__declspec(dllexport) vector<long> getBasinRegions();
		__declspec(dllexport) void setBasinRegions( const vector<long>& basinRegions);

		__declspec(dllexport) vector<long> getFacilityRegions();
		__declspec(dllexport) void setFacilityRegions( const vector<long>& facilityRegions);

		__declspec(dllexport) vector<long> getToolingRegions();
		__declspec(dllexport) void setToolingRegions( const vector<long>& toolingRegions);

		__declspec(dllexport) vector<long> getParameters();
		__declspec(dllexport) void setParameters( const vector<long>& parameters);

	};
}