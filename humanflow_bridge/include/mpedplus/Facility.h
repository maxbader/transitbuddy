#pragma once

#include <string>
#include <vector>

using namespace std;

namespace mped {
	class Facility {

	private:
		long id;
		string name;
		string type;
		vector<long> basins;
		vector<long> parameters;

	public:
		__declspec(dllexport) Facility();
		__declspec(dllexport) ~Facility(void);
		
		__declspec(dllexport) long getId();
		__declspec(dllexport) void setId( const long& id);

		__declspec(dllexport) string getName();
		__declspec(dllexport) void setName( const string& name);

		__declspec(dllexport) string getType();
		__declspec(dllexport) void setType( const string& type);

		__declspec(dllexport) vector<long> getBasins();
		__declspec(dllexport) void setBasins( const vector<long>& basins);

		__declspec(dllexport) vector<long> getParameters();
		__declspec(dllexport) void setParameters( const vector<long>& parameters);

	};
}