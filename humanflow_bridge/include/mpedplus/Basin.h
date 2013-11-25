#pragma once

#include <string>
#include <vector>

using namespace std;

namespace mped {
	class Basin {

	private:
		long id;
		string name;
		string type;
		vector<long> connections;
		vector<long> parameters;

	public:
		__declspec(dllexport) Basin();
		__declspec(dllexport) ~Basin(void);
		
		__declspec(dllexport) long getId();
		__declspec(dllexport) void setId( const long& id);

		__declspec(dllexport) string getName();
		__declspec(dllexport) void setName( const string& name);

		__declspec(dllexport) string getType();
		__declspec(dllexport) void setType( const string& type);

		__declspec(dllexport) vector<long> getConnections();
		__declspec(dllexport) void setConnections( const vector<long>& connections);

		__declspec(dllexport) vector<long> getParameters();
		__declspec(dllexport) void setParameters( const vector<long>& parameters);

	};
}