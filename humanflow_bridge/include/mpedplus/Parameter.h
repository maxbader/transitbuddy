#pragma once

#include <string>
#include <vector>

using namespace std;

namespace mped {
	class Parameter {

	private:
		long id;
		string name;
		string type;
		string stringValue;

	public:
		__declspec(dllexport) Parameter();
		__declspec(dllexport) virtual ~Parameter(void);
		
		__declspec(dllexport) long getId();
		__declspec(dllexport) void setId( const long id);

		__declspec(dllexport) string getName();
		__declspec(dllexport) void setName( const string& name);

		__declspec(dllexport) string getType();
		__declspec(dllexport) void setType( const string& type);

		__declspec(dllexport) virtual string getStringValue();
		__declspec(dllexport) virtual void setStringValue(const string& value);

	};
}