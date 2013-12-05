#pragma once

#include <string>
#include <vector>

using namespace std;

namespace mped {
	class StringParameter : public Parameter {

	private:
		string pattern;
		long minLength;
		long maxLength;

	public:
		__declspec(dllexport) StringParameter();
		__declspec(dllexport) virtual ~StringParameter(void);

		__declspec(dllexport) string getPattern();
		__declspec(dllexport) void setPattern( const string& pattern);

		__declspec(dllexport) long getMinLength();
		__declspec(dllexport) void setMinLength( const long& minLength);

		__declspec(dllexport) long getMaxLength();
		__declspec(dllexport) void setMaxLength( const long& maxLength);

		__declspec(dllexport) string getValue();
		__declspec(dllexport) void setValue( const string& value);

	};
}