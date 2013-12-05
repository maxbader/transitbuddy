#pragma once

#include <string>
#include <vector>

using namespace std;

namespace mped {
	class LongParameter : public Parameter {

	private:
		long min;
		long max;
		long longValue;

	public:
		__declspec(dllexport) LongParameter();
		__declspec(dllexport) virtual ~LongParameter(void);

		__declspec(dllexport) long getMin();
		__declspec(dllexport) void setMin( const long min);

		__declspec(dllexport) long getMax();
		__declspec(dllexport) void setMax( const long max);

		__declspec(dllexport) long getValue();
		__declspec(dllexport) void setValue( const long value);

		__declspec(dllexport) virtual void setStringValue(const string& value);
	};
}