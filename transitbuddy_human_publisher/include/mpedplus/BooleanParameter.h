#pragma once

#include <string>
#include <vector>

using namespace std;

namespace mped {
	class BooleanParameter : public Parameter {

	private:
		bool booleanValue;

	public:
		__declspec(dllexport) BooleanParameter();
		__declspec(dllexport) virtual ~BooleanParameter(void);

		__declspec(dllexport) bool getValue();
		__declspec(dllexport) void setValue(const bool value);

		__declspec(dllexport) virtual void setStringValue(const string &value);

	};
}