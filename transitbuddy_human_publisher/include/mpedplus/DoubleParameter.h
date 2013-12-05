#pragma once

#include <string>
#include <vector>

using namespace std;

namespace mped {
	class DoubleParameter : public Parameter {

	private:
		double minInclusive;
		double minExclusive;
		double maxInclusive;
		double maxExclusive;
		double doubleValue;

	public:
		__declspec(dllexport) DoubleParameter();
		__declspec(dllexport) virtual ~DoubleParameter(void);

		__declspec(dllexport) double getMinInclusive();
		__declspec(dllexport) void setMinInclusive( const double minInclusive);

		__declspec(dllexport) double getMinExclusive();
		__declspec(dllexport) void setMinExclusive( const double minExclusive);

		__declspec(dllexport) double getMaxInclusive();
		__declspec(dllexport) void setMaxInclusive( const double maxInclusive);

		__declspec(dllexport) double getMaxExclusive();
		__declspec(dllexport) void setMaxExclusive( const double maxExclusive);

		__declspec(dllexport) double getValue();
		__declspec(dllexport) void setValue( const double value);

		__declspec(dllexport) virtual void setStringValue(const string& value);

	};
}