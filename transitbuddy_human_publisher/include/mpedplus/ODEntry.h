#pragma once

#include <string>
#include <vector>

using namespace std;

namespace mped {
	class ODEntry {

	private:
		long id;
		long origin;
		long destination;
		double frequency;
		vector<long> parameters;

	public:
		__declspec(dllexport) ODEntry();
		__declspec(dllexport) ~ODEntry(void);
		
		__declspec(dllexport) long getId();
		__declspec(dllexport) void setId( const long& id);

		__declspec(dllexport) long getOrigin();
		__declspec(dllexport) void setOrigin( const long& origin);

		__declspec(dllexport) long getDestination();
		__declspec(dllexport) void setDestination( const long& destination);

		__declspec(dllexport) double getFrequency();
		__declspec(dllexport) void setFrequency( const double& frequency);

		__declspec(dllexport) vector<long> getParameters();
		__declspec(dllexport) void setParameters( const vector<long>& parameters);

	};
}