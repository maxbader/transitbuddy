#pragma once

#include <string>
#include <vector>

using namespace std;

namespace mped {
	class BasinConnection {

	private:
		long id;
		long source;
		long sink;
		vector<long> parameters;

	public:
		__declspec(dllexport) BasinConnection();
		__declspec(dllexport) ~BasinConnection(void);
		
		__declspec(dllexport) long getId();
		__declspec(dllexport) void setId( const long& id);

		__declspec(dllexport) long getSource();
		__declspec(dllexport) void setSource( const long& source);

		__declspec(dllexport) long getSink();
		__declspec(dllexport) void setSink( const long& sink);

		__declspec(dllexport) vector<long> getParameters();
		__declspec(dllexport) void setParameters( const vector<long>& parameters);

	};
}