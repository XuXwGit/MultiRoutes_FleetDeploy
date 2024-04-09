#ifndef _INPUT_DATA_H_
#define _INPUT_DATA_H_

#include <string>
#include <vector>
#include <set>
#include <map>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <algorithm>
#include <sstream>
#include <regex>

#include "ContainerPath.h"
#include "OD.h"
#include "Port.h"
#include "ship_route.h"

using namespace std;

namespace fleetdeployment
{

	struct Ship
	{
		int TEUs;
		double chart_in;
		double chart_out;

		int max_num;
		int max_chart_in;
	};

	class  Rs
	{
	public:
		Rs( vector<ship_route>& routes);
		~Rs();

		const ship_route& GetRoute(int route_index) const;
                ship_route& GetRoute(int shipRouteID);
		inline const std::vector<ship_route>& GetRouteSet() const{
			return route_set;
		}
       inline vector<ship_route>& GetRoutes() { return route_set; }
	   std::string GetPort(int r, int i) const;

	private:
		std::vector<ship_route> route_set;
	};

	class  Data
	{
	private:
		map<string, Port>	P;
		std::vector<ship_route> R;
		std::vector<OD>			W;
		std::vector<Ship>			S;

	public:
		Data(string& path);
		Data();
		~Data();

	public:
		// input data(P, R, W)
		// P: the Set of ports,  |P| : P.size() : the number of port
		// R: the Set of ship routes
		// W: the Set of ods
		bool inputData(const string& path);

		map<string, Port>& readPortFile(const string& path);
		map<string, Port>& inputPortData(const string& path);

		std::vector<ship_route>& inputRouteData(const string& path);

		std::vector<OD>& inputOdData(const string& path);

		std::vector<OD>& input_demand_data(const string& path);
		map<string, Port>& generate_ports(const string& path);
		std::vector<OD>& generate_request(const string& path);

		std::vector<OD>& generate_demands(std::vector<string>& origin_ports, vector<string>& destination_ports, int min_demand, int max_demand, vector<OD>& demands);

		std::vector<Ship>& intputShipTypes(const string& path);

		inline const map<string, Port>&	GetP() const
		{
			return P;
		}
		inline const vector<ship_route>& GetR() const
		{
			return R;
		}
		inline const vector<OD>&				GetW() const
		{
			return W;
		}
		inline const vector<Ship>&			GetS() const
		{
			return S;
		}
		inline void SetR(const vector<ship_route>& R) { 
			this->R = R; 
		}

		//bool compare(const std::pair<std::string, Port>& left, const std::pair<std::string, Port>& right) {
		//	return left.second < right.second;
		//}

		void read_ship_route(const string& path);

		void output_shipping_route(const string& path);

		void output_routes(const string& path);

		void count_next_and_front_port();
	};

}
#endif // !_INPUT_DATA_H_
