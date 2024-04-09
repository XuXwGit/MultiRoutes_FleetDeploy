#include "input_data.h"

namespace fleetdeployment
{

	Rs::Rs(std::vector<ship_route>& routeset)
		:route_set(routeset)
	{
	}

	Rs::~Rs()
	{
	}

	const ship_route& Rs::GetRoute(int route_index) const
	{
		try
		{
			for (auto& x : route_set)
			{
				if (x.GetRouteID() == route_index)
				{
					return x;
				}
			}
		}
		catch (std::exception&)
		{
			std::cerr << "no ship route" << route_index << std::endl;
		}

		throw;
	}

	Data::Data(string& path)
	{
		inputData(path);
	}

	Data::Data()
	{
	}

	Data::~Data()
	{
	}

	bool Data::inputData(const string& path)
	{
		try
		{
			inputRouteData(path);
			inputPortData(path);
			inputOdData(path);
			/*intputShipTypes(path, S, R);*/
			return true;
		}
		catch (std::exception&)
		{
			return false;
		}
	}

	map<string, Port>& Data::readPortFile(const string& path)
	{
		std::string filename = path + "ports.txt";
		ifstream fin(filename);

		if (!fin.is_open())
		{
			std::cout << "can't open file: " << filename << std::endl;
		}

		std::string first_line;
		getline(fin, first_line);
		std::cout << first_line << std::endl;

		std::string line;
		//PortID	Port	WhetherTrans	Region	Group
		//1	A1	1	A	1
		std::regex pattern(R"((\d+)\t(.+)\t(\d+)\t(.+)\t(\d+))");

		while (std::getline(fin, line)) {
			std::smatch match;
			if (std::regex_match(line, match, pattern)) {
				int portID = std::stoi(match[1]);
				std::string port = match[2];
				int whetherTrans = std::stoi(match[3]);
				std::string region = match[4];
				int group = std::stoi(match[5]);

				Port tempPort;
				tempPort.SetPortID(portID);
				tempPort.SetPort(port);
				tempPort.SetRegion(region);

				P.emplace(std::make_pair(port, tempPort));
			}
		}

		return P;
	}


	map<string, Port>& Data::inputPortData(const string& path)
	{
		std::filesystem::path filepath = std::filesystem::path(path) / "ports.txt";
		if (!std::filesystem::exists(filepath)) {
			P = generate_ports(path);
		}
		else {
			P = readPortFile(path);
		}

		//output ports information
		std::cout << "PortID\t" << "Port\t"  << "Region" << std::endl;
		std::vector <pair<string, Port >> ports(P.begin(), P.end());
		std::sort(ports.begin(), ports.end(), compare);
		for (auto& x : ports)
		{
			std::cout << x.second.GetPortID() << '\t' << x.second.GetPort() << '\t' << x.second.GetRegion() << std::endl;
		}

		count_next_and_front_port();

		return P;
	}

	std::vector<ship_route>& Data::inputRouteData(const string& path)
	{
		std::filesystem::path filepath = std::filesystem::path(path) / "routes.txt";
		if (!std::filesystem::exists(filepath)) {
			read_ship_route(path);
		}

		std::string filename = path + "routes.txt";
		ifstream fin(filename);
		if (!fin.is_open())
		{
			std::cout << "can't open routes file" << " from "<< filename << std::endl;
		}

		std::string first_line;
		getline(fin, first_line);
		std::cout << first_line << std::endl;

		std::string line;
		std::vector<route_node> tempPath;
		int tempID = 0;

		std::regex pattern(R"((\d+)\s+(\d+)\s+(.+)\s+(\d+))");
		while (std::getline(fin, line)) {
			std::smatch match;
			if (std::regex_match(line, match, pattern)) {
				int routeID = std::stoi(match[1]);
				int call = std::stoi(match[2]);
				std::string port = match[3];
				int arrivalTime = std::stoi(match[4]);

				route_node tempNode(call, port, arrivalTime);

				if (tempID != 0) {
					if (routeID != tempID) {
						R.push_back(ship_route(tempID, tempPath));
						tempPath.clear();
						tempID = routeID;
						tempPath.push_back(tempNode);
					}
					else {
						tempPath.push_back(tempNode);
					}
				}
				else {
					tempID = routeID;
					tempPath.push_back(tempNode);
				}
			}
		}
		// Add the last route
		if (!tempPath.empty()) {
			R.push_back(ship_route(tempID, tempPath));
		}

		//output ship routes
		std::cout << "output ship routes" << std::endl;
		for (auto& x : R)
		{
			std::cout << x.GetRouteID() << " : ";
			for (auto& y : x.GetPath())
			{
				std::cout << y.port << "(" << y.call << ", " << y.arrivalTime << ") ";
			}
			std::cout << std::endl;
		}

		return R;
	}

	std::vector<OD>& Data::inputOdData(const string& path)
	{
		std::filesystem::path filepath = std::filesystem::path(path) / "request.txt";
		if (!std::filesystem::exists(filepath)) {
			std::cout << "Generate ods and output to file" << std::endl;
			W = generate_request(path);
		}
		else {
			string filename = path + "request.txt";
			ifstream fin(filename);
			if (!fin.is_open())
			{
				std::cout << "can't open file : " << filename << std::endl;
			}
			else
			{
				std::string first_line;
				getline(fin, first_line);
				std::cout << first_line << std::endl;

				std::string line;

				std::regex pattern(R"((\d+)\t([^\t]+)\t([^\t]+)\t(\d+)\t(\d+))");

				while (std::getline(fin, line)) {
					std::smatch match;
					if (std::regex_match(line, match, pattern)) {
						int RequestID = std::stoi(match[1]);
						std::string origin = match[2];
						std::string destination = match[3];
						int earlist_pickupara_time = std::stoi(match[4]);
						int deadline_time = std::stoi(match[5]);

						OD od;
						od.odID = RequestID;
						od.originPort = origin;
						od.destinationPort = destination;
						od.earliest_pickup_time = earlist_pickupara_time;
						od.deadline_time = deadline_time;

						W.push_back(od);
					}
				}
			}
		}

		std::cout << "Print the od / request data" << std::endl;
		for (auto& x : W)
		{
			std::cout << x.odID<< '\t' << x.originPort << '\t' << x.destinationPort << '\t' << x.earliest_pickup_time << '\t' << x.deadline_time << std::endl;
		}

		return W;
	}

	std::vector<OD>& Data::input_demand_data(const string& path)
	{
		return W;
	}


	map<string, Port>& Data::generate_ports(const string& path) {
		std::string filename = path + "ports.txt";

		ofstream fout(filename);
		if (!fout.is_open())
		{
			std::cout << "can't open file : " << filename << std::endl;
		}
		else {
			int count_port = int(P.size());
			
			for (size_t r = 0; r < R.size(); r++)
			{
				for (size_t p = 0; p < R[r].GetPath().size(); p++)
				{
					string port = R[r].GetPath()[p].port;
					if (P.find(port) == P.end()) {
						count_port++;
						P.emplace(make_pair(port, Port(count_port, port)));
					}
				}
			}

			fout << "PortID\t" << "Port" << std::endl;
			std::vector <pair<string,  Port >> ports(P.begin(), P.end());
			std::sort(ports.begin(), ports.end(), compare);
			for (auto& x : ports)
			{
				fout << x.second.GetPortID() << '\t' << x.second.GetPort() << std::endl;
			}
		}

		return P;
	}


	std::vector<OD>& Data::generate_request(const string& path) {
		OD temp_od;
		int count = 0;
		for (auto& x : P)
		{
			for (auto& y : P)
			{
				if (x.first != y.first) {
					temp_od.odID = (++count);
					temp_od.originPort = x.first;
					temp_od.destinationPort = y.first;
					temp_od.earliest_pickup_time = 1;
					temp_od.deadline_time = 180;
					W.push_back(temp_od);
				}
			}
		}

		ofstream fout(path + "request.txt");
		fout << "PortID\tOriginPort\tDestinationPort\tEarlistTime\tDeadline" << std::endl;
		for (auto& x : W)
		{
			fout << x.odID << '\t'
				<< x.originPort << '\t'
				<< x.destinationPort << '\t'
				<< x.earliest_pickup_time << '\t'
				<< x.deadline_time << std::endl;
		}

		return W;
	}



	std::vector<OD>& Data::generate_demands(std::vector<string>& origin_ports, vector<string>& destination_ports, int min_demand, int max_demand, vector<OD>& demands)
	{
		OD temp_od;
		for (auto& x : origin_ports)
		{
			for (auto& y : destination_ports)
			{
				temp_od.originPort = x;
				temp_od.destinationPort = y;
				temp_od.normal_demand = min_demand;
				temp_od.varable_demand = max_demand - min_demand;

				demands.push_back(temp_od);
			}
		}

		return demands;
	}



	std::vector<Ship>& Data::intputShipTypes(const string& path)
	{
		ifstream fin(path + "ships.txt");
		if (!fin.is_open())
		{
			std::cout << "can't open ods file" << std::endl;
		}

		std::string first_line;
		getline(fin, first_line);
		std::cout << first_line << std::endl;

		std::vector<int> capacity;
		std::vector<double> chart_out;
		std::vector<double> chart_in;
		std::vector<int> max_num;
		std::vector<int> max_num_chart_in;
		std::string lineData;
		int order = 0;
		// read capacity data
		getline(fin, lineData);
		while (!lineData.empty())
		{
			auto loc = find(lineData.begin(), lineData.end(), '\t');
			order = (int)distance(lineData.begin(), loc);
			capacity.push_back(stoi(lineData.substr(0, order)));
			lineData.erase(0, order + 1);
		}
		// read chart_out data
		getline(fin, lineData);
		while (!lineData.empty())
		{
			auto loc = find(lineData.begin(), lineData.end(), '\t');
			order = (int)distance(lineData.begin(), loc);
			chart_out.push_back(stoi(lineData.substr(0, order)));
			lineData.erase(0, order + 1);
		}
		// read chart_in data
		getline(fin, lineData);
		while (!lineData.empty())
		{
			auto loc = find(lineData.begin(), lineData.end(), '\t');
			order = (int)distance(lineData.begin(), loc);
			chart_in.push_back(stoi(lineData.substr(0, order)));
			lineData.erase(0, order + 1);
		}
		// read num data
		getline(fin, lineData);
		while (!lineData.empty())
		{
			auto loc = find(lineData.begin(), lineData.end(), '\t');
			order = (int)distance(lineData.begin(), loc);
			max_num.push_back(stoi(lineData.substr(0, order)));
			lineData.erase(0, order + 1);
		}
		// read max_num_chart_in data
		getline(fin, lineData);
		while (!lineData.empty())
		{
			auto loc = find(lineData.begin(), lineData.end(), '\t');
			order = (int)distance(lineData.begin(), loc);
			max_num_chart_in.push_back(stoi(lineData.substr(0, order)));
			lineData.erase(0, order + 1);
		}
		for (size_t i = 1; i < capacity.size(); i++)
		{
			struct Ship tempShip;
			tempShip.TEUs = capacity[i];
			tempShip.chart_out = chart_out[i];
			tempShip.chart_in = chart_in[i];
			tempShip.max_num = max_num[i];
			tempShip.max_chart_in = max_num_chart_in[i];
			S.push_back(tempShip);
		}

		// read the cost data
		while (getline(fin, lineData))
		{
			int route_index = 0;
			int index = 0;
			double cost = 0.0;
			std::vector<pair<int, double>> shipTypes;
			while (!lineData.empty())
			{
				auto loc = find(lineData.begin(), lineData.end(), '\t');
				order = (int)distance(lineData.begin(), loc);
				if (index == 0)
				{
					route_index = stoi(lineData.substr(0, order));
				}
				else
				{
					cost = stof(lineData.substr(0, order));
					if (cost != 0)
						shipTypes.push_back(pair<int, double>(capacity[index], cost));
				}
				lineData.erase(0, order + 1);
				++index;
			}
			R[route_index - 1].SetShipTypes(shipTypes);
		}

		//for (auto x : S)
		//{
		//	std::cout << x.TEUs << " " << x.chart_out << " " << x.chart_in << " " << x.max_num << " " << x.max_chart_in << std::endl;
		//}
		for (size_t i = 0; i < R.size(); i++)
		{
			std::cout << "ship route " << i << " : " << std::endl;
			for (auto& x : R[i].GetShipTypes())
			{
				std::cout << x.first << "(" << x.second << ")" << " ";
			}
			std::cout << std::endl;
		}

		return S;
	}



	void Data::count_next_and_front_port()
	{
		for (size_t r = 0; r < R.size(); r++)
		{
			std::string origin, destination;
			for (size_t p = 0; p < R[r].GetPath().size() - 1; p++)
			{
				origin = R[r].GetPath()[p].port;
				destination = R[r].GetPath()[p + 1].port;

				P[origin].AddPort2NextPortSet(destination);
				P[destination].AddPort2FrontPortSet(origin);
			}
		}

		for (auto& x : P)
		{
			x.second.SetWhetherTrans((x.second.GetNextPortSet().size() != 1
				|| x.second.GetFrontPortSet().size() != 1));
		}
	}




	void Data::read_ship_route(const string& path)
	{
		std::string filename = path + "ship route.txt";

		std::vector<ship_route> R;

		ifstream fin(filename);
		if (!fin.is_open()) {
			std::cout << "Can't open file: " << filename << std::endl;
		}

		std::string stringline;
		int routeID = 0;
		std::string s_node;
		while (!fin.eof())
		{
			std::vector<route_node> tempPath;

			getline(fin, stringline);
			std::cout << stringline << std::endl;

			int loc = (int)stringline.find('\t');
			routeID = stoi(stringline.substr(0, loc));
			stringline.erase(0, loc + 1);

			while (!stringline.empty())
			{

				route_node temp_node;

				int loc1 = (int)stringline.find("(");
				temp_node.port = stringline.substr(0, loc1);
				stringline.erase(0, loc1 + 1);

				int loc2 = (int)stringline.find("-");
				temp_node.arrivalTime = std::stoi(stringline.substr(0, loc2)) / 24;
				stringline.erase(0, loc2 + 1);

				int loc3 = (int)stringline.find(")");
				temp_node.departureTime = std::stoi(stringline.substr(0, loc3)) / 24;

				tempPath.push_back(temp_node);

				if (stringline.size() == 1) break;

				stringline.erase(0, loc3 + 3);
			}

			R.push_back(ship_route(routeID, tempPath));
		}

		for (auto& shipara_route : R) {
			std::cout << "Ship Route ID: " << shipara_route.GetRouteID() << std::endl;

			for (auto& Path : shipara_route.GetPath()) {
				std::cout << "Port: " << Path.port << std::endl;
				std::cout << "Arrival Time: " << Path.arrivalTime << std::endl;
				std::cout << "Departure Time: " << Path.departureTime << std::endl;
				std::cout << std::endl;
			}
		}

		this->SetR(R);

		output_shipping_route(path);
		output_routes(path);
	}

	void Data::output_shipping_route(const string& path) {
		std::string filename = path + "Shipingroute.txt";

		ofstream fout(filename);
		if (!fout.is_open())
		{
			std::cout << "can't open file : " << filename << std::endl;
		}
		else
		{
			std::vector<ship_route> R = this->GetR();
			fout << "RouteID\t"
				<< "NumPorts\t"
				<< "Ports\t"
				<< "NumCall\t"
				<< "PortsofCall\t"
				<< "ArrivalTime" << std::endl;
			for (size_t i = 0; i < R.size(); i++)
			{
				fout << R[i].GetRouteID() << '\t'
					<< R[i].GetNumPort() << '\t';

				// output the port set
				for (size_t p = 0; p < R[i].GetNumPort(); p++)
				{
					if (p) {
						fout << ",";
					}
					fout << R[i].GetPort(int(p));
				}
				fout << "\t";

				fout << R[i].GetNumPort() + 1 << "\t";

				// output the port of call sequence
				for (size_t p = 0; p < R[i].GetPath().size(); p++)
				{
					fout << R[i].GetPath()[p].port;
					if (p != R[i].GetPath().size() - 1) {
						fout << ",";
					}
				}
				fout << '\t';

				// output the arrival time sequence
				for (size_t p = 0; p < R[i].GetPath().size(); p++)
				{
					fout << R[i].GetPath()[p].arrivalTime;
					if (p != R[i].GetPath().size() - 1) {
						fout << ",";
					}
				}

				fout << std::endl;
			}
		}
	}

	void Data::output_routes(const string& path)
	{
		std::string filename = path + "routes.txt";

		ofstream fout(filename);
		if (!fout.is_open())
		{
			std::cout << "can't open file : " << filename << std::endl;
		}
		else
		{
			std::vector<ship_route> R = this->GetR();
			fout << "Route\t"
				<< "Call\t"
				<< "Port\t"
				<< "Time" << std::endl;
			for (size_t i = 0; i < R.size(); i++)
			{
				for (size_t p = 0; p < R[i].GetPath().size(); p++)
				{
					fout << R[i].GetRouteID() << '\t';
					fout << p + 1 << '\t';
					fout << R[i].GetPath()[p].port << '\t';
					fout << R[i].GetPath()[p].arrivalTime;
					fout << std::endl;
				}
			}
		}

		fout.close();
	}

       
        string fleetdeployment::Rs::GetPort(int r, int i) const {
                string port;
                try {
                        for (auto x : route_set) {
                                if (x.GetID() == r) {
                                        port = x.GetPath()[i - 1].port;
                                        return port;
                                }
                        }
                } catch (const std::exception&) {
                        cout << "no port (r, i)" << endl;
                }
				return port;
        }

        bool inputData(string& path, map<string, Port>& P,
                       vector<ship_route>& R, vector<OD>& W) {
                try {
                        inputPortData(path, P);
                        inputRouteData(path, R);
                        inputODData(path, W, P);
                        return true;
                } catch (const std::exception&) {
                        return false;
                }
        }

        map<string, Port>& inputPortData(string& path, map<string, Port>& P) {
                ifstream fin(path + "/ports.txt");
                if (!fin.is_open()) {
                        cout << "can't open ports file" << endl;
                }
                double loadcost, discharge, transshopment;
                string port;
                while (!fin.eof()) {
                        fin >> port >> loadcost >> discharge >> transshopment;
                        Port tempPort(port, loadcost, discharge,
                                                transshopment);
                        P.emplace(make_pair(port, tempPort));
                }

                // output ports information
                for (auto x : P) {
                        cout << x.first << ": " << x.second.GetPort() << " "
                             << x.second.GetLoadingCost() << " "
                             << x.second.GetDischargeCost() << " "
                             << x.second.GetTransshipmentCost() << endl;
                }

                return P;
        }

        vector<ship_route>& inputRouteData(string& path,
                                           vector<ship_route>& R) {
                ifstream fin(path + "/ship route.txt");
                if (!fin.is_open()) {
                        cout << "can't open ship routes file" << endl;
                }
                int routeID, portID, arrivalTime;
                string port;
                int tempID = 0;
                vector<route_node> tempPath;
                while (!fin.eof()) {
                        fin >> routeID >> portID >> port >> arrivalTime;
                        struct route_node tempNode = {portID, port,
                                                      arrivalTime};
                        if (tempID != 0) {
                                if (routeID != tempID) {
                                        R.push_back(
                                            ship_route(tempID, tempPath));
                                        tempPath.clear();
                                        tempID = routeID;
                                        tempPath.push_back(tempNode);
                                } else {
                                        tempPath.push_back(tempNode);
                                }
                        } else {
                                tempID = routeID;
                                tempPath.push_back(tempNode);
                        }
                }
                tempPath.pop_back();
                R.push_back(ship_route(tempID, tempPath));

                // output ship routes
                cout << "output ship routes" << endl;
                for (auto x : R) {
                        cout << x.GetID() << " : ";
                        for (auto y : x.GetPath()) {
                                cout << y.port << "(" << y.routeID << ", "
                                     << y.arrivalTime << ") ";
                        }
                        cout << endl;
                }

                return R;
        }

        vector<OD>& inputODData(string& path, vector<OD>& W,
                                map<string, Port>& P) {
                ifstream fin(path + "/ods.txt");
                if (!fin.is_open()) {
                        cout << "can't open ods file" << endl;
                }
                int demand;
                string origin, destination;
                double freightRate, maxProfitRate;
                while (!fin.eof()) {
                        fin >> origin >> destination >> demand >> freightRate;
                        maxProfitRate = freightRate -
                                        P[origin].GetLoadingCost() -
                                        P[destination].GetDischargeCost();
                        OD od(origin, destination, demand, 
						freightRate, maxProfitRate);
                        W.push_back(od);
                }
                W.pop_back();

                cout << "output the od data" << endl;
                for (auto x : W) {
                        cout << x.GetOrigin() << " " << x.GetDestination()
                             << " " << x.GetDemand() << endl;
                }

                return W;
        }


}		// ! namespace fleetdeployment end !