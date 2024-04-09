# include "ReadData.h"


/*
* format :
1	Singapore(166-186)->Brisbane(348-368)->Sydney(386-406)->Melbourne(428-448)->AdelaIDe(468-488)->Fremantle(546-566)->Singapore(670690)
2	Xiamen(48-72)->Chiwan(84-108)->HongKong(108-132)->Singapore(194-218)->PortKlang(226-250)->Salalah(383-407)->Jeddah(463-487)->Aqabah(511-535)->Salalah(615-639)->Singapore(780-804)->Xiamen(888-912)
3	Yokohama(100-144)->Tokyo(144-188)->Nagoya(197-241)->Kobe(251-295)->Shanghai(333-377)->Yokohama(436-480)
4	HoChiMinh(35-87)->LaemChabang(118-170)->Singapore(209-261)->PortKlang(270-322)->HoChiMinh(371-423)
5	Brisbane(141-172)->Sydney(194-225)->Melbourne(252-283)->AdelaIDe(307-338)->Fremantle(408-439)->Jakarta(530-561)->Singapore(586-617)->Brisbane(813-844)
6	Manila(101-136)->Kaohsiung(163-198)->Xiamen(206-241)->HongKong(254-289)->Yantian(289-324)->Chiwan(325-360)->HongKong(360-395)->Manila(437-472)
7	Dalian(56-84)->Xingang(93-121)->Qingdao(141-169)->Shanghai(185-213)->Ningbo(217-245)->Shanghai(249-277)->Kwangyang(297-325)->Busan(328-356)->Dalian(392-420)
8	Chittagong(137-167)->Chennai(213-243)->Colombo(273-303)->Cochin(319-349)->NhavaSheva(387-417)->Cochin(455-485)->Colombo(501-531)->Chennai(561-591)->Chittagong(641-671)
9	Sokhna(155-186)->Aqabah(197-228)->Jeddah(252-283)->Salalah(339-370)->Karachi(409-440)->JebelAli(470-501)->Salalah(539-570)->Sokhna(659-690)
10	Southampton(161-176)->Thamesport(182-197)->Hamburg(212-227)->Bremerhaven(230-245)->Rotterdam(252-267)->Antwerp(268-283)Zeebrugge(285-300)->LeHavre(306-321)->Southampton(329-344)
11	Southampton(55-71)->Sokhna(197-213)->Salalah(288-304)->Colombo(369-385)->Singapore(447-463)->HongKong(519-535)->Xiamen(545-561)->Shanghai(580-596)->Busan(613-629)->Dalian(648-664)->Xingang(671-687)->Qingdao(702-718)->Shanghai(730-746)->HongKong(775-791)->Singapore(847-863)->Colombo(925-941)->Salalah(1006-1022)->Southampton(1231-1247)
*/

namespace fleetdeployment {

	Data& ReadData::read_ship_route(const string& path, Data& data)
	{
		std::string filename = path + "ship route.txt";

		std::vector<ship_route> R;

		ifstream fin(filename);
		if (!fin.is_open()) {
			std::cout << "Can't open " << filename << std::endl;
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

		data.SetR(R);

		_output_input_data(path, data);

		return data;
	}

	void ReadData::_output_input_data(string path, Data& data) {
		output_ship_route(path, data);
	}

	void ReadData::output_ship_route(string path, Data& data) {
		std::string filename = path + "shipingroute.txt";

		ofstream fout(filename);
		if (!fout.is_open())
		{
			std::cout << "can't open file : " << filename << std::endl;
		}
		else
		{
			std::vector<ship_route> R = data.GetR();
			fout << "RouteID\t"
				<<"NumPorts\t"
				<<"Ports\t"
				<<"NumCall\t"
				<<"PortsofCall\t"
				<<"ArrivalTime" << std::endl;
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
				fout <<  '\t';

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


	std::vector<std::vector<Node>> ReadData::read_sequence(string filename/*, vector<std::vector<Node>> sequence_vector*/)
	{
		std::vector<std::vector<Node>> temp;

		ifstream fin(filename);
		std::string stringline;
		int order = 0;
		std::string s_node;
		std::vector<Node> sequence;
		while (!fin.eof())
		{
			getline(fin, stringline);
			std::cout << stringline << std::endl;
			sequence.clear();
			while (!stringline.empty())
			{
				auto loc = find(stringline.begin(), stringline.end(), '/');
				if (loc == stringline.end())
					loc = stringline.end();
				s_node = string(stringline.begin(), loc);
				Node node = string_to_Node(s_node);
				sequence.push_back(node);
				if (loc != stringline.end())
					stringline.erase(stringline.begin(), loc + 1);
				else
					stringline.erase(stringline.begin(), loc);
			}
			sort(sequence.begin(), sequence.end());
			if (!sequence.empty())
				temp.push_back(sequence);
		}
		//for (size_t i = 0; i < temp.size(); i++)
		//{
		//	sequence_vector.push_back(temp[i]);
		//}

		//return sequence_vector;

		return temp;
	}

	std::vector<ContainerPath>& ReadData::read_paths(const string& filename, vector<ContainerPath>& Ps)
	{
		ifstream fin(filename);
		std::string stringline;
		getline(fin, stringline);
		std::cout << stringline << std::endl;
		getline(fin, stringline);
		std::cout << stringline << std::endl;
		getline(fin, stringline);
		std::cout << stringline << std::endl;
		while (!fin.eof())
		{
			int temp_PathID, temp_origin_Time, temp_Destination_Time, temp_PathTime, temp_PortPath_length, temp_Arcs_length;
			std::string temp_OriginPort, temp_Destination_Port, temp_TransshipPort, temp_TransshipTime, temp_PortPath, temp_ArcIDs;
			fin >> temp_PathID
				>> temp_OriginPort
				>> temp_origin_Time
				>> temp_Destination_Port
				>> temp_Destination_Time
				>> temp_PathTime
				>> temp_TransshipPort
				>> temp_TransshipTime
				>> temp_PortPath_length
				>> temp_PortPath
				>> temp_Arcs_length
				>> temp_ArcIDs;
			ContainerPath tempPath;
			tempPath.SetPathID(temp_PathID);
			tempPath.SetOriginPort(temp_OriginPort);
			tempPath.SetOriginTime(temp_origin_Time);
			tempPath.SetDestinationPort(temp_Destination_Port);
			tempPath.SetOriginTime(temp_origin_Time);
			tempPath.SetDestinationTime(temp_Destination_Time);
			tempPath.SetPathTime(temp_PathTime);
			tempPath.SetNumArcs(temp_Arcs_length);

			if (tempPath.GetOriginPort().empty())
				break;

			// print new path
			std::cout << temp_PathID << '\t'
				<< temp_OriginPort << '\t'
				<< temp_origin_Time << '\t'
				<< temp_Destination_Port << '\t'
				<< temp_Destination_Time << '\t'
				<< temp_PathTime << '\t'
				<< temp_TransshipPort << '\t'
				<< temp_TransshipTime << '\t'
				<< temp_PortPath_length << '\t'
				<< temp_PortPath << '\t'
				<< temp_Arcs_length << '\t'
				<< temp_ArcIDs << std::endl;

			while (!temp_TransshipPort.empty())
			{
				auto loc1 = find(temp_TransshipPort.begin(), temp_TransshipPort.end(), ',');
				auto loc2 = find(temp_TransshipTime.begin(), temp_TransshipTime.end(), ',');
				tempPath.AddPort2TransshipPorts(string(temp_TransshipPort.begin(), loc1));
				tempPath.AddTime2TransshipTime(stoi(string(temp_TransshipTime.begin(), loc2)));

				if (loc1 == temp_TransshipPort.end())
				{
					temp_TransshipPort.erase(temp_TransshipPort.begin(), loc1);
					temp_TransshipTime.erase(temp_TransshipTime.begin(), loc2);
				}
				else
				{
					temp_TransshipPort.erase(temp_TransshipPort.begin(), loc1 + 1);
					temp_TransshipTime.erase(temp_TransshipTime.begin(), loc2 + 1);
				}
			}

			while (!temp_PortPath.empty())
			{
				auto loc = find(temp_PortPath.begin(), temp_PortPath.end(), ',');
				tempPath.AddPort2PathPorts(string(temp_PortPath.begin(), loc));

				if (loc == temp_PortPath.end())
				{
					temp_PortPath.erase(temp_PortPath.begin(), loc);
				}
				else
				{
					temp_PortPath.erase(temp_PortPath.begin(), loc + 1);
				}
			}

			while (!temp_ArcIDs.empty())
			{
				auto loc = find(temp_ArcIDs.begin(), temp_ArcIDs.end(), ',');
				tempPath.AddArcID2ArcIDs(stoi(string(temp_ArcIDs.begin(), loc)));

				if (loc == temp_ArcIDs.end())
				{
					temp_ArcIDs.erase(temp_ArcIDs.begin(), loc);
				}
				else
				{
					temp_ArcIDs.erase(temp_ArcIDs.begin(), loc + 1);
				}
			}
			// update

			Ps.push_back(tempPath);

			tempPath.printPath();
		}
		// delete the last path (because the last row is '\d')
		/*Ps.pop_back();*/
		return Ps;
	}

	Node ReadData::string_to_Node(string& node_string)
	{
		Node node;
		std::string s_nodeID, s_routeID, s_call, s_rotation, s_arrivaltime;
		int order = 0;

		// Get the node ID
		auto loc = find(node_string.begin(), node_string.end(), '(');
		s_nodeID = node_string.substr(0, (int)distance(node_string.begin(), loc));
		node.SetNodeID(stoi(s_nodeID));
		order = (int)distance(node_string.begin(), loc);
		node_string.erase(0, order + 1);

		// Get the port
		loc = find(node_string.begin(), node_string.end(), ',');
		node.SetPortString(string(node_string.begin(), loc));
		//node_string.erase(node_string.begin(), loc + 1);
		order = (int)distance(node_string.begin(), loc);
		node_string.erase(0, order + 1);

		// Get node route
		loc = find(node_string.begin(), node_string.end(), ',');
		s_routeID = node_string.substr(0, distance(node_string.begin(), loc));
		node.SetRouteID(stoi(s_routeID));
		//node_string.erase(node_string.begin(), loc + 1);
		order = (int)distance(node_string.begin(), loc);
		node_string.erase(0, order + 1);

		// Get the call
		loc = find(node_string.begin(), node_string.end(), ',');
		s_call = node_string.substr(0, distance(node_string.begin(), loc));
		node.SetCall(stoi(s_call));
		//node_string.erase(node_string.begin(), loc + 1);
		order = (int)distance(node_string.begin(), loc);
		node_string.erase(0, order + 1);

		// Get the rotation
		loc = find(node_string.begin(), node_string.end(), ',');
		s_rotation = node_string.substr(0, distance(node_string.begin(), loc));
		node.SetRotation(stoi(s_rotation));
		//node_string.erase(node_string.begin(), loc + 1);
		order = (int)distance(node_string.begin(), loc);
		node_string.erase(0, order + 1);

		// Get the arrival time
		loc = find(node_string.begin(), node_string.end(), ')');
		s_arrivaltime = node_string.substr(0, distance(node_string.begin(), loc));
		node.SetTime(stoi(s_arrivaltime));

		return node;
	}

	ContainerPath ReadData::sequence_to_path(std::vector<Node>& sequence)
	{
		return ContainerPath();
	}

	std::vector<ContainerPath>& ReadData::sequences_to_Paths(ST_network& ST, vector<std::vector<Node>>& sequence_vector, vector<ContainerPath>& Ps)
	{
		int path_count = (int)Ps.size();
		for (size_t i = 0; i < sequence_vector.size(); i++)
		{
			ContainerPath tempPath;
			tempPath.SetPathID(++path_count);
			for (size_t j = 0; j < sequence_vector[i].size() - 1; j++)
			{
				Node origin = sequence_vector[i][j];
				Node destination = sequence_vector[i][j + 1];
				tempPath.AddArcID2ArcIDs(ST.GetArcID(origin, destination));
				if (j == 0)
				{
					tempPath.AddPort2PathPorts(origin.GetPort());
				}
				if (origin.GetPort() == destination.GetPort())
				{
					tempPath.AddPort2TransshipPorts(destination.GetPort());
					tempPath.AddTime2TransshipTime(destination.GetTime() - origin.GetTime());
				}
				else
				{
					tempPath.AddPort2PathPorts(destination.GetPort());
				}
			}
			tempPath.SetNumArcs((int)tempPath.GetArcIDs().size());
			tempPath.SetOriginPort(sequence_vector[i].front().GetPort());
			tempPath.SetDestinationPort(sequence_vector[i].back().GetPort());
			tempPath.SetOriginTime(sequence_vector[i].front().GetTime());
			tempPath.SetDestinationTime(sequence_vector[i].back().GetTime());
			tempPath.SetPathTime(tempPath.GetDestinationTime() - tempPath.GetOriginTime());

			Ps.push_back(tempPath);
		}

		return Ps;
	}



}