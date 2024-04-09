#include "output_data.h"

namespace fleetdeployment {

void output_STdata(const string& path, ST_network& ST)
{
	std::string filename = path + "N=" + i_to_s(ST.GetN().size()) + ", A=" + i_to_s(ST.GetA().size()) + ".txt";
	output_space_time_network_status(filename, ST);
	filename = path + "ShipingRoutes.txt";
	output_shipingroute_data(filename, ST);
	filename = path + "VesselP-aths.txt";
	output_vesselpaths_data(filename, ST);
	filename = path + "Nodes.txt";
	output_nodes_data(filename, ST);
	filename = path + "TravelingArcs.txt";
	output_TravelArc_data(filename, ST);
	filename = path + "TransshipArcs.txt";
	output_transshipArcs_data(filename, ST);
}

void output_Path_data(const string& path, GeneratePath& GP)
{
	std::string filename = path + "P-aths.txt";
	output_paths_data(filename, GP);
	output_requests_data(path, GP);
}

void output_space_time_network_status(const string& filename, ST_network& ST) {
	ofstream fout(filename);
	if (!fout.is_open())
	{
		std::cout << "can't open shiping route file" << std::endl;
	}
	else
	{
		fout << "=========Base Status of Space-Time Network========" << std::endl;
		fout << "TimeHorizon = " << ST.GetT() << std::endl;
		fout << "ShipRoutes = " << ST.GetR().size() << std::endl;
		fout << "Ports = " << ST.GetP().size() << std::endl;		
		fout << "Nodes = " << ST.GetN().size() << std::endl;
		fout << "Arcs = " << ST.GetA().size() << std::endl;
		fout << "Traveling Arcs = " << ST.GetAv().size() << std::endl;
		fout << "Transship Arcs = " << ST.GetAt().size() << std::endl;
		fout << "VesselPaths = " << ST.GetVP().size() << std::endl;
		fout << "==========================================" << std::endl;
		fout.close();
	}
}

void output_shipingroute_data(const string& filename, ST_network& ST)
{
	ofstream fout(filename);
	if (!fout.is_open())
	{
		std::cout << "can't open shiping route file" << std::endl;
	}
	else
	{
		fout << "ShippingRouteID\tNumberofPorts\tPorts\tNumberofCall\tPortsofCall\tTime" << std::endl;
		std::vector<ship_route> R = ST.GetR();
		for (size_t i = 0; i < R.size(); i++)
		{
			fout << R[i].GetRouteID() << '\t'
				<< R[i].GetNumPort() << '\t';
			for (size_t j = 0; j < R[i].GetNumPort(); j++)
			{
				if (j != 0)
					fout << ",";
				fout << R[i].GetPath()[j].port;
			}
			fout << '\t'<< R[i].GetNumPort() + 1 << '\t';
			for (size_t j = 0; j < R[i].GetNumPort() + 1; j++)
			{
				if (j != 0)
					fout << ",";
				fout << R[i].GetPath()[j].port;
			}
			fout << "\t";
			for (size_t j = 0; j < R[i].GetNumPort() + 1; j++)
			{
				if (j != 0)
					fout << ",";
				fout << R[i].GetPath()[j].arrivalTime;
			}
			fout << std::endl;
		}
		fout.close();
	}
}

void output_nodes_data(const string& filename, ST_network& ST)
{
	ofstream fout(filename);
	if (!fout.is_open())
	{
		std::cout << "can't open nodes file" << std::endl;
	}
	else
	{
		fout << "ID	Route	Call	Port	Round_trip	Time" << std::endl;
		std::vector<Node> N = ST.GetN();
		for (size_t i = 0; i < N.size(); i++)
		{
			fout << N[i].GetNodeID() << '\t'
				<< N[i].GetRouteID() << '\t'
				<< N[i].GetCall() << '\t'
				<< N[i].GetPort() << '\t'
				<< N[i].GetRotation() << '\t'
				<< N[i].GetTime() << std::endl;
		}
	}
}

void output_vesselpaths_data(const string& filename, ST_network& ST)
{
	ofstream fout(filename);
	if (!fout.is_open())
	{
		std::cout << "can't open vessel path file" << std::endl;
	}
	else
	{
		std::vector<VesselPath> V = ST.GetVP();

		// output the vessel path in Set V
		fout << "VesselPathID\tVesselRouteID\tNumOfArcs\tArcIDs\toriginTime\tdestinationTime\n";
		int vssselpath_count = 0;
		for (size_t i = 0; i < V.size(); i++)
		{
			fout << V[i].GetPathID() << '\t' 
					<< V[i].GetRouteID() << '\t' 
					<< V[i].GetNumArcs() << '\t';
			for (size_t k = 0; k < V[i].GetArcIDs().size(); k++)
			{
					if (k == 0)
						fout << V[i].GetArcIDs()[k];
					else
						fout << "," << V[i].GetArcIDs()[k];
			}
			fout << '\t';
			fout << V[i].GetOriginTime() << "\t";
			fout << V[i].GetDestinationTime();
			fout << std::endl;
		}
	}
}

void output_TravelArc_data(const string& filename, ST_network& ST)
{
	ofstream fout(filename);
	if (!fout.is_open())
	{
		std::cout << "can't open Travel arcs file" << std::endl;
	}
	else
	{
		fout << "TravelArc_ID\tRoute\torigin_node_ID\torigin_Call\torigin_Port\tRound_Trip\tOriginTime\tTravelTime\tDestination_node_ID\tDestination_Call\tDestination_Port\tDestinationTime" << std::endl;
		std::vector<Arc> Av = ST.GetAv();
		for (size_t i = 0; i < Av.size(); i++)
		{
			fout << Av[i].GetArcID()<< '\t'
				<< Av[i].GetTail().GetRouteID() << '\t'
				<< Av[i].GetTail().GetNodeID() << '\t'
				<< Av[i].GetTail().GetCall() << '\t'
				<< Av[i].GetTail().GetPort() << '\t'
				<< Av[i].GetTail().GetRotation() << '\t'
				<< Av[i].GetTail().GetTime() << '\t'
				<< Av[i].GetTravelTime() << '\t'
				<< Av[i].GetHead().GetNodeID() << '\t'
				<< Av[i].GetHead().GetCall()<< '\t'
				<< Av[i].GetHead().GetPort() << '\t'
				<< Av[i].GetHead().GetTime() << std::endl;
		}
	}
}

void output_transshipArcs_data(const string& filename, ST_network& ST)
{
	ofstream fout(filename);
	if (!fout.is_open())
	{
		std::cout << "can't open transship arcs file" << std::endl;
	}
	else
	{
		fout << "TransshipArc ID	Port	origin_node_ID	OriginTime	TransshipTime	Destination_node_ID	DestinationTime	FromRoute	ToRoute" << std::endl;
		std::vector<Arc> At = ST.GetAt();
		for (size_t i = 0; i < At.size(); i++)
		{
			fout << At[i].GetArcID()<< '\t'
				<< At[i].GetTail().GetPort() << '\t'
				<< At[i].GetTail().GetNodeID() << '\t'
				<< At[i].GetTail().GetTime() << '\t'
				<< At[i].GetTravelTime() << '\t'
				<< At[i].GetHead().GetNodeID() << '\t'
				<< At[i].GetHead().GetTime() << '\t'
				<< At[i].GetTail().GetRouteID() << '\t'
				<< At[i].GetHead().GetRouteID() << std::endl;
		}
	}
}

void output_paths_data(const string& filename, GeneratePath& GP)
{
	const vector<ContainerPath>& Ps = GP.GetPathSet();

	time_t sys_t = time(0);
	struct tm ti;
	localtime_s(&ti, &sys_t);

	ofstream fout(filename);
	if (!fout.is_open())
	{
		std::cout << "Can't output file" << std::endl;
	}
	fout << "Path ID	OriginPort	OriginTime	DestinationPort	DestinationTime	PathTime	TransshipPort	TransshipTime	PortPath_length	PortPath	Arcs_length	ArcsID" << std::endl;

	for (size_t i = 0; i < Ps.size(); i++)
	{
		Ps[i].outputPath(fout);
	}
}

void output_requests_data(const string& filepath, GeneratePath& GP)
{
	const vector<OD>& ODset = GP.GetODSet();
	const vector<ContainerPath>& PathSet = GP.GetPathSet();

	time_t sys_t = time(0);
	struct tm ti;
	localtime_s(&ti, &sys_t);

	ofstream fout(filepath + "Requests.txt");
	ofstream fout_laden_paths(filepath + "LadenP-aths.txt");
	ofstream fout_empty_paths(filepath + "EmptyP-aths.txt");
	if (!fout.is_open())
	{
		std::cout << "can't output request" << std::endl;
	}
	std::cout << "RequestID\tOriginPort\tDestinationPort\tW_i_Earlist\tLatestDestinationTime\tLadenPaths\tNumberOfLadenPath\tEmptyPaths\tNumberOfEmptyPath" << std::endl;
	fout << "RequestID\tOriginPort\tDestinationPort\tW_i_Earlist\tLatestDestinationTime	LadenPaths	NumberOfLadenPath\tEmptyPaths\tNumberOfEmptyPath" << std::endl;
	fout_laden_paths << "RequestID\tOriginPort\tOriginTime\tDestinationPort\tRoundTrip\tW_i_Earlist\tArrivalTime_to_Destination\tPathTime\tTransshipPort\tTransshipTime\tPort_Path\tPathID\tArcIDs" << std::endl;
	fout_empty_paths << "RequestID\tOriginPort\tOriginTime\tNumOfEmptyPath\tPathIDs" << std::endl;
	int RequestID = 0;
	for (size_t i = 0; i < ODset.size(); i++)
	{
		std::cout << (++RequestID) << '\t'
			<< ODset[i].originPort << '\t'
			<< ODset[i].destinationPort << '\t'
			<< ODset[i].earliest_pickup_time << '\t'
			<< ODset[i].deadline_time << '\t';
		fout << RequestID << '\t'
			<< ODset[i].originPort << '\t'
			<< ODset[i].destinationPort << '\t'
			<< ODset[i].earliest_pickup_time << '\t'
			<< ODset[i].deadline_time << '\t';
		if (ODset[i].laden_pathIDs.empty())
		{
			std::cout << 0 << '\t' << 0 << '\t';
			fout << 0 << '\t' << 0 << '\t';
		}
		else
		{
			for (size_t j = 0; j < ODset[i].laden_pathIDs.size(); j++)
			{
				if (j == 0)
				{
					std::cout << ODset[i].laden_pathIDs[j];
					fout << ODset[i].laden_pathIDs[j];
				}
				else
				{
					std::cout << "," << ODset[i].laden_pathIDs[j];
					fout << "," << ODset[i].laden_pathIDs[j];
				}

				// output laden path
				ContainerPath tempPath = PathSet[ODset[i].laden_pathIDs[j] - 1];
				fout_laden_paths << RequestID << '\t'
					<< tempPath.GetOriginPort() << '\t'
					<< tempPath.GetOriginTime() << '\t'
					<< tempPath.GetDestinationPort() << '\t'
					<< ODset[i].round_trip << '\t'
					<< ODset[i].earliest_pickup_time << '\t'
					<< tempPath.GetDestinationTime() << '\t'
					<< tempPath.GetPathTime() << '\t';
				// output transshipment port and time
				if (tempPath.GetTransshipPorts().empty())
				{
					fout_laden_paths << 0 << '\t' << 0 << '\t';
				}
				else
				{
					for (size_t k = 0; k < tempPath.GetTransshipPorts().size(); k++)
					{
						if (k != 0)
							fout_laden_paths << ",";
						fout_laden_paths << tempPath.GetTransshipPorts()[k];
					}
					fout_laden_paths << '\t';
					for (size_t k = 0; k < tempPath.GetTransshipTime().size(); k++)
					{
						if (k != 0)
							fout_laden_paths << ",";
						fout_laden_paths << tempPath.GetTransshipTime()[k];
					}
					fout_laden_paths << '\t';
				}
				fout_laden_paths << tempPath.GetPathID() << '\t';
				// output arcIDs
				for (size_t k = 0; k < tempPath.GetPathPorts().size(); k++)
				{
					if (k != 0)
						fout_laden_paths << ",";
					fout_laden_paths << tempPath.GetPathPorts()[k];
				}
				fout_laden_paths << '\t';
				for (size_t k = 0; k < tempPath.GetArcIDs().size(); k++)
				{
					if (k != 0)
						fout_laden_paths << ",";
					fout_laden_paths << tempPath.GetArcIDs()[k];
				}
				fout_laden_paths << std::endl;
			}
			std::cout << '\t';
			fout << '\t';
			std::cout << ODset[i].num_laden_paths << '\t';
			fout << ODset[i].num_laden_paths << '\t';
		}

		fout_empty_paths << RequestID << '\t'
			<< ODset[i].originPort << '\t'
			<< ODset[i].earliest_pickup_time << '\t'
			<< ODset[i].num_empty_paths << '\t';
		if (ODset[i].empty_pathIDs.empty())
		{
			std::cout << 0 << '\t' << 0;
			fout << 0 << '\t' << 0;
			fout_empty_paths << 0;
		}
		else
		{
			// output empty paths
			for (size_t j = 0; j < ODset[i].empty_pathIDs.size(); j++)
			{
				if (j == 0)
				{
					std::cout << ODset[i].empty_pathIDs[j];
					fout << ODset[i].empty_pathIDs[j];
					fout_empty_paths << ODset[i].empty_pathIDs[j];
				}
				else
				{
					std::cout << "," << ODset[i].empty_pathIDs[j];
					fout << "," << ODset[i].empty_pathIDs[j];
					fout_empty_paths << "," << ODset[i].empty_pathIDs[j];
				}
			}
			std::cout << '\t';
			std::cout << ODset[i].num_empty_paths;
			fout << '\t';
			fout << ODset[i].num_empty_paths;
		}
		fout_empty_paths << std::endl;
		std::cout << std::endl;
		fout << std::endl;
	}
}
}