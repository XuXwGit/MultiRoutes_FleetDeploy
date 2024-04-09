#include "checker.h"

namespace fleetdeployment {

	const bool check_path_feasiblility(const ContainerPath& path,  const ST_network& ST)
	{
		// (1) check origin and destination

		// (2) check whether indegree equals outdegree
		if (path.GetNumArcs() > 1)
		{
			Arc first_arc, second_arc;
			for (size_t i = 0; i < path.GetNumArcs() - 1; i++)
			{
				first_arc = path.GetArcs()[i];
				second_arc = path.GetArcs()[i + 1];
				if (first_arc.GetHead().GetNodeID() != second_arc.GetTail().GetNodeID())
				{
					return false;
				}
			}
		}

		// (3) check if the origin or destination port exits transship
		Arc originArc = path.GetArcs().front();
		Arc destination = path.GetArcs().back();
		if (originArc.GetTail().GetPort() == originArc.GetHead().GetPort()
			|| destination.GetTail().GetPort() == destination.GetHead().GetPort())
		{
			return false;
		}

		//(4) check if each port at most trasship once
		set<string> Set;
		for (size_t i = 0; i < path.GetTransshipPorts().size(); i++)
		{
			Set.insert(path.GetTransshipPorts()[i]);
		}
		if (Set.size() != path.GetTransshipPorts().size())
		{
			return false;
		}

		return true;
	}

	// check the container path whether be feasible
	std::vector<ContainerPath> check_paths(const vector<ContainerPath>& Ps,  const ST_network& ST)
	{
		cut_duplicate_paths(Ps, ST);

		std::vector<ContainerPath> newPs;
		int path_count = 0;

		for (size_t i = 0; i < Ps.size(); i++)
		{
			ContainerPath Path_i = Ps[i];

			std::vector<Arc> tempArcs;
			for (size_t j = 0; j < Path_i.GetArcIDs().size(); j++)
			{
				tempArcs.push_back(ST.GetArc(Path_i.GetArcIDs()[j]));
			}
			sort(tempArcs.begin(), tempArcs.end());
			Path_i.SetArcIDs(std::vector<int>());
			for (size_t j = 0; j < tempArcs.size(); j++)
			{
				Path_i.AddArcID2ArcIDs(tempArcs[j].GetArcID());
			}

			if (check_path_feasiblility(Path_i, ST))
			{
				ContainerPath tempPath = Path_i;
				tempPath.SetPathID(++path_count);
				newPs.push_back(tempPath);
				tempPath.printPath();
			}
			else
			{
				std::cout << "exit infeasible container path" << std::endl;
				Path_i.printPath();
				throw;
			}
		}

		if (newPs.size() != Ps.size())
			throw;

		return newPs;
	}

	std::vector<ContainerPath> cut_duplicate_paths(const vector<ContainerPath>& priPs, const ST_network& ST)
	{
		std::vector<ContainerPath> newPs;
		set<std::vector<int>> paths_set;
		for (size_t i = 0; i < priPs.size(); i++)
		{
			std::vector<int> temp_arcs_sequence;
			for (size_t j = 0; j < priPs[i].GetArcIDs().size(); j++)
			{
				temp_arcs_sequence.push_back(priPs[i].GetArcIDs()[j]);
			}
			if (paths_set.find(temp_arcs_sequence) != paths_set.end())
			{
				std::cout << "Exit duplicate container path : ";
				priPs[i].printPath();
			}
			paths_set.insert(temp_arcs_sequence);
		}

		int path_count = 0;
		for (const auto& x : paths_set)
		{
			ContainerPath tempPath;
			tempPath.SetPathID((++path_count));
			tempPath.SetArcIDs(x);
			tempPath.SetNumArcs((int)x.size());
			tempPath.SetOriginPort(ST.GetArc(x.front()).GetTail().GetPort());
			tempPath.SetOriginTime(ST.GetArc(x.front()).GetTail().GetTime());
			tempPath.SetDestinationPort(ST.GetArc(x.back()).GetHead().GetPort());
			tempPath.SetDestinationTime(ST.GetArc(x.back()).GetHead().GetTime());
			tempPath.SetPathTime(tempPath.GetDestinationTime() - tempPath.GetOriginTime());
			for (size_t i = 0; i < x.size(); i++)
			{
				Arc tempArc = ST.GetArc(x[i]);
				if (tempArc.GetTail().GetPort() == tempArc.GetHead().GetPort())
				{
					tempPath.AddPort2TransshipPorts(tempArc.GetTail().GetPort());
					tempPath.AddTime2TransshipTime(tempArc.GetHead().GetTime() - tempArc.GetTail().GetTime());
				}
				else
				{
					tempPath.AddPort2PathPorts(tempArc.GetTail().GetPort());
				}
				if (i == x.size() - 1)
				{
					tempPath.AddPort2PathPorts(tempArc.GetHead().GetPort());
				}
				tempPath.AddArc2Arcs(tempArc);
			}
			newPs.push_back(tempPath);
		}

		if (newPs.size() != priPs.size())
		{
			std::cout << "Primal Path Set Size: " << priPs.size() << std::endl;
			std::cout << "New Path Set Size: " << newPs.size() << std::endl;
			throw;
		}

		return newPs;
	}
}
