#include "space_time_network.h"

namespace fleetdeployment {

	extern  int maxTransshipTime = 14;

	ST_network::ST_network(std::vector<ship_route>& R,  map<string, Port>& P,  vector<OD>& W)
		:	R(R),
			P(P),
			W(W),
		NR(std::vector<int>(R.size()))
	{
		generateST(R);
	}

	ST_network::ST_network(std::vector<ship_route>& Rs, map<string,  Port>& P)
		:	R(Rs),
			P(P)
	{
		generateST(R);
	}

	ST_network::ST_network(std::vector<ship_route>& Rs, vector<OD>& W)
		: R(Rs),
		  W(W)
	{
		generateST(R);
	}

	ST_network::ST_network(std::vector<ship_route>& Rs)
		:	R(Rs)
	{
		generateST(R);
	}

	ST_network::ST_network(Data& data)
		: R(data.GetR()),
		P(data.GetP()),
		W(data.GetW()),
		NR(std::vector<int>(data.GetR().size()))
	{
		generateST(R);
	}

	// dtr
	ST_network::~ST_network()
	{
	}

	void ST_network::generateST(std::vector<ship_route>& R)
	{
		this->generateNodes(R);
		this->generateTravelArc();
		this->generateTransshipArc();

		//this->generateAdjmatrix();

#ifdef _GENERATE_NEW_OD_PAIRS_SET_

		this->generate_new_od_set();

#endif // _GENERATE_NEW_OD_PAIRS_SET_

		this->printStatus();
	}

	std::vector<Node>& ST_network::generateNodes(std::vector<ship_route>& R)
	{
		Rs Rs(R);
		//Generate Nodes Set N
		int index = 0;
		std::cout << "----------the Nodes in N as follows :----------" << std::endl;
		std::cout << "NodeID" << "(" << "RouteID" << ", " <<  "Call"<< ", " << "Port" << ", " << "Rotation" << ", " << "Time" << ")" << std::endl;
		int node_count = 0;
		for (int i = 0; i < R.size(); i++)
		{
			Node Nrit;
			Nrit.SetRouteID(R[i].GetRouteID());
			for (int kk = 0; kk < NT; kk++)
			{
#ifdef AVOID_INCOMPLETE_ROTATION
				// avoid incomplete rotation
				if (R[i].GetPath()[0].arrivalTime + 7 * day_length * kk + R[i].GetTransitTime() > T)
				{
					continue;
				}
#endif // AVOID_INCOMPLETE_ROTATION

				for (int j = 0; j < R[i].GetNumPort() + 1; j++)
				{
					Nrit.SetCall(R[i].GetPath()[j].call);
					Nrit.SetTime(R[i].GetPath()[j].arrivalTime + 7 * day_length * kk);

					if (j == 0 
						&& Nrit.GetTime() >= R[i].GetPath()[0].arrivalTime + R[i].GetTransitTime())
						continue;

					// avoid over-timehorizon node 
					if (Nrit.GetTime() >= T) {
						continue;
					}

					node_count++;
					Nrit.SetNodeID(node_count);
					Nrit.SetPortString(Rs.GetPort(Nrit.GetRouteID(), Nrit.GetCall()));
					Nrit.SetRotation( kk + 1);
					Nrit.SetRoundTrip(R[i].GetTransitTime());
					N.push_back(Nrit);
					if (Nrit.GetTime() < 7 * day_length)
					{
						N1.push_back(Nrit);
					}
					std::cout << Nrit.GetNodeID() << ": ";
					std::cout << "(" 
						<< Nrit.GetRouteID() << ", " 
						<< Nrit.GetCall() << ", " 
						<< Nrit.GetPort() << ", " 
						<< Nrit.GetRotation() << ", " 
						<< Nrit.GetTime() << ")" 
						<< std::endl;

					// update num_rotation
					if (R[i].GetNumRotation() < Nrit.GetRotation())
					{
						R[i].num_of_rotation = Nrit.GetRotation();
						NR[i] = Nrit.GetRotation();
					}
				}
			}
		}

		return N;
	}

	std::vector<Arc>& ST_network::generateTravelArc()
	{
		//Generate Voyage arcs Set Av
		std::cout << "the arcs in Av : " << std::endl;
		int arc_count = 0;
		int vesselpath_count = 0;
		int temp_route = -1;
		int temp_rotation = -1;
		Node tail, head;
		VesselPath temp_vesselpath;
		std::vector<std::vector<VesselPath>> vs(R.size());
		for (size_t i = 0; i < vs.size(); i++)
		{
			temp_vesselpath.SetRotation( -1);
			temp_vesselpath.SetNumArcs(0);
			vs[i] = vector<VesselPath>(180 / 7, temp_vesselpath);
		}
		for (int i = 0; i < N.size(); i++)
		{
			tail = N[i];
			int head_route_index = tail.GetRouteID();
			int head_call, head_arrival_time;
			if (tail.GetCall() < R[head_route_index - 1].GetPath().size())
			{
				head_call = R[head_route_index - 1].GetPath()[tail.GetCall()].call;
				head_arrival_time = tail.GetTime() 
					+ R[head_route_index - 1].GetPath()[tail.GetCall()].arrivalTime 
					- R[head_route_index - 1].GetPath()[tail.GetCall() - 1].arrivalTime;
			}
			else
			{
				head_call = R[head_route_index - 1].GetPath()[1].call;
				head_arrival_time = tail.GetTime() 
					+ R[head_route_index - 1].GetPath()[1].arrivalTime 
					- R[head_route_index - 1].GetPath()[0].arrivalTime;
			}
			if(head_arrival_time )
			head = GetNode(head_route_index, head_call, head_arrival_time);
			Arc tempArc;
			if (head.GetNodeID() != -1)
			{
				tempArc.SetArcID(++arc_count);
				tempArc.SetTail(tail);
				tempArc.SetHead(head);
				tempArc.SetTravelTime(head.GetTime() - tail.GetTime());
				tempArc.SetCost(0);
				tempArc.printArc();
				std::cout << std::endl;

				Av.push_back(tempArc);
				A.push_back(tempArc);
			}
			else
			{
				continue;
			}

			// Get the vesselpath (route and rotation) corresponding to node N[i] 
			if (tail.GetCall() <= R[tail.GetRouteID() - 1].GetNumPort())
			{
				temp_rotation = (tail.GetTime() - R[tail.GetRouteID() - 1].GetPath()[tail.GetCall() - 1].arrivalTime) / 7;
				vs[tail.GetRouteID() - 1][temp_rotation].SetRotation(temp_rotation + 1);
				vs[tail.GetRouteID() - 1][temp_rotation].SetRouteID(tail.GetRouteID());
			}
			else
			{
				temp_rotation = (tail.GetTime() - R[tail.GetRouteID() - 1].GetPath()[0].arrivalTime) / 7;
				vs[tail.GetRouteID() - 1][temp_rotation].SetRotation(temp_rotation + 1);
				vs[tail.GetRouteID() - 1][temp_rotation].SetRouteID(tail.GetRouteID());
			}
			vs[tail.GetRouteID() - 1][temp_rotation].AddArcID2ArcIDs(tempArc.GetArcID());
			vs[tail.GetRouteID() - 1][temp_rotation].AddArc2Arcs(tempArc);
			vs[tail.GetRouteID() - 1][temp_rotation].NumArcPlusOne();
		}

		// add the vessel path to Set V
		int vssselpath_count = 0;
		for (size_t i = 0; i < R.size(); i++)
		{
			for (size_t j = 0; j < vs[i].size(); j++)
			{
				if (vs[i][j].GetRotation() != -1)
				{
					vs[i][j].SetPathID(++vesselpath_count);
					vs[i][j].SetOriginTime( vs[i][j].GetArcs().front().GetTail().GetTime() );
					vs[i][j].SetDestinationTime(vs[i][j].GetArcs().back().GetHead().GetTime());
					VP.push_back(vs[i][j]);
				}
			}
		}

		// print vessel paths
		std::cout << "vesselPath ID: \t" 
			<< "route ID\t" 
			<< "rotation\t" 
			<< "num of Arcs\t" 
			<< "Arcs\t" 
			<< "Nodes\t"
			<< "OriginTime\t"
			<< "DestinationTime\t"
			<< std::endl;
		for ( auto& x : VP)
		{
			x.printPath();
		}

		return Av;
	}

	std::vector<Arc>& ST_network::generateTransshipArc()
	{
		//Generate Transshipment arc Set At
		int arc_count = (int)Av.size();
		std::cout << "the arcs in At :" << std::endl;
		for (int i = 0; i < N.size(); i++)
		{
			for (int j = 0; j < N.size(); j++)
			{
				/*if (i != j)*/
				if (N[i].GetRouteID() != N[j].GetRouteID())
				{
					if (N[i].GetPort() == N[j].GetPort())
					{
						if (N[i].GetTime() <= N[j].GetTime() && N[j].GetTime() < N[i].GetTime() + maxTransshipTime * day_length)
						{
							Arc tempArc;
							tempArc.SetArcID(++arc_count);
							tempArc.SetTail(N[i]);
							tempArc.SetHead(N[j]);
							tempArc.SetTravelTime(N[j].GetTime() - N[i].GetTime());
							tempArc.SetCost(P[N[i].GetPort()].GetTransshipmentCost());
							At.push_back(tempArc);
							A.push_back(tempArc);
							tempArc.printArc();
							std::cout << std::endl;
						}
					}
				}
			}
		}

		return At;
	}

	std::vector<std::vector<int>>& ST_network::generateAdjmatrix()
	{
		// generate the adjacency matrix
		adjMatrix = vector<std::vector<int>>(N.size(), vector<int>(N.size(), 0));
		for (int i = 0; i < N.size(); i++)
		{
			for (int j = 0; j < N.size(); j++)
			{
				Arc tempArc;
				tempArc.SetTail(N[i]);
				tempArc.SetHead(N[j]);
				if (find(Av.begin(), Av.end(), tempArc) != Av.end() 
					|| find(At.begin(), At.end(), tempArc) != At.end())
				{
					SetTrue(i, j);
				}
			}
		}

		return adjMatrix;
	}

	std::vector<std::vector<pair<Node, Node>>>&  ST_network::generate_new_od_set()
	{
		// Generate the Set of new OD pairs W^od
		// step1 : ruct the Set of No
		// step2 : ruct the Set of Nm
		// step3 : remove unfeasible Node
		// step4 : generate new OD pairs for Wod
		std::cout << "----------the new OD pairs in Wod as follows :----------" << std::endl;
		for (auto od : W)
		{
			std::vector<pair<Node, Node>> W_od;

			// put the node which port is origin_port into the Set No0
			std::vector<Node> No0;
			for (int i = 0; i < N.size(); i++)
			{
				if (N[i].GetPort() == od.GetOrigin() /*&& N[i].arrivalTime < 7 * day_length*/)
				{
					No0.push_back(N[i]);
				}
			}

			// put the reachable node which port is destination port correspordding to No nodes into the Set Nd0
			std::vector<std::vector<Node>> Nd0;
			for (int i = 0; i < No0.size(); i++)
			{
				std::vector<Node> Nm;
				for (int j = 0; j < N.size(); j++)
				{
					if (N[j].GetPort() == od.GetDestination() /*&& N[i].arrivalTime - (*it).arrivalTime <= od.GetMaxTransitTime()*/)
					{
						if (isReachable(No0[i], N[j]))
						{
							Nm.push_back(N[j]);
						}
					}
				}
				Nd0.push_back(Nm);
			}

			// update No and Nd 
			// delete the empty node pairs
			std::vector<Node> No;
			std::vector<std::vector<Node>> Nd;
			for (int i = 0; i < Nd0.size(); i++)
			{
				if (Nd0[i].empty())
				{
					continue;
				}
				else
				{
					No.push_back(No0[i]);
					Nd.push_back(Nd0[i]);
				}
			}

			std::cout << od.GetOrigin() << " ---- " << od.GetDestination() << "(" << od.GetDemand() << ") :" << std::endl;
			if (No.empty())
			{
				std::cout << "No feasible new od Node pairs!" << std::endl;
			}
			for (int i = 0; i < No.size(); i++)
			{
				for (int j = 0; j < Nd[i].size(); j++)
				{
					W_od.push_back(pair<Node, Node>(No[i], Nd[i][j]));
					No[i].printNodeSelf();
					Nd[i][j].printNodeSelf();
				}
			}

			Wod.push_back(W_od);
		}

		return Wod;
	}

	const int ST_network::GetArcID(const  Node& tail, const  Node& head) const
	{
		for (size_t i = 0; i < A.size(); i++)
		{
			if (A[i].GetTail() == tail && A[i].GetHead() == head)
			{
				return A[i].GetArcID();
			}
		}
		return -1;
	}

	 const int ST_network::GetArcID( int tail_nodeID,  int head_nodeID) const
	{
		for (size_t i = 0; i < A.size(); i++)
		{
			if (A[i].GetTail().GetNodeID() == tail_nodeID && A[i].GetHead().GetNodeID() == head_nodeID)
			{
				return A[i].GetArcID();
			}
		}
		return -1;
	}

	 const int ST_network::GetArcID_after_oneweek( int arcID) const
	{
		Arc tempArc = GetArc(arcID);
		Node tail_node = tempArc.GetTail();
		Node head_node = tempArc.GetHead();
		int nextweek_tail_nodeID = GetNodeID_after_oneweek(tail_node);
		int nextweek_head_nodeID = GetNodeID_after_oneweek(head_node);

		if (nextweek_tail_nodeID == -1 || nextweek_head_nodeID == -1) {
			return -1;
		}

		Node nextweek_tail_node = GetNode_after_oneweek(tail_node);
		Node nextweek_head_node = GetNode_after_oneweek(head_node);
		//GetArc(nextweek_tail_node, nextweek_head_node);

		int newArcID = GetArcID(nextweek_tail_nodeID, nextweek_head_nodeID);
		return newArcID;
	}

	 const int ST_network::GetNodeID_after_oneweek(const Node& node) const
	{
		for (size_t i = 0; i < N.size(); i++)
		{
			if ((N[i].GetRouteID() == node.GetRouteID()
				&& N[i].GetPort() == node.GetPort()
				&& N[i].GetTime() == node.GetTime() + 7 * day_length)
				&&
				(N[i].GetRotation() == node.GetRotation() + 1
				|| N[i].GetRotation() + N[i].GetRoundTrip() / (7 * day_length) == node.GetRotation() + 1
					))
			{
				return N[i].GetNodeID();
			}
		}
		return -1;
	}

	const Node& ST_network::GetNode_after_oneweek(const Node& node) const
	{
		for (size_t i = 0; i < N.size(); i++)
		{
			if ((N[i].GetRouteID() == node.GetRouteID()
				&& N[i].GetPort() == node.GetPort()
				&& N[i].GetTime() == node.GetTime() + 7 * day_length)
				&&
				(N[i].GetRotation() == node.GetRotation() + 1
					|| N[i].GetRotation() + N[i].GetRoundTrip() / (7 * day_length) == node.GetRotation() + 1
					))
			{
				return N[i];
			}
		}
		throw;
	}

	const int ST_network::GetNodeIndex(const  Node& node) const
	{
		for (int i = 0; i < N.size(); i++)
		{
			if (N[i] == node)
				return i;
		}
		return -1;
	}

	const int ST_network::GetN1Index(const  Node& node) const
	{
		for (int i = 0; i < N1.size(); i++)
		{
			if (N1[i] == node)
				return i;
		}
		return -1;
	}

	 const int ST_network::GetAIndex(const Arc& arc) const
	{
		for (int i = 0; i < A.size(); i++)
		{
			if (arc == A[i] )
				return i;
		}
		//return arc.GetArcID() - 1;
		return -1;
	}

	 bool ST_network::isReachable(const  Node& u, const  Node& v)
	{
		std::vector<int> visited(N.size(), 0);
		DFS(visited, u);
		if (1 == visited[GetNodeIndex(v)])
		{
			return true;
		}
		else
		{
			return false;
		}
	}

	 const int ST_network::GetPortIndex(const vector<Port>& P,  const string& port) const
	{
		for (int i = 0; i < P.size(); i++)
		{
			if (port == P[i].GetPort())
			{
				return i;
			}
		}
		return -1;
	}

	 const int ST_network::GetODindex(const string& origin, const string& destination) const
	{
		for (size_t od = 0; od < W.size(); od++)
		{
			if (W[od].GetOrigin() == origin
				&& W[od].GetDestination() == destination)
				return (int)od;
		}
		return -1;
	}

	 const OD& ST_network::GetODbyString(const string& origin, const string& destination) const
	 {
		 for (size_t od = 0; od < W.size(); od++)
		 {
			 if (W[od].GetOrigin() == origin
				 && W[od].GetDestination() == destination)
				 return W[od];
		 }
		 throw std::runtime_error("OD not found");
	 }

	 const Node& ST_network::GetNode(int route_index, int call, int arrival_time) const
	{
		for ( auto& x : N)
		{
			if (x.GetRouteID() == route_index &&
				x.GetCall() == call &&
				x.GetTime() == arrival_time)
				return x;
		}
		
		static Node null_node;
		return null_node;
		//std::cerr << "Error in get node :" << route_index << " " << call << " " << arrival_time << std::endl;
		//throw std::runtime_error("Node not found");
	}

	 const double ST_network::GetOdMaxCost(const  Node& o, const  Node& d) const
	{
		for (size_t i = 0; i < W.size(); i++)
		{
			if (W[i].GetOrigin() == o.GetPort() && W[i].GetDestination() == d.GetPort())
				return W[i].GetMaxCost();
		}
		return 0.0;
	}

	 double const ST_network::GetOdMaxCost(const OD& od) const
	{
		std::vector<OD>::const_iterator it = find(W.begin(), W.end(), od);
		if (it != W.end())
			return (*it).GetMaxCost();
		return 0.0;
	}

	 vector<int>& ST_network::DFS(std::vector<int>& visited,  const Node& v)
	{
		int u = GetNodeIndex(v);
		visited[u] = 1;
		for (int i = 0; i < N.size(); i++)
		{
			if (GetAdjState(u, i) == 1)
			{
				visited[GetNodeIndex(N[i])] = 1;
				DFS(visited, N[i]);
			}
		}
		return visited;
	}

	 vector<OD>& ST_network::generate_od_pairs(map<string, Port>& P, vector<OD>& W)
	{
		for ( auto& x : P)
		{
			for ( auto& y : P)
			{
				if (x.first == y.first)
				{
					continue;
				}
				OD od(x.first, y.first, 0, 48);
				W.push_back(od);
			}
		}
		return W;
	}

} // namespace ST