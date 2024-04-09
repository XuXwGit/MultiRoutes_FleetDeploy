#ifndef _SPACETIMENETWORK_H_
#define _SPACETIMENETWORK_H_

#define AVOID_INCOMPLETE_ROTATION

#include <utility>
#include <vector>
#include <sstream>
#include <iostream>
#include <fstream>
#include <algorithm>

#include "data_process.h"
#include "input_data.h"
#include "vesselpath.h"
#include "node.h"
#include "arc.h"
#include "Port.h"
#include "ship_route.h"

using namespace std;

// int maxTod = 480;
// int NT = (maxTod - 1) / 7 * day_length + 1 + 1;

namespace fleetdeployment {
	class  ST_network;
}


// Space-time Network model
// input : data about shipping route (include shedule)
	// R : the Set of shipping routes
	// P
	// W
// output : G(N, A)
	// N : the Set of Nodes
	// A : the Set of arcs
class  fleetdeployment::ST_network
{
public:
	int day_length = 1;
	int T = 180;
	int NT = (T - 1) / (7 * day_length) + 1;
	std::vector<int> NR;
private:
	std::vector<ship_route> R;												// the Set of shipping route
	map<string, Port> P;														// the Set of port
	std::vector<OD> W;														// the Set of od port pairs
	std::vector<Node> N;														// the Set of node (r, i, t)
	std::vector<Node> N1;													// the Set of node in first week
	std::vector<Arc> A;														// the Set of arc A = At + Av
	std::vector<Arc> Av;														// the Set of voyage arc
	std::vector<Arc> At;														// the Set of transshipment arc
	std::vector<VesselPath> VP;											// the Set of vessel paths / rotation
	std::vector<std::vector<int>> adjMatrix;						// adjacency matrix refer to the space-time network Graph
	std::vector<std::vector<pair<Node, Node>>> Wod;		// the Set of new OD pairs

public:
	/*
	* Constructor
	*/
	explicit ST_network(vector<ship_route>& R, map<string, Port>& P, vector<OD>& W);
	explicit ST_network(vector<ship_route>& R, map<string, Port>& P);
	explicit ST_network(vector<ship_route>& R, vector<OD>& W);
	explicit ST_network(vector<ship_route>& R);
	explicit ST_network(Data& data);
	~ST_network();

	/*
	* Generate the space-time network
	*/
	void generateST(std::vector<ship_route>& R);
	std::vector<Node>& generateNodes(std::vector<ship_route>& R);
	std::vector<Arc>& generateTravelArc();
	std::vector<Arc>& generateTransshipArc();
	std::vector<std::vector<int>>& generateAdjmatrix();
	std::vector<std::vector<pair<Node, Node>>>& generate_new_od_set();
	std::vector<OD>& generate_od_pairs(map<string, Port>& P, vector<OD>& W);


	/*
	* Print the space-time network Status
	*/
	inline void printStatus() const
	{
		// out put the number of nodes and arcs
		std::cout << "N.size() = " << N.size() << std::endl
			<< "A.size() = " << A.size() << std::endl
			<< "Av.size() = " << Av.size() << std::endl
			<< "At.size() = " << At.size() << std::endl
			<< "W.size() = " << W.size() << std::endl;
	}

	/*
	* Getters
	*/
	inline const vector<OD>& GetW() const{		return W;	}
	inline const vector<ship_route>& GetR() const {		return R;	}
	inline const vector<std::vector<pair<Node, Node>>>& GetWod() const {		return Wod;	}
	inline const vector<Node>& GetN() const {		return N;	}
	inline const vector<Node>& GetN1() const {		return N1;	}
	inline const vector<Arc>& GetA() const {		return A;	}
	inline const vector<Arc>& GetAt()const {		return At;	}
	inline const vector<Arc>& GetAv()const {		return Av;	}
	inline const map<string, Port>& GetP() const {		return P;	}
	inline const vector<VesselPath>& GetVP() const {		return VP;	}
	inline const Arc GetArc(const int arcID) const
	{
		for (size_t i = 0; i < A.size(); i++)
		{
			if (A[i].GetArcID() == arcID) {
				return A[i];
			}
		}
		return Arc();
		// throw std::runtime_error("Arc not found");
	}
	inline const Arc GetArc(const Node& tail, const Node& head)const {
		for (size_t i = 0; i < A.size(); i++)
		{
			if (A[i].GetTail() == tail && A[i].GetHead() == head) {
				return A[i];
			}
		}

		return Arc();
		// Throw an exception if no match is found
		// throw std::runtime_error("Arc not found");
	}	
	inline [[nodiscard]] const int GetNT() const {		return NT;	}
	inline [[nodiscard]] const int GetT() const {		return T;	}
	inline [[nodiscard]] const int GetNodeID(const int route_id, int rotation, int port_call) const {
		for (auto& x : N)
		{
			if (x.GetRouteID() == route_id
				&& x.GetRotation() == rotation
				&& x.GetCall() == port_call) {
				return x.GetNodeID();
			}
		}
		return -1;
	}

	inline const bool GetAdjState(const int i, int j) const
	{
		try
		{
			if (i < N.size() && j < N.size() && i >= 0 && j >= 0)
				return adjMatrix[i][j];
		}
		catch (std::exception&)
		{
			std::cout << "out of range" << std::endl;
		}
		return 0;
	}
	[[nodiscard]] const int GetArcID(const  Node& tail, const  Node& head) const;
	[[nodiscard]] const int GetArcID(const int tail_Node_ID, int head_Node_ID) const;
	[[nodiscard]] const int GetArcID_after_oneweek(const int arcID) const;
	[[nodiscard]] const int GetNodeID_after_oneweek(const Node& node) const;
	const Node& GetNode_after_oneweek(const Node& node) const;
	[[nodiscard]] const int GetNodeIndex(const Node&) const;
	const Node& GetNode(const int route_index, int call, int arrival_time) const;
	[[nodiscard]] const int GetN1Index(const Node&) const;
	[[nodiscard]] const int GetAIndex(const Arc& arc) const;
	[[nodiscard]] const int GetPortIndex(const vector<Port>&, const string&) const;
	[[nodiscard]] const int GetODindex(const string& origin, const string& destination) const;
	const OD& GetODbyString(const string& origin, const string& destination) const;
	[[nodiscard]] const double GetOdMaxCost(const Node&, const Node&) const;
	[[nodiscard]] const double GetOdMaxCost(const OD&) const;
	[[nodiscard]] bool isReachable(const Node&, const Node&);

	/*
	* setters
	*/
	inline bool SetW(const vector<OD>& W)
	{
		try
		{
			this->W = W;
		}
		catch (std::exception&)
		{
			return false;
		}
		return true;
	}
	inline bool SetTrue(const int i, int j)
	{
		if (i < N.size() && j < N.size())
		{
			adjMatrix[i][j] = 1;
			return true;
		}
		return false;
	}
	inline bool SetFalse(const int i, int j)
	{
		if (i < N.size() && j < N.size())
		{
			adjMatrix[i][j] = 0;
			return true;
		}
		return false;
	}

	std::vector<int>& DFS(std::vector<int>&, const Node&);
};
#endif // !_SPACETIMENETWORK_H_
