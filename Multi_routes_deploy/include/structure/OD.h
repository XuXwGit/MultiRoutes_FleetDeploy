#ifndef _OD_H_
#define _OD_H_

#include <string>
#include <vector>
#include <set>

#include "ContainerPath.h"

namespace fleetdeployment {
	class OD;
}

class fleetdeployment::OD {
	//private:
public:
	int odID;
	int round_trip;
	std::string originPort;					// o ¡Ê P
	std::string destinationPort;			// d ¡Ê P
	int maxTransitTime;				// T^_max_od
	int maxCost;
	int demand;								// TTS demand
	int containerPathNum;			// n_od
	std::vector<int> transitTime;		// t_od_1, t_od_2, ... , t_od_(n_od)
	double freightRate;				// g'od : the freight rate of this o-d pairs
	double maxProfitRate;			// god = g'od - co(the loading cost of port o) - cp(the discharge cost of port p)
	double cost_by_alliances;

	double normal_demand;
	double varable_demand;

	int shortest_Travel_time = 180;

	int earliest_pickup_time = 0;
	int deadline_time = 180;

	std::vector<int> laden_pathIDs;
	std::vector<int> empty_pathIDs;
	std::vector<ContainerPath> laden_paths;
	std::vector<ContainerPath> empty_paths;

	std::set<ContainerPath> laden_path_set;

	// type : 0 / 1 /2 / 3 / 4
	int type;
	std::vector<OD> composition;

	int num_laden_paths = 0;
	int num_empty_paths = 0;

public:
	inline OD()
		: odID(0),
		round_trip(0),
		originPort(""),
		destinationPort(""),
		maxTransitTime(0),
		maxCost(0),
		demand(0),
		containerPathNum(0),
		freightRate(0.0),
		maxProfitRate(0.0),
		cost_by_alliances(0.0),
		normal_demand(0.0),
		varable_demand(0.0),
		type(0)
	{	}
	inline OD(const std::string& origin, const std::string& destination,
		const int earlist_setup_time, const int deadline_time)
		: originPort(origin),
		destinationPort(destination),
		earliest_pickup_time(earlist_setup_time),
		deadline_time(deadline_time)
	{	}
	inline OD(const std::string& origin, const std::string& destination, const int earlist_setup_time,		const int deadline_time, const int roundtrip)
		:originPort(origin),
		destinationPort(destination),
		earliest_pickup_time(earlist_setup_time),
		deadline_time(deadline_time),
		round_trip(roundtrip)
	{	}
	inline OD(const std::string& origin, const std::string& destination, const int& demand,
		const double& freightRate, const double& maxProfit)
		: originPort(origin),
		destinationPort(destination),
		demand(demand),
		maxTransitTime(672),
		maxCost((int)freightRate),
		freightRate(freightRate),
		maxProfitRate(maxProfit),
		cost_by_alliances(0)
	{}
	inline OD(const std::string& origin, const std::string& destination)
		: originPort(origin),
		destinationPort(destination)
	{	}
	inline ~OD() {}

	inline void set_composition(const std::vector<OD>&compos)
	{
		composition = compos;
	}

	inline const std::vector<OD>& get_composition() const
	{
		return composition;
	}

	inline const std::string& GetOrigin() const
	{
		return originPort;
	}

	inline const std::string& GetDestination() const
	{
		return destinationPort;
	}

	inline int GetDemand()
	{
		return demand;
	}

	inline int GetMaxTransitTime()
	{
		return maxTransitTime;
	}

	inline double GetMaxProfitRate()
	{
		return maxProfitRate;
	}

	inline const double GetMaxCost() const
	{
		return maxCost;
	}

	inline double GetCostByAlliances()
	{
		return cost_by_alliances;
	}

	bool operator==(const OD& right) const
	{
		return odID == right.odID;
	}
};


#endif // !_OD_H_
