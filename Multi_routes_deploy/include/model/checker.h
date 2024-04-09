#ifndef _CHECKER_H_
#define _CHECKER_H_

#include <algorithm>
#include <set>

#include "ContainerPath.h"
#include "space_time_network.h"
#include "data_process.h"


namespace fleetdeployment {

	const bool check_path_feasiblility(const ContainerPath& path, const ST_network& ST);

	std::vector<ContainerPath> check_paths(const vector<ContainerPath>& priPs, const ST_network& ST);

	std::vector<ContainerPath> cut_duplicate_paths(const vector<ContainerPath>& priPs, const ST_network& ST);
}

#endif // !_CHECKER_H_
