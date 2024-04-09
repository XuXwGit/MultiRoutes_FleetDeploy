#ifndef _OUTPUT_DATA_H_
#define _OUTPUT_DATA_H_

#include <fstream>
#include <iostream>
#include <ctime>
#include <algorithm>
#include <set>

#include "data_process.h"
#include "generate_path.h"
#include "ContainerPath.h"

using namespace std;

namespace fleetdeployment {

	void output_STdata(const string& path,  ST_network& ST);
	void output_space_time_network_status(const string& filename, ST_network& ST);

	void output_Path_data(const string& path,  GeneratePath& GP);

	void output_paths_data(const string& filename, GeneratePath& GP);

	void output_requests_data(const string& filepath, GeneratePath& GP);

	void output_shipingroute_data(const string& filename,  ST_network& ST);

	void output_nodes_data(const string& filename, ST_network& ST);

	void output_vesselpaths_data(const string& filename,  ST_network& ST);

	void output_TravelArc_data(const string& filename,  ST_network& ST);

	void output_transshipArcs_data(const string& filename, ST_network& ST);
}

#endif // !_OUTPUT_DATA_H_
