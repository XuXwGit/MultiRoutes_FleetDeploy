#ifndef _GENERATE_PATHS_H_
#define _GENERATE_PATHS_H_

#include <vector>
#include <set>
#include <sstream>
#include <fstream>
#include <algorithm>

#include <ilcplex/ilocplex.h>

#include "data_process.h"
#include "input_data.h"
#include "space_time_network.h"
#include "checker.h"

using namespace std;

namespace fleetdeployment
{
	const int maxTravelTime = 14;

	typedef vector<std::vector<ContainerPath>>				od_pair_paths;
	typedef vector<od_pair_paths>				container_paths;

	class  GeneratePath
	{
	private:
		ST_network  ST;
		std::vector<ContainerPath> PathSet;
		std::vector<OD> ODset;

		std::vector<ContainerPath> P0;
		std::vector<ContainerPath> P1;
		std::vector<ContainerPath> P2;
		std::vector<ContainerPath> P3;
		std::vector<ContainerPath> P4;

		ofstream fout;

	public:
		GeneratePath(const ST_network& ST, const string& output_filepath);
		~GeneratePath();

		inline const vector<ContainerPath>& GetPathSet()
		{
			return PathSet;
		}
		inline const vector<OD>& GetODSet()
		{
			return ODset;
		}

		const vector<OD>& separateOD();

		void reloadHistoryPaths(const string& output_filepath);
		std::vector<ContainerPath>& generate_paths_seperatly();
		std::vector<ContainerPath>& generate_paths_input_type0();
		std::vector<ContainerPath>& generate_paths_input_type00();
		std::vector<ContainerPath>& generate_paths_input_type1();
		std::vector<ContainerPath>& generate_paths_input_type2();
		std::vector<ContainerPath>& generate_paths_input_type3();
		std::vector<ContainerPath>& generate_paths_input_type33();
		std::vector<ContainerPath>& generate_paths_input_type4();



		std::vector<ContainerPath> connectOD(const OD& odod);
		ContainerPath connect_two_paths(const ContainerPath& first_Path,  const ContainerPath& second_Path);

		std::vector<ContainerPath>& numbering_path();

		ContainerPath& Path_add_oneweek(ContainerPath& path);
		std::vector<ContainerPath>& Get_paths_set(const container_paths& container_paths, vector<ContainerPath>& Ps);

		std::vector<ContainerPath>& cut_long_paths();

		std::vector<ContainerPath> extend_from_one_to_multi_stage_paths(const ContainerPath& Ps);

		std::vector<ContainerPath> extend_multi_stage_paths(std::vector<ContainerPath>& Ps);
		std::vector<OD>& generate_all_request();

		std::vector<ContainerPath> Get_node_pair_paths(const Node& n_src, const  Node& n_sink);
	};
}
#endif // !_GENERATE_PATHS_H_