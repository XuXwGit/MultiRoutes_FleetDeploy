/*
 ==========================================================
Algorithm for generating all practical container routes
 ==========================================================
For each (o, d) ∈ D{
    // Find all source-sink node pairs (n_src,n_sink) in the space–time network
    For(each nsrc ∈ N and para_n^src = 0 and tarr^nsrc < 7 * day_length)
    {
        For(each nsink ∈ N and para_n^sink = d and tdepara_nsink - tarr_nsrc <= Tod)
        {
            IDentify all efficient container routes from nsrc to nsink that satisfy the threshold operating cost raint.
            }// End For
        }// End For
    }// End For
==========================================================
==========================================================
*/
#include "generate_path.h"
#include "ReadData.h"

namespace fleetdeployment
{
     const double M = IloInfinity;
     const int MaxTolerancePathTimeGap = 14;
     const int MaxContainerPathLength = 180;

    GeneratePath::GeneratePath(const ST_network& st, const string& output_filepath)
        : ST(st), 
        fout(ofstream(output_filepath + "temp_paths.txt", std::ios_base::app))
    {
        if (!fout.is_open())
        {
            std::cout << "Can't output file" << std::endl;
        }

        // 检查文件是否为空
        std::ifstream fin(output_filepath + "temp_paths.txt");
        fin.seekg(0, std::ios::end);
        if (fin.tellg() == 0) {
            // 文件为空，添加标题列
            fout << "Path ID	OriginPort	OriginTime	DestinationPort	DestinationTime	PathTime	TransshipPort	TransshipTime	PortPath_length	PortPath	Arcs_length	ArcsID" << std::endl;
        }
        
        separateOD();
        reloadHistoryPaths(output_filepath);
        generate_paths_seperatly();
        cut_long_paths();
        check_paths(PathSet, ST);
        numbering_path();
          generate_all_request();
    }

    GeneratePath::~GeneratePath()
    {
    }

    const vector<OD>& GeneratePath::separateOD()
    {
        vector<OD> W = ST.GetW();
        map<string, Port> P = ST.GetP();

        // for request : od
        for (size_t od = 0; od < W.size(); od++)
        {
            OD tempOD = W[od];

            // get origin and destination (variable)
            Port destination = P[tempOD.GetDestination()];
            Port origin = P[tempOD.GetOrigin()];
            // get origin and destination (fixed)
            Port dd = P[tempOD.GetDestination()];
            Port oo = P[tempOD.GetOrigin()];

            // if origin or destination is not transship port (hub)
            // origin no transship : origin -> origin + 1
            // destination no transship : destination = destination - 1
            while (
                (!origin.GetWhetherTrans() || !destination.GetWhetherTrans())
                && origin != destination
                )
            {
                if(origin.GetNextPortSet().size() == 1)
                    origin = P[origin.GetNextPortSet()[0]];
                if(destination.GetFrontPortSet().size() == 1)
                    destination = P[destination.GetFrontPortSet()[0]];
            }

            // 0). OD = OD = 1
            // 1). OD = TT
            // 2). OD = OD / OD = OT / OD = TD
            // 3). OD = OT + TD
            // 4). OD = OT1 + T1T2 + T2D

            // one segment between origin and destination port
            // OD = OD = 1
            if ((!oo.GetWhetherTrans() && oo.GetNextPortSet()[0] == tempOD.destinationPort)
                || (!dd.GetWhetherTrans() && dd.GetFrontPortSet()[0] == tempOD.originPort))
            {
                W[od].type = 0;
                vector<OD> temp_compos;
                temp_compos.push_back(tempOD);
                W[od].set_composition(temp_compos);
            }

            // from transship port (hub) to transship port (hub) 
            // OD = TT
            else if (oo.GetWhetherTrans() 
                && dd.GetWhetherTrans())
            {
                W[od].type = 1;
                vector<OD> temp_compos;
                temp_compos.push_back(tempOD);
                W[od].set_composition(temp_compos);
            }

            // no any transportation between origin and destination
            // OD = OD
            else if ( origin == destination
                 && 
                 (!origin.GetWhetherTrans() ||
                   origin == P[tempOD.GetOrigin()] || 
                     destination == P[tempOD.GetDestination()])
                )
            {   
                W[od].type = 2;
                vector<OD> temp_compos;

                Port temp_origin = P[tempOD.GetOrigin()];
                Port temp_destination = P[tempOD.GetDestination()];
                while (temp_origin!= temp_destination)
                {
                    if (!P[tempOD.GetOrigin()].GetWhetherTrans())
                    {
                        std::string next_port = temp_origin.GetNextPortSet()[0];
                        OD oodd(temp_origin.GetPort(), next_port);
                        temp_compos.push_back(oodd);
                        temp_origin = P[next_port];
                    }
                    else
                    {
                        std::string front_port = temp_destination.GetFrontPortSet()[0];
                        OD od(front_port, temp_destination.GetPort());
                        temp_compos.push_back(od);
                        temp_destination = P[front_port];
                    }
                }
                if(!P[tempOD.GetOrigin()].GetWhetherTrans())
                    W[od].set_composition(temp_compos);
                else
                {
                    std::reverse(temp_compos.begin(), temp_compos.end());
                    W[od].set_composition(temp_compos);
                }
            }

            // exit only one transportation between origin and destination
            // OD = OT + TD
            else if (origin == destination && origin.GetWhetherTrans())
            {
                W[od].type = 3;
                vector<OD> temp_compos;
                OD OT(tempOD.GetOrigin(), origin.GetPort());
                OD TD(origin.GetPort(), tempOD.GetDestination());
                temp_compos.push_back(OT);
                temp_compos.push_back(TD);
                W[od].set_composition(temp_compos);
            }

            // exit at least two transportations between origin and destination
            // OD = OT1 + T1T2 + T2D
            else if(origin != destination 
                && origin.GetWhetherTrans() )
            {
                W[od].type = 4;
                vector<OD> temp_compos;
                OD OT(tempOD.GetOrigin(), origin.GetPort());
                OD TT(origin.GetPort(), destination.GetPort());
                OD TD(destination.GetPort(), tempOD.GetDestination());
                if(!oo.GetWhetherTrans())
                    temp_compos.push_back(OT);
                temp_compos.push_back(TT);
                if( !dd.GetWhetherTrans())
                    temp_compos.push_back(TD);
                W[od].set_composition(temp_compos);
            }

            else
            {
                std::cout << "Error in OD composition" << std::endl;
                throw;
            }

            OD temp_od = W[od];
            std::cout << temp_od.originPort << "->" << temp_od.destinationPort <<"("<<temp_od.type<<")" << " : ";
            for (size_t i = 0; i < temp_od.composition.size(); i++)
            {
                if (i != 0)
                {
                    std::cout << "+";
                }
                std::cout << "<" << temp_od.composition[i].originPort <<"--" << temp_od.composition[i].destinationPort << ">";
            }
            std::cout << std::endl;
        }

        ST.SetW(W);

        return ST.GetW();
    }

    vector<ContainerPath>& GeneratePath::generate_paths_seperatly()
    {
        // 0). OD = OD
        // 2). OD = OD / OD = OT / OD = TD
        // 1). OD = TT
        // 3). OD = OT + TD
        // 4). OD = OT1 + T1T2 + T2D
        //generate_paths_input_type0();
        //generate_paths_input_type2();
        P0 = generate_paths_input_type00();
        P3 = generate_paths_input_type3();
        P1 = generate_paths_input_type1();
        P4 = generate_paths_input_type4();

        return PathSet;
    }

    void GeneratePath::reloadHistoryPaths(const string& output_filepath)
    {
        std::vector<std::string> temp;
        int totalLine = 0;
        int columnNumber = 0;
        try {
            std::ifstream file(output_filepath + "temp_paths.txt");

            if (file.is_open()) {
                std::cout << "Success to Read Path File" << std::endl;

                std::string line;
                bool firstTime = true;
                while (std::getline(file, line)) {
                    std::vector<std::string> ss;
                    size_t pos = 0;
                    while ((pos = line.find('\t')) != std::string::npos) {
                        ss.push_back(line.substr(0, pos));
                        line.erase(0, pos + 1);
                    }
                    ss.push_back(line);

                    for (int j = 0; j < int(ss.size()); j++) {
                        temp.push_back(ss[j]);
                    }
                    if (firstTime) {
                        columnNumber = (int)ss.size();
                        firstTime = false;
                    }
                    totalLine++;
                }
                file.close();
            }
            else {
                std::cout << "Can not find the path file" << std::endl;
            }
        }
        catch (const std::exception& e) {
            std::cout << "Error in read data" << std::endl;
            std::cerr << e.what() << std::endl;
        }

        std::vector<std::vector<std::string>> result(totalLine, std::vector<std::string>(columnNumber));
        for (int i = 0; i < totalLine; i++) {
            for (int j = 0; j < columnNumber; j++) {
                result[i][j] = temp[i * columnNumber + j];
            }
        }

        std::vector<ContainerPath> tempPathSet;

        for (int i = 1; i < result.size(); i++) {
            ContainerPath ff;

            ff.SetRequestID(std::stoi(result[i][0]));
            ff.SetOriginPort(result[i][1]);
            ff.SetOriginTime(std::stoi(result[i][2]));
            ff.SetDestinationPort(result[i][3]);
            ff.SetDestinationTime(std::stoi(result[i][4]));
            ff.SetPathTime(std::stoi(result[i][5]));

            if (result[i][6] == "0" && result[i][7] == "0") {
                ff.SetTransshipPorts(std::vector<std::string>());
                ff.SetTransshipTime(std::vector<int>());
            }
            else {
                std::vector<std::string> trans_port = split(result[i][6], ",");
                ff.SetTransshipPorts(trans_port);

                std::vector<std::string> s_trans_time = split(result[i][7], ",");
                std::vector<int> trans_time(s_trans_time.size());
                for (int j = 0; j < s_trans_time.size(); j++) {
                    trans_time[j] = std::stoi(s_trans_time[j]);
                }
                ff.SetTransshipTime(trans_time);
            }

            ff.SetNumberofPath(std::stoi(result[i][8]));

            std::vector<std::string> port_path = split(result[i][9], ",");
            ff.SetPathPorts(port_path);

            int num_of_arcs = std::stoi(result[i][10]);
            ff.SetNumArcs(num_of_arcs);

            std::vector<int> arcIDs(num_of_arcs);
            std::vector<Arc> arcs(num_of_arcs);
            std::vector<std::string> s_arc;

            std::vector<std::string> s_arcIDs = split(result[i][11], ",");
            for (int j = 0; j < s_arcIDs.size(); j++) {
                arcIDs[j] = std::stoi(s_arcIDs[j]);
                arcs[j] = ST.GetArc(arcIDs[j]);
            }
            ff.SetArcIDs(arcIDs);
            ff.SetArcs(arcs);

            if (ff.GetDestinationTime() <= ST.GetT()) {
                tempPathSet.push_back(ff);
            }
        }

        vector<OD> W = ST.GetW();
        for (ContainerPath& path: tempPathSet)
        {
            int od_index = ST.GetODindex(path.GetOriginPort(), path.GetDestinationPort());
            if (W[od_index].laden_path_set.find(path) == W[od_index].laden_path_set.end()) {
                if (W[od_index].type == 0) {
                    P0.push_back(path);
                }
                else if (W[od_index].type == 1) {
                    P1.push_back(path);
                }
                else if (W[od_index].type == 2) {
                    P2.push_back(path);
                }
                else if (W[od_index].type == 3) {
                    P3.push_back(path);
                }
                else if (W[od_index].type == 4) {
                    P4.push_back(path);
                }
                else {
                    throw;
                }

                W[od_index].laden_path_set.insert(path);
                W[od_index].laden_paths.push_back(path);
                W[od_index].num_laden_paths++;

                PathSet.push_back(path);
            }
        }
        ST.SetW(W);
    }

    // 0). OD = OD = 1
    vector<ContainerPath>& GeneratePath::generate_paths_input_type0()
    {
        std::cout << "Type 0 : OD = One segment" << std::endl;

        vector<OD> W = ST.GetW();

        for (size_t od_index = 0; od_index < W.size(); od_index++)
        {
            OD tempOD = W[od_index];
            
            if (tempOD.type == 0)
            {
                for (size_t nn = 0; nn < ST.GetAv().size(); nn++)
                {
                    Arc tempArc = ST.GetAv()[nn];
                    Node origin = tempArc.GetTail();
                    Node destination = tempArc.GetHead();

                    if (origin.GetPort() == tempOD.GetOrigin()
                        && destination.GetPort() == tempOD.GetDestination())
                    {
                        ContainerPath path;
                        path.SetOriginPort(origin.GetPort());
                        path.SetDestinationPort(destination.GetPort());
                        path.SetOriginTime(origin.GetTime());
                        path.SetDestinationTime(destination.GetTime());
                        path.SetPathTime(path.GetDestinationTime() - path.GetOriginTime());
                        path.SetArcs(std::vector<Arc>({ tempArc }));
                        path.AddArcID2ArcIDs(tempArc.GetArcID());
                        path.SetNumArcs(1);
                        path.AddPort2PathPorts(origin.GetPort());
                        path.AddPort2PathPorts(destination.GetPort());

                        if (W[od_index].laden_path_set.find(path) == W[od_index].laden_path_set.end()) {
                            P0.push_back(path);

                            W[od_index].laden_path_set.insert(path);
                            W[od_index].laden_paths.push_back(path);
                            W[od_index].num_laden_paths++;

                            PathSet.push_back(path);
                            path.outputPath(fout);
                        }

                        //path.printPath();
                    }
                }
            }
        }

        ST.SetW(W);
        return P0;
    }

    vector<ContainerPath>& GeneratePath::generate_paths_input_type00()
    {
        std::cout << "Type 0 : OD = Direct Path" << std::endl;

        vector<OD> W = ST.GetW();

        // generate direct container path
        for (size_t r = 0; r < ST.GetR().size(); r++)
        {
            int round_trip_time = ST.GetR()[r].GetTransitTime();
            for (size_t p_o = 1; p_o <= ST.GetR()[r].GetNumPortCall(); p_o++)
            {
                for (size_t p_d = 1; p_d <= ST.GetR()[r].GetNumPortCall(); p_d++)
                {
                    std::string origin = ST.GetR()[r].GetPath()[p_o - 1].port;
                    std::string destination = ST.GetR()[r].GetPath()[p_d - 1].port;
                    int origin_call_time = ST.GetR()[r].GetPath()[p_o - 1].arrivalTime;
                    int destination_call_time = ST.GetR()[r].GetPath()[p_d - 1].arrivalTime;

                    if (origin == destination) {
                        continue;
                    }
                    // at least two rotation
                    if (p_o > p_d && ST.GetR()[r].GetPath().back().arrivalTime + round_trip_time > ST.GetT())
                    {
                        continue;
                    }

                    int od_index = ST.GetODindex(origin, destination);

                    ContainerPath path;
                    path.SetOriginPort(origin);
                    path.SetDestinationPort(destination);

                    if (p_o < p_d) {
                        path.SetOriginTime(origin_call_time);
                        path.SetDestinationTime(destination_call_time);
                        path.SetPathTime(path.GetDestinationTime() - path.GetOriginTime());

                        std::vector<Arc> tempArcSet;
                        std::vector<int> tempArcIDs;
                        std::vector<std::string> tempPathPorts = { origin };
                        for (size_t arc_call = p_o; arc_call < p_d; arc_call++)
                        {
                            Node tail_node = ST.GetNode((int)r + 1, 1, (int)arc_call);
                            Node head_node = ST.GetNode((int)r + 1, 1, (int)arc_call + 1);
                            int temp_arc_id = ST.GetArcID(tail_node.GetNodeID(), head_node.GetNodeID());
                            Arc temp_arc = ST.GetArc(temp_arc_id);
                            tempArcIDs.push_back(temp_arc_id);
                            tempArcSet.push_back(temp_arc);

                            tempPathPorts.push_back(head_node.GetPort());
                        }
                        path.SetArcs(tempArcSet);
                        path.SetArcIDs(tempArcIDs);
                        path.SetNumArcs((int)tempArcSet.size());
                        path.SetPathPorts(tempPathPorts);
                    }
                    else {
                        path.SetOriginTime(origin_call_time);
                        path.SetDestinationTime(destination_call_time + round_trip_time);
                        path.SetPathTime(path.GetDestinationTime() - path.GetOriginTime());

                        std::vector<Arc> tempArcSet;
                        std::vector<int> tempArcIDs;
                        std::vector<std::string> tempPathPorts = { origin };

                        //
                        for (size_t arc_call = p_o; arc_call < ST.GetR()[r].GetPath().size(); arc_call++)
                        {
                            Node tail_node = ST.GetNode((int)r + 1, 1, (int)arc_call);
                            Node head_node;
                            head_node = ST.GetNode((int)r + 1, 1, (int)arc_call + 1);
                            int temp_arc_id = ST.GetArcID(tail_node.GetNodeID(), head_node.GetNodeID());
                            Arc temp_arc = ST.GetArc(temp_arc_id);
                            tempArcIDs.push_back(temp_arc_id);
                            tempArcSet.push_back(temp_arc);
                            tempPathPorts.push_back(head_node.GetPort());
                        }
                        // 
                        for (size_t arc_call = 1; arc_call < p_d; arc_call++)
                        {
                            Node tail_node;
                            if (arc_call == 1) {
                                tail_node = ST.GetNode((int)r + 1, 1, (int)ST.GetR()[r].GetPath().size());
                            }
                            else {
                                tail_node = ST.GetNode((int)r + 1, 1 + (int)round(round_trip_time / 7), (int)arc_call);
                            }                 
                            Node head_node = ST.GetNode((int)r + 1, 1 + (int)round(round_trip_time / 7), (int)arc_call + 1);
                            int temp_arc_id = ST.GetArcID(tail_node.GetNodeID(), head_node.GetNodeID());
                            Arc temp_arc = ST.GetArc(temp_arc_id);
                            tempArcIDs.push_back(temp_arc_id);
                            tempArcSet.push_back(temp_arc);
                            tempPathPorts.push_back(head_node.GetPort());
                        }
                        path.SetArcs(tempArcSet);
                        path.SetArcIDs(tempArcIDs);
                        path.SetNumArcs((int)tempArcSet.size());
                        path.SetPathPorts(tempPathPorts);
                    }
                    //path.printPath();
                    if (check_path_feasiblility(path, ST)) {
                        vector<ContainerPath> multi_stage_paths = extend_from_one_to_multi_stage_paths(path);

                        for (ContainerPath& od_path : multi_stage_paths)
                        {
                            if (W[od_index].laden_path_set.find(od_path) == W[od_index].laden_path_set.end()) {
                                P0.push_back(od_path);

                                W[od_index].laden_path_set.insert(od_path);
                                W[od_index].laden_paths.push_back(od_path);
                                W[od_index].num_laden_paths++;

                                PathSet.push_back(od_path);
                                od_path.outputPath(fout);
                                fout.flush();

                                if (od_path.GetPathTime() < W[od_index].shortest_Travel_time)
                                {
                                    W[od_index].shortest_Travel_time = od_path.GetPathTime();
                                }
                                //od_path.printPath();
                            }
                        }                        
                    }
                }
            }
        }
        ST.SetW(W);
        return P0;
    }



    // 1). OD = TT
    // only this type od pairs need to calculate container paths with solver
    vector<ContainerPath>& GeneratePath::generate_paths_input_type1()
    {
        //string test_origin = "Aqabah";
        //string test_destination = "Chiwan";
        //OD od = ST.GetODbyString(test_origin, test_destination);
        //for (size_t o = 0; o < ST.GetN().size(); o++)
        //{
        //    Node origin = ST.GetN()[o];
        //    if (origin.GetRotation() > 1
        //        || origin.GetPort() != od.originPort
        //        || origin.GetCall() > ST.GetR()[origin.GetRouteID() - 1].num_port)
        //        continue;
        //    for (size_t d = 0; d < ST.GetN().size(); d++)
        //    {
        //        Node destination = ST.GetN()[d];
        //        if (origin.GetPort() == od.originPort
        //            && destination.GetPort() == od.destinationPort
        //            && origin.GetTime() >= od.earliest_pickupara_time
        //            && origin.GetTime() < destination.GetTime()
        //            && destination.GetTime() <= od.deadline_time
        //            && destination.GetTime() - origin.GetTime() < od.shortest_Travel_time + TurnOverTime)
        //        {
        //            //cout << origin.GetPort() << "==>" << destination.port << std::endl;
        //            vector<ContainerPath> temp_path_set = Get_node_pair_paths(origin, destination);
        //        }
        //    }
        //}


        std::cout << "Type 1 : OD = T/T" << std::endl;

        vector<OD> W = ST.GetW();

        for (size_t od_index = 0; od_index < W.size(); od_index++)
        {
            if (W[od_index].type == 1)
            {
                for (size_t o = 0; o < ST.GetN().size(); o++)
                {
                    Node origin = ST.GetN()[o];
                    if (origin.GetRotation() > 1
                        || origin.GetPort() != W[od_index].originPort
                        || origin.GetCall() > ST.GetR()[origin.GetRouteID() - 1].num_port)
                        continue;
                    for (size_t d = 0; d < ST.GetN().size(); d++)
                    {
                        Node destination = ST.GetN()[d];
                        if (origin.GetPort() == W[od_index].originPort
                            && destination.GetPort() == W[od_index].destinationPort
                            && origin.GetTime() >= W[od_index].earliest_pickup_time
                            && origin.GetTime() < destination.GetTime()                            
                            && destination.GetTime() <= W[od_index].deadline_time
                            && destination.GetTime() - origin.GetTime() < W[od_index].shortest_Travel_time + MaxTolerancePathTimeGap)
                        {
                            //cout << origin.GetPort() << "==>" << destination.port << std::endl;
                            vector<ContainerPath> temp_path_set = Get_node_pair_paths(origin, destination);

                            if (temp_path_set.empty())
                                continue;
                            else
                            {
                                temp_path_set = extend_multi_stage_paths(temp_path_set);
                                // add all paths in temp_path_set to paths Set Ps
                                for (const auto& tempPath : temp_path_set)
                                {
                                    // !there exits a error while push back the seventh path!
                                    if (check_path_feasiblility(tempPath, ST))
                                    {
                                        if (W[od_index].laden_path_set.find(tempPath) == W[od_index].laden_path_set.end()) {
                                            P1.push_back(tempPath);

                                            W[od_index].laden_path_set.insert(tempPath);
                                            W[od_index].laden_paths.push_back(tempPath);
                                            W[od_index].num_laden_paths++;

                                            PathSet.push_back(tempPath);
                                            tempPath.outputPath(fout);
                                            fout.flush();
                                        }

                                        // update the shortest departure time
                                        if (tempPath.GetPathTime() < W[od_index].shortest_Travel_time)
                                        {
                                            W[od_index].shortest_Travel_time = tempPath.GetPathTime();
                                        }

                                        // print Path
                                        //tempPath.printPath();
                                    }
                                    else {
                                        tempPath.printPath();
                                        //throw;
                                    }                                        
                                }
                            }
                        }
                    }
                }
            }
        }

        ST.SetW(W);
        return P1;
    }

    // 2). OD = OD / OD = OT / OD = TD
    vector<ContainerPath>& GeneratePath::generate_paths_input_type2()
    {
        std::cout << "Type 2 : OD = OD" << std::endl;

        vector<OD> W = ST.GetW();

        for (size_t i = 0; i < W.size(); i++)
        {
            if (W[i].type == 2)
            {
                for (auto x : connectOD(W[i]))
                {
                    int od_index = ST.GetODindex(x.GetOriginPort(), x.GetDestinationPort());
                    if (W[od_index].laden_path_set.find(x) == W[od_index].laden_path_set.end()) {
                        P2.push_back(x);

                        W[od_index].laden_path_set.insert(x);
                        W[od_index].laden_paths.push_back(x);
                        W[od_index].num_laden_paths++;
                        
                        PathSet.push_back(x);
                        x.outputPath(fout);
                        fout.flush();
                    }
                }
            }
        }
        ST.SetW(W);

        return P2;
    }

    // 3). OD = OT + TD
    vector<ContainerPath>& GeneratePath::generate_paths_input_type3()
    {
        std::cout << "Type 3 : OD = <OT + TD>" << std::endl;

        vector<OD> W = ST.GetW();

        for (size_t i = 0; i < W.size(); i++)
        {
            if (W[i].type == 3)
            {
                for (auto x : connectOD(W[i]))
                {
                    int od_index = ST.GetODindex(x.GetOriginPort(), x.GetDestinationPort());
                    if (W[od_index].laden_path_set.find(x) == W[od_index].laden_path_set.end()) {
                        P3.push_back(x);

                        W[od_index].laden_path_set.insert(x);
                        W[od_index].laden_paths.push_back(x);
                        W[od_index].num_laden_paths++;
                        
                        PathSet.push_back(x);
                        x.outputPath(fout);
                        fout.flush();
                    }
                }
            }
        }
        ST.SetW(W);
        return P3;
    }


    //vector<ContainerPath>& GeneratePath::generate_paths_input_type33()
    //{
    //    std::cout << "Type 3 : OD = <OT + TD>" << std::endl;

    //    vector<OD> W = ST.GetW();
    //    for (size_t left_p = 0; left_p < P0.size(); left_p++)
    //    {
    //        for (size_t right_p = 0; right_p < P0.size(); right_p++)
    //        {
    //            ContainerPath left_path = P0[left_p];
    //            ContainerPath right_path = P0[right_p];
    //            if (left_p == right_p) {
    //                continue;
    //            }
    //            if (right_path.GetTransshipPorts().empty() && left_path.GetTransshipPorts().empty()
    //                && right_path.GetOriginNode().GetRouteID() == left_path.GetDestinationNode().GetRouteID())
    //            {
    //                continue;
    //            }
    ////......
    ////......
    //        }
    //    }
    //    ST.SetW(W);
    //    return P3;
    //}

    // 4). OD = OT1 + T1T2 + T2D
    vector<ContainerPath>& GeneratePath::generate_paths_input_type4()
    {
        std::cout << "Type 4 : <OD = OT + TT + TD>" << std::endl;

        vector<OD> W = ST.GetW();

        for (size_t i = 0; i < W.size(); i++)
        {
            if (W[i].type == 4)
            {
                for (auto x : connectOD(W[i]))
                {
                    int od_index = ST.GetODindex(x.GetOriginPort(), x.GetDestinationPort());
                    if (W[od_index].laden_path_set.find(x) == W[od_index].laden_path_set.end()) {
                        W[od_index].laden_path_set.insert(x);
                        W[od_index].laden_paths.push_back(x);
                        W[od_index].num_laden_paths++;
                        P4.push_back(x);
                        PathSet.push_back(x);
                        x.outputPath(fout);
                        fout.flush();
                    }
                }
            }
        }
        ST.SetW(W);
        return P4;
    }


    // Step 1 : calcualte the shortest path for each od
    // Step 2 : delete <transship> path that length over 2 weeks than shortest path
    vector<ContainerPath>& GeneratePath::cut_long_paths() {
        vector<OD> W = ST.GetW();

        for (size_t i = 0; i < ST.GetW().size(); i++)
        {
            for (size_t j = 0; j < W[i].laden_paths.size(); j++)
            {
                if (W[i].shortest_Travel_time > W[i].laden_paths[j].GetPathTime())
                {
                    W[i].shortest_Travel_time = W[i].laden_paths[j].GetPathTime();
                }
            }
        }

        ST.SetW(W);

        vector<ContainerPath>newP;
        for (size_t p = 0; p < PathSet.size(); p++)
        {
            ContainerPath tempPath = PathSet[p];
            std::string origin = tempPath.GetOriginPort();
            std::string destination = tempPath.GetDestinationPort();
            OD od = ST.GetW()[ST.GetODindex(origin, destination)];

            if (!tempPath.GetTransshipPorts().size()
                || (tempPath.GetPathTime() < od.shortest_Travel_time + maxTravelTime
                && tempPath.GetPathTime() < MaxContainerPathLength))
            {
                newP.push_back(tempPath);
            }
        }

        PathSet = newP;

        return PathSet;
    }

    vector<ContainerPath> GeneratePath::connectOD(const OD& odod)
    {
        vector<OD> od_composition = odod.get_composition();

        std::cout << "==============================" << std::endl;
        for (size_t i = 0; i < od_composition.size(); i++)
        {
            if (i != 0)
                std::cout << " + ";
            std::cout << "<" << od_composition[i].originPort
                << "-" << od_composition[i].destinationPort << ">";
        }
        std::cout << " : " << std::endl;

        vector<ContainerPath> PP;
        vector<OD> W = ST.GetW();
        for (size_t i = 0; i < od_composition.size(); i++)
        {
            vector<ContainerPath> PosibilyP;
            OD od = od_composition[i];
            OD com_OD = W[ST.GetODindex(od.GetOrigin(), od.GetDestination())];
            if (com_OD.num_laden_paths)
            {
                for (size_t j = 0; j < com_OD.num_laden_paths; j++)
                {
                    ContainerPath right_path = com_OD.laden_paths[j];
                    if (i != 0)
                    {
                        for (size_t k = 0; k < PP.size(); k++)
                        {
                            ContainerPath left_path = PP[k];

                            if (right_path.GetTransshipPorts().empty() && left_path.GetTransshipPorts().empty()
                                && right_path.GetOriginNode().GetRouteID() == left_path.GetDestinationNode().GetRouteID())
                            {
                                continue;
                            }

                            if (odod.type == 2)
                            { 
                               if(
                                   (left_path.GetDestinationPort() == right_path.GetOriginPort()
                                       && left_path.GetDestinationTime() <= right_path.GetOriginTime()
                                       && left_path.GetDestinationNode().GetRouteID() != right_path.GetOriginNode().GetRouteID()
                                       && right_path.GetOriginTime() - left_path.GetDestinationTime() < maxTravelTime)
                                   || 
                                   left_path.GetDestinationNode() == right_path.GetOriginNode()
                                   )
                                {
                                    ContainerPath tempPath = connect_two_paths(left_path, right_path);
                                    // !there exits a error while push back the seventh path!
                                    if (check_path_feasiblility(tempPath, ST))
                                    {
                                        //tempPath.printPath();
                                        PosibilyP.push_back(tempPath);
                                    }
                                }
                            }
                            else if (odod.type == 3 || odod.type == 4)
                            {
                                if (
                                    (left_path.GetDestinationPort() == right_path.GetOriginPort()
                                    && left_path.GetDestinationTime() <= right_path.GetOriginTime()
                                    && left_path.GetDestinationNode().GetRouteID() != right_path.GetOriginNode().GetRouteID()
                                    && right_path.GetOriginTime() - left_path.GetDestinationTime() < maxTravelTime)
                                    ||
                                    left_path.GetDestinationNode() == right_path.GetOriginNode()
                                    )
                                {
                                    ContainerPath tempPath = connect_two_paths(left_path, right_path);

                                    if (check_path_feasiblility(tempPath, ST))
                                    {
                                        //tempPath.printPath();
                                        PosibilyP.push_back(tempPath);
                                    }
                                }
                            }
                            else
                            {
                                throw;
                            }
                        }
                    }
                    else
                    {
                        PosibilyP.push_back(com_OD.laden_paths[j]);
                    }
                }
            }
            else
            {
                std::cout << "exit an error : " << com_OD.num_laden_paths << std::endl;
                throw;
            }

            PP = PosibilyP;
        }

        return PP;
    }   

    ContainerPath GeneratePath::connect_two_paths(const ContainerPath& first_Path, const ContainerPath& second_Path)
    {
        ContainerPath tempPath = first_Path;
        tempPath.SetPathID(-1);

        int first_path_head_nodeID = ST.GetArc(first_Path.GetArcIDs().back()).GetHead().GetNodeID();
        int second_path_tail_nodeID = ST.GetArc(second_Path.GetArcIDs().front()).GetTail().GetNodeID();

        if (!tempPath.GetTransshipPorts().empty())
        {
            if (tempPath.GetTransshipPorts().size() == 1
                && tempPath.GetTransshipPorts().front() == "0")
            {
                //tempPath.transship_Port.clear();
                //tempPath.transship_time.clear();
                tempPath.SetTransshipPorts(std::vector<std::string>());
                tempPath.SetTransshipTime(std::vector<int>());
            }
        }
        
        // whether first path head != second path tail : add one transship arc
        if (first_path_head_nodeID != second_path_tail_nodeID) 
        {
            // exit one transshipment between different shipping path
            if (first_Path.GetDestinationNode().GetRouteID() != second_Path.GetOriginNode().GetRouteID()) 
            {
                int addtion_transship_arcID = ST.GetArcID(first_path_head_nodeID, second_path_tail_nodeID);
                tempPath.AddArcID2ArcIDs(addtion_transship_arcID);
                tempPath.AddArc2Arcs(ST.GetArc(addtion_transship_arcID));
                tempPath.AddPort2TransshipPorts(first_Path.GetDestinationPort());
                tempPath.AddTime2TransshipTime(second_Path.GetOriginTime() - first_Path.GetDestinationTime());
                tempPath.SetNumArcs(tempPath.GetNumArcs() + 1);
            }
            else
            {
                tempPath.SetPathID(0);
                throw;
            }
        }

        tempPath.SetDestinationTime(second_Path.GetDestinationTime());
        tempPath.SetDestinationPort(second_Path.GetDestinationPort());
        tempPath.SetPathTime( second_Path.GetDestinationTime() - first_Path.GetOriginTime());

        for (size_t i = 0; i < second_Path.GetArcIDs().size(); i++)
        {
            tempPath.AddArcID2ArcIDs(second_Path.GetArcIDs()[i]);
            tempPath.AddArc2Arcs(ST.GetArc(second_Path.GetArcIDs()[i]));
        }

        tempPath.SetNumArcs(tempPath.GetNumArcs() + second_Path.GetNumArcs());

        for (size_t i = 1; i < second_Path.GetPathPorts().size(); i++)
        {
            tempPath.AddPort2PathPorts(second_Path.GetPathPorts()[i]);
        }

        for (size_t i = 0; i < second_Path.GetTransshipPorts().size(); i++)
        {
            if (second_Path.GetTransshipPorts()[i] != "0")
            {
                tempPath.AddPort2TransshipPorts(second_Path.GetTransshipPorts()[i]);
                tempPath.AddTime2TransshipTime(second_Path.GetTransshipTime()[i]);
            }
        }

        // print new path
        //cout << "<" << first_Path.origin_port << "," << first_Path.GetDestinationPort() << ">"
        //    <<" with "
        //    << "<" << second_Path.origin_port << "," << second_Path.GetDestinationPort() << ">" << std::endl;
        //tempPath.printPath();

        // !there exits a error while push back the seventh path!
        if (!check_path_feasiblility(tempPath, ST))
        {
            first_Path.printPath();
            second_Path.printPath();
            tempPath.printPath();
            throw;
        }

        return tempPath;
    }

    // calculate "the shortest_Travel_time" for each "OD" in "W"
    // numbering the path in "PathSet"
    vector<ContainerPath>& GeneratePath::numbering_path()
    {
        vector<OD> W = ST.GetW();

        int shortest_time = 180;
        for (size_t i = 0; i < W.size(); i++)
        {
            int shortest_time = 180;
            for (size_t j = 0; j < W[i].num_laden_paths; j++)
            {
                if (W[i].laden_paths[j].GetPathTime() < shortest_time)
                {
                    shortest_time = W[i].laden_paths[j].GetPathTime();
                }
            }
            W[i].laden_paths.clear();
            W[i].num_laden_paths = 0;
            W[i].shortest_Travel_time = shortest_time;
        }

        sort(PathSet.begin(), PathSet.end());
        int num = 0;
        for (size_t i = 0; i < PathSet.size(); i++)
        {
            PathSet[i].SetPathID(++num);
            W[ST.GetODindex(PathSet[i].GetOriginPort(), PathSet[i].GetDestinationPort())].laden_paths.push_back(PathSet[i]);
            W[ST.GetODindex(PathSet[i].GetOriginPort(), PathSet[i].GetDestinationPort())].laden_pathIDs.push_back(PathSet[i].GetPathID());
            W[ST.GetODindex(PathSet[i].GetOriginPort(), PathSet[i].GetDestinationPort())].num_laden_paths++;
        }

        ST.SetW(W);
        return PathSet;
    }

    // generate request(OD) for each od port pairs from 1 to 180
    vector<OD>& GeneratePath::generate_all_request()
    {
        vector<OD> W = ST.GetW();

        for (size_t i = 0; i < W.size(); i++)
        {
            std::string origin = W[i].GetOrigin();
            std::string destination = W[i].GetDestination();
            if (ST.GetP().find(origin)->second.GetRegion() == ST.GetP().find(destination)->second.GetRegion()) {
                continue;
            }
            
            int earlist_setup_time = 1;
            int last_deadline_time = 1 + W[i].shortest_Travel_time + (MaxTolerancePathTimeGap - 1);
            int roundtrip = 1;
            while (earlist_setup_time <= 180)
            {
                ODset.push_back(OD(origin, destination
                    , earlist_setup_time, last_deadline_time, roundtrip));
                earlist_setup_time += 7;
                ++roundtrip;
                last_deadline_time = earlist_setup_time + W[i].shortest_Travel_time + (MaxTolerancePathTimeGap - 1) > 180 ? 
                    180 : earlist_setup_time + W[i].shortest_Travel_time + (MaxTolerancePathTimeGap - 1);
            }
        }
        for (size_t i = 0; i < ODset.size(); i++)
        {
            std::string origin = ODset[i].GetOrigin();
            std::string destination = ODset[i].GetDestination();
            int earlist_setup_time = ODset[i].earliest_pickup_time;
            int last_deadline_time = ODset[i].deadline_time;
            for (size_t j = 0; j < PathSet.size(); j++)
            {
                if (PathSet[j].GetOriginPort() == origin
                    && PathSet[j].GetDestinationPort() == destination
                    && PathSet[j].GetOriginTime() >= earlist_setup_time
                    && PathSet[j].GetDestinationTime() <= last_deadline_time)
                {
                    ODset[i].laden_pathIDs.push_back(PathSet[j].GetPathID());
                    ODset[i].laden_paths.push_back(PathSet[j]);
                    ODset[i].num_laden_paths++;
                }
                else if (PathSet[j].GetDestinationPort() == ODset[i].originPort
                    && PathSet[j].GetDestinationTime() <= ODset[i].earliest_pickup_time
                    && PathSet[j].GetDestinationTime() > ODset[i].earliest_pickup_time - 7)
                {
                    ODset[i].empty_pathIDs.push_back(PathSet[j].GetPathID());
                    ODset[i].empty_paths.push_back(PathSet[j]);
                    ODset[i].num_empty_paths++;
                }
            }
        }

        return ODset;
    }



    // ref Qiang Meng , Shuaian Wang : Liner ship fleet deployment with week-dependent container shipment demand. European Journal of Operational Research. 2012
    vector<ContainerPath> GeneratePath::Get_node_pair_paths(const Node& src, const Node& sink)
    {
        Node n_src = src;
        Node n_sink = sink;

        vector<Arc> A = ST.GetA();
        vector<Arc> At = ST.GetAt();
        vector<Arc> Av = ST.GetAv();
        vector<Node> N = ST.GetN();
        vector<ship_route> R = ST.GetR();
        map<string, Port> P = ST.GetP();

        // create an empty container routes Set
        vector<ContainerPath> Routes;

        try
        {
            IloEnv env_;

            IloModel model(env_);
            IloCplex cplex_(model);

            // add binary variables
            // xi : i ∈ A
            //      xi = 1 —— the arc A[i] was included in the path that between Node src and Node sink 
            //      xi = 0 —— otherwise
            //IloBoolVarArray x(env_, A.size());
            //model.add(x);
            IloBoolVarArray x = IloBoolVarArray(env_, A.size());
            model.add(x);

            for (size_t i = 0; i < A.size(); i++)
            {
                x[i] = IloBoolVar(env_);

                if (A[i].GetTail().GetTime() >= n_src.GetTime() && A[i].GetHead().GetTime() <= n_sink.GetTime())
                {
                    std::string name = "x" + i_to_s(A[i].GetArcID());
                    x[i].setName(name.c_str());
                }

                // exclude the outsIDe arcs which are not in [n_src.arrival_time, n_sink.arrival_time]
                if (A[i].GetTail().GetTime() < n_src.GetTime()
                    || A[i].GetHead().GetTime() > n_sink.GetTime())
                {
                    model.add(x[i] == 0);
                    std::string name = "x" + i_to_s(A[i].GetArcID());
                    x[i].setName(name.c_str());
                }
            }

            ////////////////
            // add infeasible cut based on known container path
            OD _od = ST.GetODbyString(src.GetPort(), sink.GetPort());
            for (int _p_index = 0; _p_index < _od.num_laden_paths; _p_index++)
            {
                if (_od.laden_paths[_p_index].GetOriginTime() >= n_src.GetTime()
                    && _od.laden_paths[_p_index].GetDestinationTime() <= n_sink.GetTime())
                {
                    IloExpr sum_x(env_);
                    for (size_t arc_index = 0; arc_index < _od.laden_paths[_p_index].GetArcs().size(); arc_index++)
                    {
                        int index_in_A = ST.GetAIndex(_od.laden_paths[_p_index].GetArcs()[arc_index]);
                        sum_x += x[index_in_A];
                    }
                    std::string constr_name = "Infeas-" + i_to_s(_p_index);
                    int num_arcs = (int)_od.laden_paths[_p_index].GetArcs().size();
                    model.add(sum_x <= num_arcs - 1).setName(constr_name.c_str());
                }
            }
            ///////////////



            int index = 0, index1 = 0, index2 = 0;
            std::string name;
            // add raint(4-13)

            // flow servation equation(4)
            // Node src    : OutDegree - InDegree = 1 
            // Node sink  : InDegree - OutDegree = 1
            // Other Node: InDegree = OutDegree
            index = 0;
            for (size_t m = 0; m < N.size(); m++)
            {
                if (N[m].GetTime() < n_src.GetTime() || N[m].GetTime() > n_sink.GetTime())
                {
                    continue;
                }
                int bm = 0;
                if (N[m] == n_src)
                {
                    bm = 1;
                }
                else if (N[m] == n_sink)
                {
                    bm = -1;
                }
                else
                {
                    bm = 0;
                }

                // please notice : here IloExpr create objective "env"
                IloExpr sum_xi(env_);
                for (size_t nn = 0; nn < A.size(); nn++)
                {
                    if (A[nn].GetTail().GetTime() >= n_src.GetTime()
                        && A[nn].GetHead().GetTime() <= n_sink.GetTime())
                    {
                        int coef = 0;
                        if (A[nn].GetTail() == N[m])
                        {
                            coef = 1;
                        }
                        else if (A[nn].GetHead() == N[m])
                        {
                            coef = -1;
                        }
                        else
                        {
                            coef = 0;
                        }
                        sum_xi += coef * x[nn];
                    }
                }
                name = "C(4-" + N[m].GetPort() + i_to_s(N[m].GetNodeID()) + ")";
                model.add(sum_xi == bm).setName(name.c_str());
            }

            // Rule 1 -> (6)
            // Rule 3 -> (7)
            index1 = 0, index2 = 0;
            for (size_t mn = 0; mn < A.size(); mn++)
            {
                if (A[mn].GetTail().GetTime() >= n_src.GetTime() && A[mn].GetHead().GetTime() <= n_sink.GetTime())
                {
                    if (A[mn].GetTail().GetPort() != A[mn].GetHead().GetPort())         // if mn is a Travel arc (in Av)
                    {
                        if (A[mn].GetHead().GetPort() == n_src.GetPort())
                        {
                            // the src port shouldn't be visited again between node src and node sink 
                            // eg : o-1-2-o-4-d
                            std::string name = "C(6-" + i_to_s(index1++) + ")";
                            model.add(x[mn] == 0).setName(name.c_str());
                        }
                        else if (A[mn].GetTail().GetPort() == n_sink.GetPort())
                        {
                            // the sink port shouldn't be visited again after arrived at sink port 
                            // eg: o-1-2-3-d-4-d
                            name = "C(7-" + i_to_s(index2++) + ")";
                            model.add(x[mn] == 0).setName(name.c_str());
                        }
                    }
                }
            }

            // Rule 2 -> (8) + (9)
            index1 = 0, index2 = 0;
            for (size_t _mn = 0; _mn < At.size(); _mn++)
            {
                if (At[_mn].GetTail().GetTime() < n_src.GetTime() || At[_mn].GetHead().GetTime() > n_sink.GetTime())
                    continue;

                int _index = ST.GetAIndex(At[_mn]);

                int r_m = A[_index].GetTail().GetRouteID();
                std::string p_m = A[_index].GetTail().GetPort();
                int r_n = A[_index].GetHead().GetRouteID();
                std::string p_n = A[_index].GetHead().GetPort();

                IloExpr sumXn(env_), sumXm(env_);

                // (8)
                // tail: at transship port and arrive at most once
                for (size_t i = 0; i < Av.size(); i++)
                {
                    if (Av[i].GetTail().GetTime() < n_src.GetTime() || Av[i].GetHead().GetTime() > n_sink.GetTime())
                        continue;

                    int index = ST.GetAIndex(Av[i]);
                    if (A[index].GetHead().GetRouteID() == r_m && A[index].GetHead().GetPort() == p_m)
                    {
                        sumXn += x[index];
                    }
                }
                name = "C(8-" + p_m + i_to_s(At[_mn].GetArcID()) + ")";
                model.add(sumXn <= M * (1 - x[_index]) + 1).setName(name.c_str());

                //(9)
                // head: at transship port and leave at most once
                for (size_t i = 0; i < Av.size(); i++)
                {
                    if (Av[i].GetTail().GetTime() < n_src.GetTime() || Av[i].GetHead().GetTime() > n_sink.GetTime())
                        continue;

                    int index = ST.GetAIndex(Av[i]);
                    if (A[index].GetTail().GetRouteID() == r_n && A[index].GetTail().GetPort() == p_n)
                    {
                        sumXm += x[index];
                    }
                }
                name = "C(9-" + p_n + i_to_s(At[_mn].GetArcID()) + ")";
                model.add(sumXm <= M * (1 - x[_index]) + 1).setName(name.c_str());
            }

            // (10)
            // the Travel arc was included in at most once
            index = 0;
            for (size_t _r = 0; _r < R.size(); _r++)
            {
                // Get the ShipRoute ID r ∈ R
                int r = R[_r].GetRouteID();
                for (size_t _i = 0; _i < R[_r].GetNumPort(); _i++)
                {
                    // Get the i ∈ Ir
                    //int i = R[_r].GetPath()[_r].ID;
                    int i = (int)_i + 1;

                    // for each (r, i):
                    IloExpr sumX(env_);
                    for (size_t mn = 0; mn < Av.size(); mn++)
                    {
                        if (Av[mn].GetTail().GetTime() < n_src.GetTime() || Av[mn].GetHead().GetTime() > n_sink.GetTime())
                            continue;

                        int _index = ST.GetAIndex(Av[mn]);
                        if (A[_index].GetTail().GetRouteID() == r && A[_index].GetHead().GetCall() == i)
                        {
                            sumX += x[_index];
                        }
                    }
                    name = "C(10-" + i_to_s(index++) + ")";
                    model.add(sumX <= 1).setName(name.c_str());
                }
            }

            //Rule 4 -> (11) (12)
            // arc and sink Node can't exist tranship
            // the trranship arc was included in at most once
            index1 = 0, index2 = 0;
            for (map<string, Port>::iterator it = P.begin(); it != P.end(); it++)
            {
                //(11)-> p = p_n^src
                if ((*it).second.GetPort() == n_src.GetPort())
                {
                    IloExpr sumX(env_);
                    for (size_t i = 0; i < At.size(); i++)
                    {
                        if (At[i].GetTail().GetTime() < n_src.GetTime() || At[i].GetHead().GetTime() > n_sink.GetTime())
                            continue;

                        int _index = ST.GetAIndex(At[i]);
                        if (A[_index].GetTail().GetPort() == (*it).second.GetPort())
                        {
                            sumX += x[_index];
                        }
                    }
                    name = "C(11-" + i_to_s(index1++) + ")";
                    model.add(sumX == 0).setName(name.c_str());
                }
                //(11)-> p = p_n^sink
                else if ((*it).second.GetPort() == n_sink.GetPort())
                {
                    IloExpr sumX(env_);
                    for (size_t i = 0; i < At.size(); i++)
                    {
                        if (At[i].GetTail().GetTime() < n_src.GetTime() || At[i].GetHead().GetTime() > n_sink.GetTime())
                            continue;

                        int index = ST.GetAIndex(At[i]);
                        if (A[index].GetTail().GetPort() == (*it).second.GetPort())
                        {
                            sumX += x[index];
                        }
                    }
                    name = "C(11-" + i_to_s(index1++) + ")";
                    model.add(sumX == 0).setName(name.c_str());
                }
                // (12)-> p!= p_n^src && p!= p_n^sink
                else
                {
                    IloExpr sumX(env_);
                    for (size_t i = 0; i < At.size(); i++)
                    {
                        if (At[i].GetTail().GetTime() < n_src.GetTime() || At[i].GetHead().GetTime() > n_sink.GetTime())
                            continue;

                        int index = ST.GetAIndex(At[i]);
                        if (A[index].GetTail().GetPort() == (*it).second.GetPort())
                        {
                            sumX += x[index];
                        }
                    }
                    name = "C(12-" + i_to_s(index2++) + ")";
                    model.add(sumX <= 1).setName(name.c_str());
                }
            }

            bool flag = true;
            int k = 0;
            while (flag)
            {
                // solve and output the model
                cplex_.setOut(env_.getNullStream());
                cplex_.setWarning(env_.getNullStream());
                name = "GeneratePaths/models/(" + n_src.GetPort() + "-" + i_to_s(n_src.GetNodeID()) + "--" + n_sink.GetPort() + "-" + i_to_s(n_sink.GetNodeID()) + "-0).lp";
                cplex_.exportModel(name.c_str());
                cplex_.solve();

                ContainerPath route;
                // add the selected arcs to the first container route 
                vector<Arc> arcs_in_container_path;
                if (cplex_.getStatus() == IloAlgorithm::Optimal)
                {
                    // get solution
                    IloExpr sumX(env_);
                    for (size_t mn = 0; mn < A.size(); mn++)
                    {
                        if (A[mn].GetTail().GetTime() < n_src.GetTime()
                            || A[mn].GetHead().GetTime() > n_sink.GetTime())
                            continue;

                        IloNum const tolerance = cplex_.getParam(IloCplex::Param::MIP::Tolerances::Integrality);
                        if (cplex_.getValue(x[mn]) >= 1 - tolerance)
                        {
                            arcs_in_container_path.push_back(A[mn]);
                            sumX += x[mn];
                        }
                    }

                    // resort the arcs by arrivaltime of tail Node
                    sort(arcs_in_container_path.begin(), arcs_in_container_path.end());

                    // calculate the cost of container route
                    double sum_cost = 0.0;
                    route.AddPort2PathPorts(arcs_in_container_path.front().GetTail().GetPort());
                    for (auto& x : arcs_in_container_path)
                    {
                        double Cmn = x.GetCost();
                        sum_cost += Cmn;
                        route.AddArcID2ArcIDs(x.GetArcID());
                        route.AddArc2Arcs(x);

                        // Get the transshipment arc
                        if (x.GetTail().GetPort() == x.GetHead().GetPort())
                        {
                            route.AddPort2TransshipPorts(x.GetTail().GetPort());
                            route.AddTime2TransshipTime(x.GetTravelTime());
                        }
                        else
                        {
                            route.AddPort2PathPorts(x.GetHead().GetPort());
                        }
                    }
                    route.SetNumArcs((int)arcs_in_container_path.size());
                    route.SetOriginPort(arcs_in_container_path.front().GetTail().GetPort());
                    route.SetOriginTime(arcs_in_container_path.front().GetTail().GetTime());
                    route.SetDestinationPort(arcs_in_container_path.back().GetHead().GetPort());
                    route.SetDestinationTime(arcs_in_container_path.back().GetHead().GetTime());
                    route.SetPathTime(route.GetDestinationTime() - route.GetOriginTime());

                    //route.printPath();

                    Routes.push_back(route);

                    // add new infeasible cut
                    int _index = 0;
                    int num = (int)(route.GetArcs().size());
                    model.add(sumX <= num - 1);
                }
                else
                {
                    flag = false;
                }
            }
        }       
        catch ( IloException& e)
        {
            std::cout << "Error code = "<< e.getMessage() << std::endl;
        }
        catch (...)
        {
            std::cout << "Exception during the optimization" << std::endl;
        }

        return Routes;
    }

    vector<ContainerPath> GeneratePath::extend_from_one_to_multi_stage_paths(const ContainerPath& Path)
    {
        vector<ContainerPath> aPs;
        ContainerPath tempPath = Path;
        tempPath.SetDestinationTime(ST.GetArc(Path.GetArcIDs().back()).GetHead().GetTime());
        tempPath.SetPathID(-1);
        aPs.push_back(tempPath);
            
        int temp_rotation = ST.GetArc(tempPath.GetArcIDs().back()).GetHead().GetRotation();
        int temp_routeID = ST.GetArc(tempPath.GetArcIDs().back()).GetHead().GetRouteID();
        int flag = 1;
        while (flag)
        {
            if (ST.GetNodeID_after_oneweek(tempPath.GetDestinationNode()) == -1) {
                flag = -1;
                break;
            }

                tempPath = Path_add_oneweek(tempPath);
                tempPath.SetPathID(-1);

                aPs.push_back(tempPath);

                // print new path
                //tempPath.printPath();

                temp_rotation = ST.GetArc(tempPath.GetArcIDs().front()).GetTail().GetRotation();
                temp_routeID = ST.GetArc(tempPath.GetArcIDs().front()).GetTail().GetRouteID();

                if (temp_rotation == ST.NR[temp_routeID - 1])
                    flag = 0;

                for (auto x : tempPath.GetArcIDs())
                {
                    temp_rotation = ST.GetArc(x).GetHead().GetRotation();
                    temp_routeID = ST.GetArc(x).GetHead().GetRouteID();

                    if (temp_rotation == ST.NR[temp_routeID - 1])
                        flag = 0;
                }
        }
        return aPs;
    }


    vector<ContainerPath> GeneratePath::extend_multi_stage_paths(std::vector<ContainerPath>& Ps)
    {
        vector<ContainerPath> aPs;
        int path_count = (int)aPs.size();
        for (size_t i = 0; i < Ps.size(); i++)
        {
            ContainerPath tempPath = Ps[i];
            tempPath.SetDestinationTime(ST.GetArc(Ps[i].GetArcIDs().back()).GetHead().GetTime());
            tempPath.SetPathID(++path_count);
            aPs.push_back(tempPath);
            int temp_rotation = ST.GetArc(tempPath.GetArcIDs().back()).GetHead().GetRotation();
            int temp_routeID = ST.GetArc(tempPath.GetArcIDs().back()).GetHead().GetRouteID();
            int flag = 1;
            while (flag)
            {
                ContainerPath nextPath = Path_add_oneweek(tempPath);
                if (nextPath == tempPath) {
                    flag = 0;
                    break;
                }
                else {
                    tempPath = nextPath;
                }

                tempPath.SetPathID(++path_count);
                
                aPs.push_back(tempPath);

                // print new path
                //tempPath.printPath();

                temp_rotation = ST.GetArc(tempPath.GetArcIDs().front()).GetTail().GetRotation();
                temp_routeID = ST.GetArc(tempPath.GetArcIDs().front()).GetTail().GetRouteID();

                if (temp_rotation == ST.NR[temp_routeID - 1])
                    flag = 0;

                for (auto x : tempPath.GetArcIDs())
                {
                    temp_rotation = ST.GetArc(x).GetHead().GetRotation();
                    temp_routeID = ST.GetArc(x).GetHead().GetRouteID();

                    if (temp_rotation == ST.NR[temp_routeID - 1])
                        flag = 0;
                }
            }
        }
        return aPs;
    }

    vector<ContainerPath>& GeneratePath::Get_paths_set(const container_paths& container_paths,  vector<ContainerPath>& Ps)
    {
        int path_count = 0;
        for (auto& temp_od_pair_paths :container_paths)
        {
            if (!temp_od_pair_paths.empty())
            {
                for ( auto& temp_node_pair_paths : temp_od_pair_paths)
                {
                    if(!temp_node_pair_paths.empty())
                    {
                        for (auto& temp_path : temp_node_pair_paths)
                        {
                            ContainerPath tempPath;
                            tempPath.SetPathID(++path_count);
                            tempPath.SetOriginPort(temp_path.GetArcs().front().GetTail().GetPort());
                            tempPath.SetDestinationPort(temp_path.GetArcs().back().GetHead().GetPort());
                            tempPath.SetOriginTime(temp_path.GetArcs().front().GetTail().GetTime());
                            tempPath.SetDestinationTime(temp_path.GetDestinationTime());
                            tempPath.SetPathTime(tempPath.GetDestinationTime() - tempPath.GetOriginTime());
                            tempPath.SetNumArcs((int)temp_path.GetArcIDs().size());
                            for ( auto& x : temp_path.GetArcs())
                            {
                                tempPath.AddArcID2ArcIDs(x.GetArcID());

                                if (x == temp_path.GetArcs().front())
                                    
                                    tempPath.AddPort2PathPorts(x.GetTail().GetPort());

                                // for the transshipment arc
                                if (x.GetTail().GetPort() == x.GetHead().GetPort())
                                {
                                    tempPath.AddPort2TransshipPorts(x.GetTail().GetPort());
                                    tempPath.AddTime2TransshipTime(x.GetTravelTime());
                                }
                                else
                                {
                                    tempPath.AddPort2PathPorts(x.GetHead().GetPort());
                                }
                            }
                            Ps.push_back(tempPath);
                        }
                    }
                }
            }
        }

        // print all paths
        std::cout << "Path ID	OriginPort	OriginTime	DestinationPort	DestinationTime	PathTime	TransshipPort	TransshipTime	PortPath_length	PortPath	Arcs_length	ArcsID" << std::endl;
        for (size_t i = 0; i < Ps.size(); i++)
        {
            Ps[i].printPath();
        }

        // output all paths
        std::string path = "../data/data3/output/";
        ofstream fout_ps(path + "Paths.txt");
        fout_ps << "Path ID	OriginPort	OriginTime	DestinationPort	DestinationTime	PathTime	TransshipPort	TransshipTime	PortPath_length	PortPath	Arcs_length	ArcsID" << std::endl;
        for (size_t i = 0; i < Ps.size(); i++)
        {
            fout_ps << Ps[i].GetPathID() << '\t'
                << Ps[i].GetOriginPort() << '\t'
                << Ps[i].GetOriginTime() << '\t'
                << Ps[i].GetDestinationPort() << '\t'
                << Ps[i].GetDestinationTime() << '\t'
                << Ps[i].GetPathTime() << '\t';

            if (Ps[i].GetTransshipPorts().empty())
                fout_ps << 0 << '\t' << 0 << '\t';
            else
            {
                for (size_t i = 0; i < Ps[i].GetTransshipPorts().size(); i++)
                {
                    if (i == 0)
                        fout_ps << Ps[i].GetTransshipPorts()[i];
                    else
                        fout_ps << "," << Ps[i].GetTransshipPorts()[i];
                }
                fout_ps << '\t';

                for (size_t i = 0; i < Ps[i].GetTransshipTime().size(); i++)
                {
                    if (i == 0)
                        fout_ps << Ps[i].GetTransshipTime()[i];
                    else
                        fout_ps << "," << Ps[i].GetTransshipTime()[i];
                }
                fout_ps << '\t';
            }
            fout_ps << Ps[i].GetPathPorts().size() << '\t';

            for (size_t i = 0; i < Ps[i].GetPathPorts().size(); i++)
            {
                if (i == 0)
                    fout_ps << Ps[i].GetPathPorts()[i];
                else
                    fout_ps << "," << Ps[i].GetPathPorts()[i];
            }
            fout_ps << '\t';

            fout_ps << Ps[i].GetNumArcs() << '\t';

            for (size_t i = 0; i < Ps[i].GetArcIDs().size(); i++)
            {
                if (i == 0)
                    fout_ps << Ps[i].GetArcIDs()[i];
                else
                    fout_ps << "," << Ps[i].GetArcIDs()[i];
            }
            fout_ps << std::endl;
        }

        return Ps;
    }

    ContainerPath& GeneratePath::Path_add_oneweek(ContainerPath& path)
    {
        ContainerPath tempPath = path;

        path.SetOriginTime(path.GetOriginTime() + 7 * ST.day_length);
        path.SetDestinationTime(path.GetDestinationTime() + 7 * ST.day_length);
        vector<int> path_arcIDs = vector<int>(path.GetArcIDs());
        vector<Arc> path_arcs = vector<Arc>(path.GetArcs());
        for (size_t i = 0; i < path.GetArcIDs().size(); i++)
        {
            int now_arcID = path.GetArcIDs()[i];
            path_arcIDs[i] = ST.GetArcID_after_oneweek(now_arcID);
            if (path_arcIDs[i] == -1) {
                path = tempPath;
                return path;
            }
            path_arcs[i] = ST.GetArc(path_arcIDs[i]);
        }
        path.SetArcIDs(path_arcIDs);
        path.SetArcs(path_arcs);
        return path;
    }
};