#ifndef _READDATA_H_
#define _READDATA_H_

#include <vector>
#include <fstream>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <string>
#include <map>

#include "space_time_network.h"
#include "input_data.h"

#include "InputData.h"
#include "Setting.h"
#include "GenerateParameter.h"
#include "Node.h"
#include "ContainerPath.h"
#include "Port.h"

namespace fleetdeployment
{

class  ReadData : public Setting {
private:
    InputData& inputdata;
    const int timeHorizon;
    const std::string filePath;

public:
    ReadData(const std::string& path, InputData& inputdata, int timeHorizon)
        : inputdata(inputdata), timeHorizon(timeHorizon), filePath(path) {

        frame();

        if (WhetherPrintDataStatus) {
            this->inputdata.ShowStatus();
        }
    }

    ReadData(const std::string& path, InputData& inputdata) : inputdata(inputdata), timeHorizon(180), filePath(path) {}

private:
    void frame() {
        readShipRoute();
        readNode();
        readTravelArc();
        readTransshipArc();
        readPath();
        readVesselPath();
        readLadenPath();
        //readEmpty();
        readPort();
        readVessel();
        readRequest();
        readRange();
    }

    std::vector<std::vector<std::string>> readToString(const std::string& filename) {
        std::vector<std::string> temp;
        int totalLine = 0;
        int columnNumber = 0;
        try {
            std::string encoding = "GBK";
            std::ifstream file(filename);

            if (file.is_open()) {
                if (WhetherPrintFileLog) {
                    std::cout << "Success to Read Range File" << std::endl;
                }

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
                std::cout << "Can not find the range file" << std::endl;
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

        return result;
    }

    void readRange() {
        std::vector<std::vector<std::string>> result = readToString(filePath + "Range.txt");

        std::map<std::string, ODRange> rangeMap;
        for (int i = 1; i < result.size(); i++) {
            ODRange ff(std::stoi(result[i][0]),
                std::stoi(result[i][1]),
                std::stoi(result[i][2]),
                std::stoi(result[i][3]),
                std::stoi(result[i][4]),
                std::stoi(result[i][5]));
            std::string key = result[i][0] + result[i][1];
            rangeMap.insert(std::make_pair(key, ff));
        }
        inputdata.SetGroupRangeMap(rangeMap);
    }

    void readPath() {
        std::vector<std::string> temp;
        int totalLine = 0;
        int columnNumber = 0;
        try {
            std::string encoding = "GBK";
            std::ifstream file(filePath + "Path.txt");

            if (file.is_open()) {
                if (WhetherPrintFileLog) {
                    std::cout << "Success to Read Path File" << std::endl;
                }

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

        std::vector<ContainerPath> PathSet;

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
            std::vector<std::string> s_arc;

            std::vector<std::string> s_arcIDs = split(result[i][11], ",");
            for (int j = 0; j < s_arcIDs.size(); j++) {
                arcIDs[j] = std::stoi(s_arcIDs[j]);
            }
            ff.SetArcIDs(arcIDs);

            if (ff.GetDestinationTime() <= timeHorizon) {
                PathSet.push_back(ff);
            }
        }

        inputdata.SetPathSet(PathSet);
    }

    void readShipRoute() {
            std::vector<std::string> temp;
            int totalLine = 0;
            int columnNumber = 0;
            try {
                std::string encoding = "GBK";
                std::ifstream file(filePath + "shipingroute.txt");

                if (file.is_open()) {
                    if (Setting::WhetherPrintFileLog) {
                        std::cout << "Success to Read Ship Route File" << std::endl;
                    }

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
                            columnNumber = int(ss.size());
                            firstTime = false;
                        }
                        totalLine++;
                    }
                    file.close();
                }
                else {
                    std::cout << "Can not find the shipping route file" << std::endl;
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

            std::vector<ship_route> request;

            for (int i = 1; i < result.size(); i++) {
                ship_route ff;

                ff.SetRouteID(std::stoi(result[i][0]));
                ff.SetNumberofPorts(std::stoi(result[i][1]));
                ff.SetNumberofCall(std::stoi(result[i][3]));

                std::vector<std::string> route_ports = split(result[i][2], ",");
                std::vector<std::string> port_calls = split(result[i][4], ",");
                std::vector<std::string> S_time_calls = split(result[i][5], ",");

                std::vector<int> time_calls(S_time_calls.size());
                for (int t = 0; t < S_time_calls.size(); t++) {
                    time_calls[t] = std::stoi(S_time_calls[t]);
                }

                ff.SetPorts(route_ports);
                ff.SetPortsofCall(port_calls);
                ff.SetTimePointsOfCall(time_calls);

                request.push_back(ff);
            }

            inputdata.SetShipRoute(request);
        }

    void readRequest() {
            std::vector<std::string> temp;
            int totalLine = 0;
            int columnNumber = 0;
            try {
                std::string encoding = "GBK";
                std::ifstream file(filePath + "Request.txt");

                if (file.is_open()) {
                    if (Setting::WhetherPrintFileLog) {
                        std::cout << "Success to Read Request File" << std::endl;
                    }

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
                            columnNumber = int(ss.size());
                            firstTime = false;
                        }
                        totalLine++;
                    }
                    file.close();
                }
                else {
                    std::cout << "Can not find the Request file" << std::endl;
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

            std::vector<Request> request;

            for (int i = 1; i < result.size(); i++) {
                Request ff;

                ff.SetRequestID(std::stoi(result[i][0]));
                ff.SetOriginPort(result[i][1]);
                ff.SetDestinationPort(result[i][2]);
                ff.SetW_i_Earliest(std::stoi(result[i][3]));
                ff.SetLatestDestinationTime(std::stoi(result[i][4]));
                ff.SetNumberOfLadenPath(std::stoi(result[i][6]));
                ff.SetNumberOfEmptyPath(std::stoi(result[i][8]));

                if (ff.GetLatestDestinationTime() >= timeHorizon) {
                    continue;
                }

                std::vector<std::string> s_laden_paths = split(result[i][5], ",");
                std::vector<int> laden_paths(s_laden_paths.size());
                std::vector<int> laden_path_indexes(s_laden_paths.size());
                for (int j = 0; j < s_laden_paths.size(); j++) {
                    laden_paths[j] = std::stoi(s_laden_paths[j]);

                    if (ff.GetNumberOfLadenPath() != 0) {
                        int flag = 0;
                        for (int k = 0; k < inputdata.GetPathSet().size(); k++) {
                            if (laden_paths[j] == inputdata.GetPathSet()[k].GetRequestID()) {
                                laden_path_indexes[j] = k;
                                flag = 1;
                                break;
                            }
                        }

                        if (flag == 0) {
                            std::cout << "Error in finding laden path" << std::endl;
                        }
                    }
                }
                if (ff.GetNumberOfLadenPath() != 0) {

                ff.SetLadenPaths(laden_paths);
                ff.SetLadenPathIndexes(laden_path_indexes);
                }
                else {
                    ff.SetLadenPaths({});
                    ff.SetLadenPathIndexes({});
                }

                std::vector<std::string> s_empty_paths = split(result[i][7], ",");
                std::vector<int> empty_paths(ff.GetNumberOfEmptyPath());
                std::vector<int> empty_path_indexes(ff.GetNumberOfEmptyPath());
                for (int j = 0; j < ff.GetNumberOfEmptyPath(); j++) {
                    empty_paths[j] = std::stoi(s_empty_paths[j]);

                    if (empty_paths[0] != 0) {
                        int flag = 0;
                        for (int k = 0; k < inputdata.GetPathSet().size(); k++) {
                            if (empty_paths[j] == inputdata.GetPathSet()[k].GetRequestID()) {
                                empty_path_indexes[j] = k;
                                flag = 1;
                                break;
                            }
                        }

                        if (flag == 0) {
                            std::cout << "Error in finding laden path" << std::endl;
                        }
                    }
                }
                ff.SetEmptyPaths(empty_paths);
                ff.SetEmptyPathIndexes(empty_path_indexes);

                int groupO = 0;
                int groupD = 0;
                for (const Port& pp : inputdata.GetPortSet()) {
                    if (pp.GetPort() == ff.GetOriginPort()) {
                        groupO =pp.GetGroup();
                    }

                    if (pp.GetPort() == ff.GetDestinationPort()) {
                        groupD =pp.GetGroup();
                    }
                }
                ff.SetOriginGroup(groupO);
                ff.SetDestinationGroup(groupD);

                if (ff.GetLatestDestinationTime() <= timeHorizon && ff.GetNumberOfLadenPath() != 0) {
                    request.push_back(ff);
                }
            }

            this->inputdata.SetRequest(request);
        }

    void readVessel() {
            std::vector<std::string> temp;
            int totalLine = 0;
            int columnNumber = 0;
            try {
                std::string encoding = "GBK";
                std::ifstream file(filePath + "Vessel.txt");

                if (file.is_open()) {
                    if (Setting::WhetherPrintFileLog) {
                        std::cout << "Success to Read Vessel File" << std::endl;
                    }

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
                            columnNumber = int(ss.size());
                            firstTime = false;
                        }
                        totalLine++;
                    }
                    file.close();
                }
                else {
                    std::cout << "Can not find the vessel file" << std::endl;
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

            std::vector<Vessel> VesselSet;

            for (int i = 1; i < result.size(); i++) {
                Vessel ff;

                ff.SetVesselID(std::stoi(result[i][0]));
                ff.SetCapacity(std::stoi(result[i][1]));
                ff.SetOperatingCost(std::stod(result[i][2]) * 1000000.0);
                ff.SetCurrentRouteID(std::stoi(result[i][3]));
                ff.SetMaxVessels(std::stoi(result[i][4]));

                VesselSet.push_back(ff);
            }

            inputdata.SetVessel(VesselSet);
     }

    void readPort() {
        std::vector<std::string> temp;
        int totalLine = 0;
        int columnNumber = 0;
        try {
            std::string encoding = "GBK";
            std::ifstream file(filePath + "Port.txt");

            if (file.is_open()) {
                if (WhetherPrintFileLog) {
                    std::cout << "Success to Read Port" << std::endl;
                }

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
                        columnNumber = int(ss.size());
                        firstTime = false;
                    }
                    totalLine++;
                }
                file.close();
            }
            else {
                std::cout << "Can not find the port file" << std::endl;
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

        std::vector<Port> port;

        for (int i = 1; i < result.size(); i++) {
            Port ff;

            ff.SetID(std::stoi(result[i][0]));
            ff.SetPort(result[i][1]);
            ff.SetWhetherTrans(std::stoi(result[i][2]));
            ff.SetGroup(std::stoi(result[i][3]));

            port.push_back(ff);
        }

        inputdata.SetPortSet(port);
    }

    void readEmpty() {
        std::vector<std::string> temp;
        int totalLine = 0;
        int columnNumber = 0;
        try {
            std::string encoding = "GBK";
            std::ifstream file(filePath + "EmptyPaths.txt");

            if (file.is_open()) {
                if (WhetherPrintFileLog) {
                    std::cout << "Success to Read EmptyPaths File" << std::endl;
                }

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
                        columnNumber = int(ss.size());
                        firstTime = false;
                    }
                    totalLine++;
                }
                file.close();
            }
            else {
                std::cout << "Can not find the EmptyPaths file" << std::endl;
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

        std::vector<EmptyPath> emptyPathSet;

        for (int i = 1; i < result.size(); i++) {
            EmptyPath ff;

            ff.SetRequestID(std::stoi(result[i][0]));
            ff.SetOriginPort(result[i][1]);
            ff.SetOriginTime(std::stoi(result[i][2]));

            std::vector<std::string> s_empty_paths = split(result[i][3], ",");
            std::vector<int> empty_paths(s_empty_paths.size());
            for (int k = 0; k < s_empty_paths.size(); k++) {
                empty_paths[k] = std::stoi(s_empty_paths[k]);
            }
            ff.SetPathID(empty_paths);

            int index = 0;
            // read empty must after read laden
            for (LadenPath ll : inputdata.GetLadenPath()) {
                if (ll.GetRequestID() == ff.GetRequestID()) {
                    index++;
                }
            }

            if (index > 0) {
                emptyPathSet.push_back(ff);
            }
        }
        inputdata.SetEmptyPaths(emptyPathSet);
    }

    void readLadenPath() {
        std::vector<std::string> temp;
        int totalLine = 0;
        int columnNumber = 0;
        try {
            std::string encoding = "GBK";
            std::ifstream file(filePath + "LadenPaths.txt");

            if (file.is_open()) {
                if (WhetherPrintFileLog) {
                    std::cout << "Success to Read LadenPaths File" << std::endl;
                }

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
                        columnNumber = int(ss.size());
                        firstTime = false;
                    }
                    totalLine++;
                }
                file.close();
            }
            else {
                std::cout << "Can not find the LadenPaths file" << std::endl;
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

        std::vector<LadenPath> ladenPath;

        for (int i = 1; i < result.size(); i++) {
            LadenPath ff;

            ff.SetRequestID(std::stoi(result[i][0]));
            ff.SetOriginPort(result[i][1]);
            ff.SetOriginTime(std::stoi(result[i][2]));
            ff.SetDestinationPort(result[i][3]);
            ff.SetRoundTrip(std::stoi(result[i][4]));
            ff.SetWiEarlist(std::stoi(result[i][5]));
            ff.SetArrivalTimeToDestination(std::stoi(result[i][6]));
            ff.SetPathTime(std::stoi(result[i][7]));

            if (result[i][8] == "0" && result[i][9] == "0") {
                ff.SetTransshipPorts({});
                ff.SetTransshipmentTime({});
            }
            else {
                std::vector<std::string> transship_port = split(result[i][8], ",");
                std::vector<std::string> s_transship_time = split(result[i][9], ",");
                std::vector<int> transship_time(s_transship_time.size());
                for (int j = 0; j < transship_port.size(); j++) {
                    transship_time[j] = std::stoi(s_transship_time[j]);
                }
                ff.SetTransshipPorts(transship_port);
                ff.SetTransshipmentTime(transship_time);   
            }

            ff.SetPathID(std::stoi(result[i][10]));

            std::vector<std::string> portPath = split(result[i][11], ",");
            ff.SetPortPath(portPath);

            std::vector<std::string> s_arcIDs = split(result[i][12], ",");
            std::vector<int> arcIDs(s_arcIDs.size());
            for (int j = 0; j < s_arcIDs.size(); j++) {
                arcIDs[j] = std::stoi(s_arcIDs[j]);
            }
            ff.SetArcIDs(arcIDs);

            if (ff.GetArrivalTimeToDestination() < timeHorizon) {
                ladenPath.push_back(ff);
            }
        }
        inputdata.SetLadenPath(ladenPath);
    }

    void readVesselPath() {
        std::vector<std::string> temp;
        int totalLine = 0;
        int columnNumber = 0;
        try {
            std::string encoding = "GBK";
            std::ifstream file(filePath + "VesselPaths.txt");

            if (file.is_open()) {
                if (WhetherPrintFileLog) {
                    std::cout << "Success to Read VesselPaths File" << std::endl;
                }

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
                        columnNumber = int(ss.size());
                        firstTime = false;
                    }
                    totalLine++;
                }
                file.close();
            }
            else {
                std::cout << "Can not find the VesselPath file" << std::endl;
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

        std::vector<VesselPath> vesselPath;
        for (int i = 1; i < result.size(); i++) {
            VesselPath ff;

            ff.SetPathID(std::stoi(result[i][0]));
            ff.SetRouteID(std::stoi(result[i][1]));
            ff.SetNumArcs(std::stoi(result[i][2]));
            ff.SetOriginTime(std::stoi(result[i][4]));
            ff.SetDestinationTime(std::stoi(result[i][5]));
            ff.SetPathTime(ff.GetDestinationTime() - ff.GetOriginTime());

            std::vector<int> arcIDs(ff.GetNumArcs());
            std::vector<std::string> s_arcIDs = split(result[i][3], ",");
            for (int j = 0; j < s_arcIDs.size(); j++) {
                arcIDs[j] = std::stoi(s_arcIDs[j]);
            }
            ff.SetArcIDs(arcIDs);

            int index = 0;
            for (TravelArc tt : inputdata.GetTravelArcs()) {
                if (tt.GetArcID() == ff.GetArcIDs()[ff.GetNumArcs() - 1]) {
                    index = index + 1;
                }
            }
            if (index > 0) {
                vesselPath.push_back(ff);
            }
        }
        inputdata.SetVesselPath(vesselPath);
    }

    void readTransshipArc() {
        std::vector<std::string> temp;
        int totalLine = 0;
        int columnNumber = 0;
        try {
            std::string encoding = "GBK";
            std::ifstream file(filePath + "TransshipArcs.txt");

            if (file.is_open()) {
                if (WhetherPrintFileLog) {
                    std::cout << "Success to Read TransshipArc File" << std::endl;
                }

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
                        columnNumber = int(ss.size());
                        firstTime = false;
                    }
                    totalLine++;
                }
                file.close();
            }
            else {
                std::cout << "Can not find the TransshipArc file" << std::endl;
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

        std::vector<TransshipArc> transshipArcs;
        for (int i = 1; i < result.size(); i++) {
            TransshipArc ff;
            ff.SetArcID(std::stoi(result[i][0]));
            ff.SetPort(result[i][1]);
            ff.SetOriginNodeID(std::stoi(result[i][2]));
            ff.SetOriginTime(std::stoi(result[i][3]));
            ff.SetTransshipTime(std::stoi(result[i][4]));
            ff.SetDestination_node_ID(std::stoi(result[i][5]));
            ff.SetDestinationTime(std::stoi(result[i][6]));
            ff.SetFromRoute(std::stoi(result[i][7]));
            ff.SetToRoute(std::stoi(result[i][8]));

            if (ff.GetDestinationTime() < timeHorizon) {
                transshipArcs.push_back(ff);
            }
        }
        inputdata.SetTransshipArc(transshipArcs);
    }

    void readTravelArc() {
        std::vector<std::string> temp;
        int totalLine = 0;
        int columnNumber = 0;
        try {
            std::string encoding = "GBK";
            std::ifstream file(filePath + "TravelArcs.txt");

            if (file.is_open()) {
                if (WhetherPrintFileLog) {
                    std::cout << "Success to Read TravelArc File" << std::endl;
                }

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
                        columnNumber = int(ss.size());
                        firstTime = false;
                    }
                    totalLine++;
                }
                file.close();
            }
            else {
                std::cout << "Can not find the TravelArc file" << std::endl;
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

        std::vector<TravelArc> TravelArcSet;
        for (int i = 1; i < result.size(); i++) {
            TravelArc ff;

            ff.SetArcID(std::stoi(result[i][0]));
            ff.SetRoute(std::stoi(result[i][1]));
            ff.Setorigin_node_ID(std::stoi(result[i][2]));
            ff.SetOriginCall(std::stoi(result[i][3]));
            ff.SetOriginPort(result[i][4]);
            ff.SetOriginTime(std::stoi(result[i][6]));
            ff.SetTravelTime(std::stoi(result[i][7]));
            ff.SetDestination_node_ID(std::stoi(result[i][8]));
            ff.SetDestination_Call(std::stoi(result[i][9]));
            ff.SetDestination_Port(result[i][10]);
            ff.SetDestinationTime(std::stoi(result[i][11]));

            ship_route r = inputdata.GetShipRouteSet()[(ff.GetRouteIndex())];
            int index = r.GetCallIndexOfPort(ff.GetOriginPort());
            int round_trip = (ff.GetOriginTime() - r.GetTimePointsOfCall()[index]) / 7 + 1;
            ff.SetRound_Trip(round_trip);

            if (r.GetTimePointsOfCall()[r.GetNumberofCall() - 1] + 7 * (ff.GetRound_Trip() - 1) <= timeHorizon) {
                TravelArcSet.push_back(ff);
            }
        }
        inputdata.SetTravelArcs(TravelArcSet);
    }

    void readNode() {
        std::vector<Node> node;
        std::vector<std::string> temp;
        int totalLine = 0;
        int columnNumber = 0;
        try {
            std::string encoding = "GBK";
            std::ifstream file(filePath + "nodes.txt");

            if (file.is_open()) {
                if (WhetherPrintFileLog) {
                    std::cout << "Success to Read Node File" << std::endl;
                }

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
                        columnNumber = int(ss.size());
                        firstTime = false;
                    }
                    totalLine++;
                }
                file.close();
            }
            else {
                std::cout << "Can not find the nodes file" << std::endl;
            }
        }
        catch (const std::exception& e) {
            std::cout << "Error in read data" << std::endl;
            std::cerr << e.what() << std::endl;
        }

        std::vector<std::vector<std::string>> result(totalLine, std::vector<std::string>(columnNumber));
        for (int i = 0; i < totalLine; i++) {
            for (int j = 0; j < columnNumber; j++) {
                result[i][j] = temp[(i * columnNumber) + j];
            }
        }

        for (int i = 1; i < result.size(); i++) {
            Node ff;
            ff.SetNodeID(std::stoi(result[i][0]));
            ff.SetRoute(std::stoi(result[i][1]));
            ff.SetCall(std::stoi(result[i][2]));
            ff.SetRoundTrip(std::stoi(result[i][4]));
            ff.SetTime(std::stoi(result[i][5]));
            ff.SetPortString(result[i][3]);

            if (ff.GetTime() < timeHorizon) {
                node.push_back(ff);
            }
        }
        inputdata.SetNode(node);
    }

public:

    void _output_input_data(string path, Data& data);
    void output_ship_route(string path, Data& data);

    Data& read_ship_route(const string& filename, Data& data);

    vector<std::vector<Node>> read_sequence(string path);

    vector<ContainerPath>& read_paths(const string& filename, vector<ContainerPath>& Ps);

    Node string_to_Node(string& node_string);

    ContainerPath sequence_to_path(std::vector<Node>& sequence);

    vector<ContainerPath>& sequences_to_Paths(ST_network& ST, vector<std::vector<Node>>& sequence_vector, vector<ContainerPath>& Ps);
};
}

# endif _READDATA_H_