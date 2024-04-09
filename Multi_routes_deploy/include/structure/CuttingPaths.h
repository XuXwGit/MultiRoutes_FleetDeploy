#ifndef CUTTING_PATHS_H_
#define CUTTING_PATHS_H_

#include <vector>
#include <string>
#include <algorithm>

#include "InputData.h"  // Replace with your actual InputData class 
#include "Parameter.h"  // Replace with your actual Parameter class 


namespace fleetdeployment
{


    class  CuttingPaths {
    public:
        CuttingPaths(InputData& input, Parameter& para) : input_(input), para_(para) {
            Frame();
        }

    private:
        InputData& input_;
        Parameter& para_;

        void Frame() {
            ReduceLongEmptyPath();

            ReduceTransshippingPaths();

            ReduceEmptyPaths();

            CutRequests();

            RecalculatePaths();
        }

        void ReduceLongEmptyPath() {

            std::vector<Request> newRequestSet(input_.GetRequests().size());
            for (int i = 0; i < input_.GetRequests().size(); i++) {
                Request request = input_.GetRequests()[i];
                int flag = 0;

                // 0/1
                if (request.GetNumberOfEmptyPath() <= 1) {
                    newRequestSet[i] = request;
                    continue;
                }

                // 仅保留用时最短的EmptyPath
                std::vector<int> newEmptyPaths(1);
                std::vector<int> newEmptyPathIndexes(1);
                newEmptyPaths[0] = request.GetEmptyPaths()[0];
                newEmptyPathIndexes[0] = request.GetEmptyPathIndexes()[0];

                int shortestPathIndex = request.GetEmptyPathIndexes()[0];
                for (int j = 1; j < request.GetEmptyPathIndexes().size(); j++) {
                    int index = request.GetEmptyPathIndexes()[j];
                    if (input_.GetPathSet()[index].GetPathTime() <= input_.GetPathSet()[shortestPathIndex].GetPathTime()) {
                        newEmptyPaths[0] = request.GetEmptyPaths()[j];
                        newEmptyPathIndexes[0] = index;
                        flag = 1;
                    }
                }

                if (flag == 1) {
                    std::vector<int> newEmptyID(1);
                    std::vector<int> newEmptyIndex(1);
                    for (int k = 0; k < 1; k++) {
                        newEmptyID[k] = newEmptyPaths[k];
                        newEmptyIndex[k] = newEmptyPathIndexes[k];
                        request.SetEmptyPaths(newEmptyID);
                        request.SetEmptyPathIndexes(newEmptyIndex);
                    }
                    request.SetNumberOfEmptyPath(1);
                }
                newRequestSet[i] = request;
            }

            input_.SetRequest(newRequestSet);
        }

        // 仅保留直达Path
        void ReduceTransshippingPaths() {

            std::vector<Request> newRequestSet(input_.GetRequests().size());
            for (int i = 0; i < input_.GetRequests().size(); i++) {
                Request request = input_.GetRequests()[i];

                if (request.GetNumberOfLadenPath() == 0) {
                    newRequestSet[i] = request;
                    continue;
                }

                std::vector<int> newLadenPaths(request.GetLadenPaths().size());
                std::vector<int> newLadenPathIndexes(request.GetLadenPaths().size());
                int newNumLadenPaths = 0;
                int flag = 0;

                for (int j = 0; j < request.GetLadenPathIndexes().size(); j++) {
                    int index = request.GetLadenPathIndexes()[j];
                    if (input_.GetPathSet()[index].GetTransshipPorts().empty()) {
                        newLadenPaths[newNumLadenPaths] = request.GetLadenPaths()[j];
                        newLadenPathIndexes[newNumLadenPaths] = index;
                        newNumLadenPaths++;
                        flag = 1;
                    }
                }

                if (flag == 1 && newNumLadenPaths < request.GetNumberOfLadenPath()) {
                    std::vector<int> newLadenID(newNumLadenPaths);
                    std::vector<int> newLadenIndex(newNumLadenPaths);
                    for (int k = 0; k < newNumLadenPaths; k++) {
                        newLadenID[k] = newLadenPaths[k];
                        newLadenIndex[k] = newLadenPathIndexes[k];
                        request.SetLadenPaths(newLadenID);
                        request.SetLadenPathIndexes(newLadenIndex);
                    }
                    request.SetNumberOfLadenPath(newNumLadenPaths);
                }
                newRequestSet[i] = request;
            }
            input_.SetRequest(newRequestSet);
        }

        // 将重定向到“进口型”港口的Request的可选EmptyPath数量置为0
        void ReduceEmptyPaths() {
            std::vector<int> min_importContainers(input_.GetPortSet().size());
            std::vector<int> max_exportContainers(input_.GetPortSet().size());

            // 计算各个港口的总流入量与总流出量
            for (int i = 0; i < input_.GetRequests().size(); i++) {
                Request request = input_.GetRequests()[i];

                std::string origin = request.GetOriginPort();
                std::string destination = request.GetDestinationPort();

                for (int j = 0; j < input_.GetPortSet().size(); j++) {
                    if (input_.GetPortSet()[j].GetPort() == origin) {
                        max_exportContainers[j] += para_.GetDemand()[i] + para_.GetMaximumDemandVariation()[i];
                    }
                    if (input_.GetPortSet()[j].GetPort() == destination) {
                        min_importContainers[j] += para_.GetDemand()[i];
                    }
                }
            }

            for (int pp = 0; pp < input_.GetPortSet().size(); pp++) {
                if (max_exportContainers[pp] <= min_importContainers[pp]) {
                    std::vector<Request> newRequestSet(input_.GetRequests().size());
                    for (int j = 0; j < input_.GetRequests().size(); j++) {
                        newRequestSet[j] = input_.GetRequests()[j];
                        if (input_.GetRequests()[j].GetOriginPort() == input_.GetPortSet()[pp].GetPort())
                        {
                            newRequestSet[j].SetEmptyPaths(std::vector<int>());
                            newRequestSet[j].SetEmptyPathIndexes(std::vector<int>());
                            newRequestSet[j].SetNumberOfEmptyPath(0);
                        }
                    }
                    input_.SetRequest(newRequestSet);
                }
            }
        }

        // 仅保留不同区域间的Request
        void CutRequests() {
            std::vector<Request> newRequestSet;
            std::vector<double> tempDemands(para_.GetDemand().size());
            std::vector<double> tempMaxVarDemands(para_.GetDemand().size());
            int newNumOfRequests = 0;
            for (int i = 0; i < input_.GetRequests().size(); i++) {
                Request request = input_.GetRequests()[i];
                if (request.GetOriginGroup() != request.GetDestinationGroup()) {
                    newRequestSet.push_back(request);
                    tempDemands[newNumOfRequests] = para_.GetDemand()[i];
                    tempMaxVarDemands[newNumOfRequests] = para_.GetMaximumDemandVariation()[i];
                    newNumOfRequests++;
                }
            }

            std::vector<double> newDemands(newNumOfRequests);
            std::vector<double> newMaxVarDemands(newNumOfRequests);
            for (int i = 0; i < newNumOfRequests; i++) {
                newDemands[i] = tempDemands[i];
                newMaxVarDemands[i] = tempMaxVarDemands[i];
            }

            // 更新 Request Set
            input_.SetRequest(newRequestSet);
            para_.SetDemand(newDemands);
            para_.SetMaximumDemandVariation(newMaxVarDemands);
            //std::cout << "The number of Requests after Cutting Requests between same group: " << input_.GetRequest().size() << std::endl;
        }

        void RecalculatePaths() {
            int numOfPaths = 0;
            std::vector<ContainerPath> newPaths;
            std::vector<Request> newRequestSet(input_.GetRequests().size());

            for (int i = 0; i < input_.GetRequests().size(); i++) {
                Request request = input_.GetRequests()[i];

                // 由于减少Request而导致减少的Path
                std::vector<int> newLadenIndexes(input_.GetRequests()[i].GetNumberOfLadenPath());
                for (int j = 0; j < input_.GetRequests()[i].GetNumberOfLadenPath(); j++) {
                    int index = input_.GetRequests()[i].GetLadenPathIndexes()[j];
                    ContainerPath tempPath = input_.GetPathSet()[index];
                    newPaths.push_back(tempPath);
                    newLadenIndexes[j] = numOfPaths;
                    numOfPaths++;
                }
                request.SetLadenPathIndexes(newLadenIndexes);


                // 更新空路径
                if (request.GetNumberOfEmptyPath() == 0)
                    newRequestSet[i] = request;
                continue;
                int newNumOfEmptyPaths = 0;
                for (int j = 0; j < request.GetNumberOfEmptyPath(); j++) {
                    int tempPathID = request.GetEmptyPaths()[j];
                    for (int k = 0; k < input_.GetPathSet().size(); k++) {
                        if (input_.GetPathSet()[k].GetRequestID() == tempPathID) {
                            input_.GetRequests()[i].GetEmptyPaths().push_back(tempPathID);
                            input_.GetRequests()[i].GetEmptyPathIndexes().push_back(k);
                            newNumOfEmptyPaths++;
                            break;
                        }
                    }
                }

                std::vector<int> newEmptyPaths(newNumOfEmptyPaths);
                std::vector<int> newEmptyPathIndexes(newNumOfEmptyPaths);
                for (int j = 0; j < newNumOfEmptyPaths; j++) {
                    newEmptyPaths[j] = input_.GetRequests()[i].GetEmptyPaths()[j];
                    newEmptyPathIndexes[j] = input_.GetRequests()[i].GetEmptyPathIndexes()[j];
                }

                //input_.GetRequest()[i].SetNumberOfEmptyPath(newNumOfEmptyPaths);
                //input_.GetRequest()[i].SetEmptyPaths(newEmptyPaths);
                //input_.GetRequest()[i].SetEmptyPathIndexes(newEmptyPathIndexes);
                request.SetNumberOfEmptyPath(newNumOfEmptyPaths);
                request.SetEmptyPaths(newEmptyPaths);
                request.SetEmptyPathIndexes(newEmptyPathIndexes);

                newRequestSet[i] = request;
            }
            input_.SetRequest(newRequestSet);
            input_.SetPath(newPaths);

            // 更新成本
            // 计算每条路径的总滞期
            // 滞期 = sum{中转时间 * 单位滞期}
            // arcAndPath：弧X路径
            // arcAndPath[arc][path] == 1：该旅行弧在路径弧中
            std::vector<double> ladenPathDemurrageCost(input_.GetPathSet().size());
            std::vector<double> emptyPathDemurrageCost(input_.GetPathSet().size());
            std::vector<int> TravelTimeOnPath(input_.GetPathSet().size());
            std::vector<int> pathSet(input_.GetPathSet().size());
            std::vector<std::vector<int>> arcAndPath(input_.GetTravelArcs().size(), std::vector<int>(input_.GetPathSet().size(), 0));
            int x = 0;
            for (ContainerPath pp : input_.GetPathSet()) {
                ladenPathDemurrageCost[x] = std::max(0, 175 * (pp.GetTotalTransshipmentTime() - 7));
                emptyPathDemurrageCost[x] = std::max(0, 100 * (pp.GetTotalTransshipmentTime() - 7));

                TravelTimeOnPath[x] = pp.GetPathTime();
                pathSet[x] = pp.GetRequestID();

                for (int i = 0; i < input_.GetTravelArcs().size(); i++) {
                    arcAndPath[i][x] = 0;
                    for (int j = 0; j < pp.GetArcsID().size(); j++) {
                        if (input_.GetTravelArcs()[i].GetArcID() == pp.GetArcsID()[j]) {
                            arcAndPath[i][x] = 1;
                        }
                    }
                }
                ++x;
            }

            para_.SetLadenPathDemurrageCost(ladenPathDemurrageCost);
            para_.SetEmptyPathDemurrageCost(emptyPathDemurrageCost);
            para_.SetTravelTimeOnPath(TravelTimeOnPath);
            para_.SetPathSet(pathSet);
            para_.SetArcAndPath(arcAndPath);
        }
    };


}  // namespace fleetdeployment

#endif  // CUTTING_PATHS_H_