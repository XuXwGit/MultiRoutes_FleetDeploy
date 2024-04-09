#ifndef GENERATEPARAMETER_H_
#define GENERATEPARAMETER_H_


#include <iostream>
#include <vector>
#include <string>
#include <random>
#include <algorithm>

#include "Setting.h"
#include "Vessel.h"
#include "Parameter.h"
#include "InputData.h"


namespace fleetdeployment
{


 class  GenerateParameter : public Setting {
    private:
        Parameter& p_;
        const InputData& in_;
        int timeHorizon;
        double uncertainDegree;
        std::default_random_engine random;

    public:
        GenerateParameter(Parameter& p, InputData& in, int timeHorizon, double uncertainDegree)
            : p_(p), in_(in), timeHorizon(timeHorizon), uncertainDegree(uncertainDegree) {
            frame();
        }

    private:
        void frame() {
            random.seed(randomSeed);
            SetSet();
        }

        void SetSet() {
            SetArcSet();
            SetTimePoint();
            SetPorts();
            SetRequests();
            SetShipRoutes();
            SetVessels();
            SetVesselPaths();
            SetPaths();
            SetArcCapacity();
            SetInitialEmptyContainers();
        }

        void SetArcSet() {
            std::vector<int> TravellingArcsSet(in_.GetTravelArcs().size());
            int x = 0;
            for (TravelArc tt : in_.GetTravelArcs()) {
                TravellingArcsSet[x] = tt.GetArcID();
                x++;
            }
            p_.SetTravellingArcsSet(TravellingArcsSet);

            std::vector<int> transhipmentArcsSet(in_.GetTransshipArc().size());
            x = 0;
            for (TransshipArc tt : in_.GetTransshipArc()) {
                transhipmentArcsSet[x] = tt.GetArcID();
                x++;
            }
            p_.SetTranshipmentArcsSet(transhipmentArcsSet);
        }

        void SetTimePoint() {
            std::vector<int> timePointSet(timeHorizon + 1);
            for (int i = 0; i < timeHorizon + 1; i++) {
                timePointSet[i] = i;
            }
            p_.SetTimePointSet(timePointSet);
        }

        void SetPorts() {
            p_.SetRentalCost(50);
            std::vector<std::string> portSet(in_.GetPortSet().size());
            std::vector<int> turnOverTime(in_.GetPortSet().size(), 14);
            std::vector<double> ladenDemurrageCost(in_.GetPortSet().size(), 175);
            std::vector<double> emptyDemurrageCost(in_.GetPortSet().size(), 100);
            int x = 0;
            for (Port pp : in_.GetPortSet()) {
                portSet[x] = pp.GetPort();
                pp.SetRentalCost(50);
                pp.SetTurnOverTime(14);
                pp.SetLadenDemurrageCost(175);
                pp.SetEmptyDemurrageCost(100);
                pp.SetLoadingCost(20);
                pp.SetDischargeCost(20);
                pp.SetTransshipmentCost(30);
                x++;
            }
            p_.SetPortSet(portSet);
            p_.SetTurnOverTimeSet(turnOverTime);
            p_.SetLadenDemurrageCost(ladenDemurrageCost);
            p_.SetEmptyDemurrageCost(emptyDemurrageCost);
        }

        void SetRequests() {
            std::vector<int> demandRequest(in_.GetRequests().size());
            std::vector<std::string> originOfDemand(in_.GetRequests().size());
            std::vector<std::string> destinationOfDemand(in_.GetRequests().size());
            std::vector<double> demand(in_.GetRequests().size());
            std::vector<double> demandMaximum(in_.GetRequests().size());
            std::vector<double> penaltyCostForDemand(in_.GetRequests().size());
            int x = 0;
            for (Request rr : in_.GetRequests()) {
                demandRequest[x] = rr.GetRequestID();
                originOfDemand[x] = rr.GetOriginPort();
                destinationOfDemand[x] = rr.GetDestinationPort();
                int groupO = rr.GetOriginGroup();
                int groupD = rr.GetDestinationGroup();

                ODRange groupRangeRange = in_.GetGroupRange(groupO, groupD);

                int demand_lb = groupRangeRange.GetDemandLowerBound();
                int demand_ub = groupRangeRange.GetDemandUpperBound();
                std::uniform_int_distribution<int> randIntDistribution(demand_lb, demand_ub);
                demand[x] = randIntDistribution(random);

                double freight_lb = groupRangeRange.GetFreightLowerBound();
                double freight_ub = groupRangeRange.GetFreightUpperBound();
                std::uniform_real_distribution<double> randDoubleDistribution(freight_lb, freight_ub);
                penaltyCostForDemand[x] = randDoubleDistribution(random);

                demandMaximum[x] = demand[x] * uncertainDegree;
                x++;
            }

            p_.SetDemandRequestSet(demandRequest);
            p_.SetOriginOfDemand(originOfDemand);
            p_.SetDestinationOfDemand(destinationOfDemand);
            p_.SetDemand(demand);
            p_.SetMaximumDemandVariation(demandMaximum);
            p_.SetPenaltyCostForDemand(penaltyCostForDemand);
        }

        void SetShipRoutes() {
            std::vector<int> vesselRoute(in_.GetShipRouteSet().size());
            int x = 0;
            for (ship_route ss : in_.GetShipRouteSet()) {
                vesselRoute[x] = ss.GetRouteID();
                x++;
            }
            p_.SetVesselRouteSet(vesselRoute);
        }

        void SetVessels() {
            std::vector<int> vessel(in_.GetVessel().size());
            std::vector<int> vesselCapacity(in_.GetVessel().size());
            std::vector<double> vesselOperationCost(in_.GetVessel().size());
            std::vector<std::vector<int>> vesselTypeAndShippingRoute(in_.GetVessel().size(), std::vector<int>(in_.GetShipRouteSet().size(), 0));
            int x = 0;
            for (const Vessel& v : in_.GetVessel()) {
                int r = v.GetCurrentRouteID() - 1;
                vesselTypeAndShippingRoute[x][r] = 1;
                vessel[x] = v.GetVesselID();
                vesselCapacity[x] = v.GetCapacity();
                vesselOperationCost[x] = v.GetOperatingCost();
                x++;
            }
            p_.SetVesselTypeAndShipRoute(vesselTypeAndShippingRoute);
            p_.SetVesselSet(vessel);
            p_.SetVesselCapacity(vesselCapacity);
            p_.SetVesselOperationCost(vesselOperationCost);
        }

        void SetVesselPaths() {
            std::vector<std::vector<int>> shipRouteAndVesselPath(in_.GetShipRouteSet().size(), std::vector<int>(in_.GetVesselPathSet().size(), 0));
            std::vector<std::vector<int>> arcAndVesselPath(in_.GetTravelArcs().size(), std::vector<int>(in_.GetVesselPathSet().size(), 0));
            std::vector<int> vesselPathSet(in_.GetVesselPathSet().size());

            for (int w = 0; w < in_.GetVesselPathSet().size(); w++) {
                int ww = in_.GetVesselPathSet()[w].GetPathID();
                int r = in_.GetVesselPathSet()[w].GetRouteID() - 1;
                shipRouteAndVesselPath[r][w] = 1;

                for (int nn = 0; nn < in_.GetTravelArcs().size(); nn++) {
                    for (int j = 0; j < in_.GetVesselPathSet()[w].GetArcIDs().size(); j++) {
                        if (in_.GetTravelArcs()[nn].GetArcID() == in_.GetVesselPathSet()[w].GetArcIDs()[j]) {
                            arcAndVesselPath[nn][w] = 1;
                        }
                    }
                }
                vesselPathSet[w] = ww;
            }

            p_.SetArcAndVesselPath(arcAndVesselPath);
            p_.SetShipRouteAndVesselPath(shipRouteAndVesselPath);
            p_.SetVesselPathSet(vesselPathSet);
        }

        void SetPaths() {
            std::vector<double> PathLoadAndDischargeCost(in_.GetPathSet().size(), 0.0);
            std::vector<double> ladenPathDemurrageCost(in_.GetPathSet().size(), 0.0);
            std::vector<double> emptyPathDemurrageCost(in_.GetPathSet().size(), 0.0);
            std::vector<double> ladenPathCost(in_.GetPathSet().size(), 0.0);
            std::vector<double> emptyPathCost(in_.GetPathSet().size(), 0.0);
            std::vector<int> TravelTimeOnLadenPath(in_.GetPathSet().size(), 0);
            std::vector<int> PathSet(in_.GetPathSet().size());
            std::vector<std::vector<int>> arcAndPath(in_.GetTravelArcs().size(), std::vector<int>(in_.GetPathSet().size(), 0));
            int x = 0;

            for (ContainerPath pp : in_.GetPathSet()) {
                for (int i = 0; i < in_.GetPortSet().size(); i++) {
                    if (in_.GetPortSet()[i].GetPort() == pp.GetOriginPort()) {
                        PathLoadAndDischargeCost[x] += in_.GetPortSet()[i].GetLoadingCost();
                    }
                    else if (in_.GetPortSet()[i].GetPort() == pp.GetDestinationPort()) {
                        PathLoadAndDischargeCost[x] += in_.GetPortSet()[i].GetDischargeCost();
                    }
                    else {
                        if (pp.GetTransshipPorts().size() != 0 && pp.GetTransshipPorts().size() > 0) {
                            for (int j = 0; j < pp.GetTransshipPorts().size(); j++) {
                                if (in_.GetPortSet()[i].GetPort() == pp.GetTransshipPorts()[j]) {
                                    PathLoadAndDischargeCost[x] += in_.GetPortSet()[i].GetTransshipmentCost();
                                }
                            }
                        }
                    }
                }

                pp.SetPathCost(PathLoadAndDischargeCost[x]);
                ladenPathDemurrageCost[x] = (((0.0) >(175.0 * pp.GetTotalDemurrageTime())) ? (0.0) : (175.0 * pp.GetTotalDemurrageTime()));
                emptyPathDemurrageCost[x] = (((0.0) > (100.0 * pp.GetTotalDemurrageTime())) ? (0.0) : (100.0 * pp.GetTotalDemurrageTime()));
                ladenPathCost[x] = ladenPathDemurrageCost[x] + PathLoadAndDischargeCost[x];
                emptyPathCost[x] = emptyPathDemurrageCost[x] + PathLoadAndDischargeCost[x] * 0.5;
                TravelTimeOnLadenPath[x] = pp.GetPathTime();
                PathSet[x] = pp.GetRequestID();

                for (int i = 0; i < in_.GetTravelArcs().size(); i++) {
                    arcAndPath[i][x] = 0;
                    for (int j = 0; j < pp.GetArcIDs().size(); j++) {
                        if (in_.GetTravelArcs()[i].GetArcID() == pp.GetArcIDs()[j]) {
                            arcAndPath[i][x] = 1;
                        }
                    }
                }
                ++x;
            }

            p_.SetLadenPathDemurrageCost(ladenPathDemurrageCost);
            p_.SetEmptyPathDemurrageCost(emptyPathDemurrageCost);
            p_.SetLadenPathCost(ladenPathCost);
            p_.SetEmptyPathCost(emptyPathCost);
            p_.SetTravelTimeOnPath(TravelTimeOnLadenPath);
            p_.SetPathSet(PathSet);
            p_.SetArcAndPath(arcAndPath);
        }

        void SetArcCapacity() {
            for (int nn = 0; nn < p_.GetTravelArcsSet().size(); nn++) {
                double capacity = 0.0;
                for (int r = 0; r < p_.GetVesselRouteSet().size(); r++) {
                    for (int w = 0; w < p_.GetVesselPathSet().size(); w++) {
                        for (int h = 0; h < p_.GetVesselSet().size(); h++) {
                            capacity += p_.GetArcAndVesselPath()[nn][w] * p_.GetVesselTypeAndShipRoute()[h][r] * p_.GetVesselCapacity()[h];
                            capacity += p_.GetArcAndVesselPath()[nn][w] *
                                p_.GetVesselCapacity()[h] *
                                p_.GetShipRouteAndVesselPath()[r][w] *
                                p_.GetVesselTypeAndShipRoute()[h][r];
                        }
                    }
                }

                if (DebugEnable && GenerateParamEnable) {
                    std::cout << "RouteID = " << in_.GetTravelArcs()[nn].GetRoute() << '\t'
                        << "TravelArcID = " << in_.GetTravelArcs()[nn].GetArcID() << '\t'
                        << "(" << in_.GetTravelArcs()[nn].GetOriginPort() << ","
                        << in_.GetTravelArcs()[nn].GetDestinationPort() << ")" << '\t'
                        << "(" << in_.GetTravelArcs()[nn].GetOriginTime() << "--"
                        << in_.GetTravelArcs()[nn].GetDestinationTime() << ")" << '\t'
                        << "Total Capacity = " << capacity << std::endl;
                }
            }
        }

        void SetInitialEmptyContainers() {
            std::vector<int> initialEmptyContainer(in_.GetPortSet().size());
            int x = 0;
            std::uniform_real_distribution<double> random_zero_one(0.0, 1.0);
            double alpha = 0.8 + 0.2 * random_zero_one(random);
            for (Port pp : in_.GetPortSet()) {
                for (int i = 0; i < in_.GetRequests().size(); i++) {
                    if (pp.GetPort() == in_.GetRequests()[i].GetOriginPort() && in_.GetRequests()[i].GetW_i_Earliest() < 28) {
                        initialEmptyContainer[x] = static_cast<int>(initialEmptyContainer[x] + alpha * p_.GetDemand()[i]);
                    }
                }
                x++;
            }
            p_.SetInitialEmptyContainer(initialEmptyContainer);
        }
    };



}

#endif // !GENERATEPARAMETER_H_