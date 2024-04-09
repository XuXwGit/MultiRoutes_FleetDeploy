#ifndef INPUT_DATA_H_
#define INPUT_DATA_H_

#include <vector>
#include <map>
#include <string>
#include <iostream>

#include "Port.h"  
#include "ContainerPath.h"  

#include "Vessel.h"  
#include "ship_route.h"  
#include "Request.h"  
#include "ODRange.h"  
#include "Node.h" 
#include "TravelArc.h" 
#include "TransshipArc.h" 
#include "VesselPath.h"  
#include "LadenPath.h" 
#include "EmptyPath.h" 

namespace fleetdeployment
{


    class  InputData {
    public:
        InputData(){}

        void SetGroupRangeMap(const std::map<std::string, ODRange>& groupara_range_map) {
            group_range_map_ = groupara_range_map;
        }

        const ODRange GetGroupRange(int origin_group, int destination_group) const {
            std::string key = std::to_string(origin_group) + std::to_string(destination_group);
            return group_range_map_.at(key);
        }

        const std::vector<ContainerPath>& GetPathSet() const {
            return path_;
        }

        void SetPathSet(const std::vector<ContainerPath>& path) {
            path_ = path;
        }

        // Other
        const std::vector<ship_route>& GetShipRouteSet() const {
            return shiproute_;
        }

        void SetShipRoute(const std::vector<ship_route>& shiproute) {
            shiproute_ = shiproute;
        }

        const std::vector<Request>& GetRequests() const {
            return request_;
        }

        void SetRequest(const std::vector<Request>& request) {
            request_ = request;
        }

        const std::vector<Node>& GetNodes() const {
            return node_;
        }

        void SetNode(const std::vector<Node>& node) {
            node_ = node;
        }

        const std::vector<TravelArc>& GetTravelArcs() const {
            return travel_arcs_;
        }

        void SetTravelArcs(const std::vector<TravelArc>& travel_arcs) {
            travel_arcs_ = travel_arcs;
        }

        const std::vector<TransshipArc>& GetTransshipArc()  const {
            return transship_arcs_;
        }

        void SetTransshipArc(const std::vector<TransshipArc>& transship_arcs) {
            transship_arcs_ = transship_arcs;
        }

        const std::vector<VesselPath>& GetVesselPathSet() const {
            return vessel_path_;
        }

        void SetVesselPath(const std::vector<VesselPath>& vessel_path) {
            vessel_path_ = vessel_path;
        }

        const std::vector<LadenPath>& GetLadenPath() const {
            return laden_path_;
        }

        void SetLadenPath(const std::vector<LadenPath>& laden_path) {
            laden_path_ = laden_path;
        }

        const std::vector<EmptyPath>& GetEmptyPaths() const {
            return empty_paths_;
        }

        void SetEmptyPaths(const std::vector<EmptyPath>& empty_paths) {
            empty_paths_ = empty_paths;
        }

        const std::vector<Port>& GetPortSet() const {
            return port_;
        }

        void SetPortSet(const std::vector<Port>& port_set) {
            port_ = port_set;
        }

        const std::vector<Vessel>& GetVessel() const {
            return vessel_;
        }

        void SetVessel(const std::vector<Vessel>& vessel) {
            vessel_ = vessel;
        }

        void ShowStatus() {
            std::cout << "Nodes = " << GetNodes().size() << "\n"
                << "Travel Arcs = " << GetTravelArcs().size() << "\n"
                << "Transship Arcs = " << GetTransshipArc().size() << "\n"
                << "ShipRoute = " << GetShipRouteSet().size() << "\n"
                << "Ports = " << GetPortSet().size() << "\n"
                << "VesselPath = " << GetVesselPathSet().size() << "\n"
                << "Vessel Type = " << GetVessel().size() << "\n"
                << "Requests = " << GetRequests().size() << "\n"
                << "Paths = " << GetPathSet().size() << "\n";
        }
    private:
        std::vector<Port> port_;
        std::vector<Vessel> vessel_;
        std::vector<Node> node_;
        std::vector<TravelArc> travel_arcs_;
        std::vector<TransshipArc> transship_arcs_;
        std::vector<VesselPath> vessel_path_;
        std::vector<LadenPath> laden_path_;
        std::vector<EmptyPath> empty_paths_;
        std::vector<Request> request_;
        std::vector<ship_route> shiproute_;
        std::vector<ContainerPath> path_;
        std::map<std::string, ODRange> group_range_map_;
    };



} // namespace fleetdeployment
#endif // INPUT_DATA_H_