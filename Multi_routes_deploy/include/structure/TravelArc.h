#ifndef _TravelArc_H_
#define _TravelArc_H_

#include <string>

#include "Arc.h"

namespace fleetdeployment
{
    class TravelArc;

}

class  fleetdeployment::TravelArc : public Arc {
private:
    int route;              // Route ID
    int round_Trip;         // Round trip indicator

    int origin_node_ID;     // Origin node ID
    int origin_Call;        // Origin call
    int originTime;         // Origin time
    int destination_node_ID; // Destination node ID
    int Destination_Call;    // Destination call
    int destinationTime;     // Destination time

    std::string origin_Port; // Origin port name
    std::string destination_Port; // Destination port name

public:
    // Default constructor
    inline TravelArc() : Arc(), route(0), origin_node_ID(0), origin_Call(0), origin_Port(""), round_Trip(0), originTime(0), destination_node_ID(0), Destination_Call(0), destination_Port(""), destinationTime(0) {}

    // Getter for route
    inline const int GetRoute() const {
        return route;
    }

    // Setter for route
    inline void SetRoute(const int newRoute) {
        route = newRoute;
    }

    // Getter for route index (route - 1)
    inline const int GetRouteIndex() const {
        return route - 1;
    }

    // Getter for origin_node_ID
    inline const int Getorigin_node_ID() const {
        return origin_node_ID;
    }

    // Setter for origin_node_ID
    inline void Setorigin_node_ID(const int neworigin_node_ID) {
        origin_node_ID = neworigin_node_ID;
    }

    // Getter for origin_Call
    inline const int Getorigin_Call() const {
        return origin_Call;
    }

    // Setter for origin_Call
    inline void SetOriginCall(const int neworigin_Call) {
        origin_Call = neworigin_Call;
    }

    // Getter for origin_Port
    inline const std::string& GetOriginPort() const {
        return origin_Port;
    }

    // Setter for origin_Port
    inline void SetOriginPort(const std::string& new_origin_Port) {
        origin_Port = new_origin_Port;
    }

    // Getter for round_Trip
    inline const int GetRound_Trip() const {
        return round_Trip;
    }

    // Setter for round_Trip
    inline void SetRound_Trip(const int newRound_Trip) {
        round_Trip = newRound_Trip;
    }

    // Getter for originTime
    inline const int GetOriginTime() const {
        return originTime;
    }

    // Setter for originTime
    inline void SetOriginTime(const int newOriginTime) {
        originTime = newOriginTime;
    }

    // Getter for TravelTime
    inline const int GetTravelTime() const {
        return TravelTime;
    }

    // Setter for TravelTime
    inline const void SetTravelTime(const int newTravelTime) {
        TravelTime = newTravelTime;
    }

    // Getter for destination_node_ID
    inline const int GetDestination_node_ID() const {
        return destination_node_ID;
    }

    // Setter for destination_node_ID
    inline const void SetDestination_node_ID(const int newDestination_node_ID) {
        destination_node_ID = newDestination_node_ID;
    }

    // Getter for Destination_Call
    inline const int GetDestination_Call() const {
        return Destination_Call;
    }

    // Setter for Destination_Call
    inline void SetDestination_Call(const int newDestination_Call) {
        Destination_Call = newDestination_Call;
    }

    // Getter for destination_Port
    inline const std::string& GetDestinationPort() const {
        return destination_Port;
    }

    // Setter for destination_Port
    inline void SetDestination_Port(const std::string& newDestination_Port) {
        destination_Port = newDestination_Port;
    }

    // Getter for destinationTime
    inline const int GetDestinationTime() const {
        return destinationTime;
    }

    // Setter for destinationTime
    inline void SetDestinationTime(const int newDestinationTime) {
        destinationTime = newDestinationTime;
    }
};


#endif // _TravelArc_H_
