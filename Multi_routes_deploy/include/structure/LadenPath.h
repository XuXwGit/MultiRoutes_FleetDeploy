#ifndef LADEN_PATH_H_
#define LADEN_PATH_H_

#include <string>
#include <vector>

#include "ContainerPath.h"


namespace fleetdeployment
{
    class  LadenPath;
}


// LadenPath class  represents a path with various shipping details.
class  fleetdeployment::LadenPath : public ContainerPath {
private:
    int round_trip_;
    int w_i_earlist_;

public:


    // Getters and Setters for the class  members.
    inline const int GetPathID() const { return PathID; }
    void SetPathID(int path_ID) { PathID = path_ID; }

    inline const int GetRequestID() const { return request_ID; }
    void SetRequestID(int request_ID) { request_ID = request_ID; }

    const std::string& GetOriginPort() const { return origin_port; }
    void SetOriginPort(const std::string& origin_port_) { origin_port = origin_port_; }

    inline const int GetOriginTime() const { return origin_time; }
    void SetOriginTime(const int origin_time_) { origin_time = origin_time_; }

    const std::string& GetDestinationPort() const { return destination_port; }
    void SetDestinationPort(const std::string& _destination_port_) { destination_port = _destination_port_; }

    inline const int GetRoundTrip() const { return round_trip_; }
    void SetRoundTrip(int round_trip) { round_trip_ = round_trip; }

    inline const int GetWiEarlist() const { return w_i_earlist_; }
    void SetWiEarlist(int w_i_earlist) { w_i_earlist_ = w_i_earlist; }

    inline const int GetArrivalTimeToDestination() const { return destination_time; }
    void SetArrivalTimeToDestination(int arrival_time_to_destination) { destination_time = arrival_time_to_destination; }

    inline const int GetPathTime() const { return path_time; }
    void SetPathTime(int pathtime) { path_time = pathtime; }

    const std::vector<std::string>& GetTransshipPorts() const { return transship_port; }
    void SetTransshipPorts(const std::vector<std::string>& transshipment_port) { transship_port = transshipment_port; }

    inline const int GetTransshipmentTime() const {
        int total_transship_time = 0;
        for (int time : transship_time) {
            total_transship_time += time;
        }
        return total_transship_time;
    }

    inline void SetTransshipmentTime(const std::vector<int>& transshipment_time) { transship_time = transshipment_time; }

    const std::vector<std::string>& GetPortPath() const { return Path_port; }
    void SetPortPath(const std::vector<std::string>& port_path) { Path_port = port_path; }

    inline const int GetNumberOfArcs() const { return num_arcs; }
    void SetNumArcs(int number_of_arcs) { num_arcs = number_of_arcs; }
};



#endif // LADEN_PATH_H_