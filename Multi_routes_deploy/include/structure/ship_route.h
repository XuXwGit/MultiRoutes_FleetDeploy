#ifndef _SHIP_ROUTE_H_
#define _SHIP_ROUTE_H_

#include <iostream>
#include <string>
#include <vector>

#include "OD.h"
#include "Port.h"

using namespace std;

namespace fleetdeployment {
struct route_node;
class ship_route;

// input data(P, R, W)
// P: the set of ports,  |P| : P.size() : the number of port
// R: the set of ship routes
// W: the set of ods
bool inputData(string& path, map<string, Port>& P, vector<ship_route>& R, vector<OD>& W);

map<string, Port>& inputPortData(string& path, map<string, Port>& P);

vector<ship_route>& inputRouteData(string& path, vector<ship_route>& R);

vector<OD>& inputODData(string& path, vector<OD>& W, map<string, Port>& P);
}  // namespace fleetdeployment

struct fleetdeployment::route_node {
  int ID;
  int call;          // para_r_i ¡Ê {1, 2, ..., N_r}
  std::string port;  // p ¡Ê P
  int arrivalTime;   // t_arr_ri
  int routeID;
  int departureTime;
  route_node() {}
  route_node(int call, string port, int arrival)
      : call(call), port(port), arrivalTime(arrival) {}
  route_node(int ID, int call, std::string port, int arrivalTime, int routeID)
      : ID(ID),
        call(call),
        port(port),
        arrivalTime(arrivalTime),
        routeID(routeID) {}
};

class fleetdeployment::ship_route {
 public:
  typedef pair<int, double> ship_type;
  int num_of_rotation = 0;
  int num_port;  // N_r

 private:
  int routeID;         // r
  std::string origin;  // para_r_0
  std::vector<route_node> path;
  // int num_port_call = -1;
  int transitTime;                   // T_r (h)
  int num_ship;                      // m_r = Tr / 7 days
  int capacity;                      // Cap_r (TEUs)
  std::vector<ship_type> shipTypes;  // Vr (Capacity, operate cost)

  int num_ports;                        // Number of ports in the route
  std::vector<std::string> Ports;       // Ports in the route
  int num_port_calls;                   // Number of port calls
  std::vector<std::string> port_calls;  // Ports of call
  std::vector<int> TimePointsOfCall;    // Time points of call

 public:
  ship_route(const int ID, std::vector<route_node>& path)
      : routeID(ID),
        origin(path[0].port),
        path(path),
        num_port((int)path.size() - 1),
        transitTime(path.back().arrivalTime - path.front().arrivalTime),
        num_ship((int)ceil(path.back().arrivalTime / 7 * 1)),
        capacity(0) {}
  ship_route()
      : routeID(-1),
        transitTime(-1),
        num_port(-1),
        num_ship(-1),
        capacity(-1) {}
  ~ship_route(){};

  inline const int GetID() const { return routeID; }
  inline const int GetRouteID() const { return routeID; }
  inline void SetRouteID(const int ID) { routeID = ID; }
  inline const std::vector<route_node>& GetPath() const { return path; }
  inline const std::string& GetPort(const int i) const {
    try {
      if (i >= 0 && i <= num_port)
        return path[i].port;
      else {
        std::cerr << "Error in get port" << std::endl;
      }
    } catch (std::exception& e) {
      std::cerr << e.what() << std::endl;
      throw;
    }

    return path[i].port;
  }

  inline const int GetNumPort() const { return num_port; }
  inline const int GetPortNum() const { return num_port; }
  inline const int GetNumRotation() const { return num_of_rotation; }
  inline const int GetCapacity() const { return capacity; }
  inline const int GetTransitTime() const { return transitTime; }
  inline const std::vector<pair<int, double>>& GetShipTypes() const {
    return shipTypes;
  }
  inline void SetShipTypes(std::vector<pair<int, double>>& types) {
    shipTypes = types;
  }
  inline const int GetNumPortCall() const { return (int)(path.size()); }

  // Getter for num_ports
  int GetNumberofPorts() const { return num_ports; }

  // Setter for num_ports
  void SetNumberofPorts(int newNumberofPorts) { num_ports = newNumberofPorts; }

  // Getter for num_port_calls
  int GetNumberofCall() const { return num_port_calls; }

  // Setter for num_port_calls
  void SetNumberofCall(int newNumberofCall) {
    num_port_calls = newNumberofCall;
  }

  // Getter for Ports
  std::vector<std::string> GetPorts() const { return Ports; }

  // Setter for Ports
  void SetPorts(const std::vector<std::string>& newPorts) { Ports = newPorts; }

  // Getter for port_calls
  std::vector<std::string> GetPortsofCall() const { return port_calls; }
  std::vector<std::string> GetPortCalls() const { return port_calls; }

  // Setter for port_calls
  void SetPortsofCall(const std::vector<std::string>& newPortsofCall) {
    port_calls = newPortsofCall;
  }

  // Getter for TimePointsOfCall
  std::vector<int> GetTimePointsOfCall() const { return TimePointsOfCall; }

  // Setter for TimePointsOfCall
  void SetTimePointsOfCall(const std::vector<int>& newTimePointsOfCall) {
    TimePointsOfCall = newTimePointsOfCall;
  }

  // Function to Get the call index of a port
  int GetCallIndexOfPort(const std::string& port) const {
    for (int p = 0; p < num_port_calls - 1; p++) {
      if (port == port_calls[p]) {
        return p;
      }
    }
    return -1;
  }
};

#endif  // !_SHIP_ROUTE_H_