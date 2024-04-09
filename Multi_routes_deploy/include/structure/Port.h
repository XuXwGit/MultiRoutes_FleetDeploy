#ifndef _PORT_H_
#define _PORT_H_

#include <string>
#include <utility>

namespace fleetdeployment
{
    class Port;

    static bool compare(const std::pair<std::string, Port>& left, const std::pair<std::string, Port>& right);
}

class  fleetdeployment::Port {
private:
    int ID;                            // Unique IDentifier for the port
    std::string port;                  // Port name
    std::string region;
    int whetherTrans;                  // Indicates whether the port is a transshipment port
    int group;                         // Port group IDentifier

    double unit_demurrage_laden_cost;			// unit-time demurrage cost of laden container (c4p)
    double unit_demurrage_empty_cost;		// unit-time demurrage cost of empty container (c5p)
    double loadingCost;                                 // Cost associated with loading
    double dischargeCost;                            // Cost associated with discharge
    double transshipmentCost;                     // Cost associated with transshipment
    double rentalCost;                                  // Rental cost for the port
    int container_turnover_time;                 // sp // Turnover time for the port

    int num_empty_container;		               // lp;

    std::vector<std::string> next_port_set;
    std::vector<std::string> front_port_set;

public:
    // Default constructor
    inline Port()
        : port(""), 
        region(""),
        whetherTrans(0),
        group(0),
        unit_demurrage_laden_cost(0.0),
        unit_demurrage_empty_cost(0.0),
        loadingCost(0.0),
        dischargeCost(0.0),
        transshipmentCost(0.0),
        rentalCost(0.0),
        container_turnover_time(0),
        num_empty_container(0),
        next_port_set(std::vector<std::string>()), 
        front_port_set(std::vector<std::string>())
    { }
    inline Port(const int port_id, const std::string& port) 
        : ID(port_id), 
        port(port),
        region(""),
        whetherTrans(0),
        group(0),
        unit_demurrage_laden_cost(0.0),
        unit_demurrage_empty_cost(0.0),
        loadingCost(0.0),
        dischargeCost(0.0),
        transshipmentCost(0.0),
        rentalCost(0.0),
        container_turnover_time(0),
        num_empty_container(0),
        next_port_set(std::vector<std::string>()),
        front_port_set(std::vector<std::string>())
    { }
    inline Port(const std::string& port, const double loadingCost, const double discharge, const double transshipment): port(port), loadingCost(loadingCost), dischargeCost(discharge), transshipmentCost(transshipment) { }

    // Getter for ID
    inline [[nodiscard]] const int GetID() const {
        return ID;
    }
    // Getter for port
    inline [[nodiscard]] const std::string& GetPort() const {
        return port;
    }
    // Getter for port
    inline [[nodiscard]] const std::string& GetRegion() const {
        return region;
    }
    inline [[nodiscard]] const int GetPortID() const {
        return ID;
    }
    // Getter for whetherTrans
    inline [[nodiscard]] const int GetWhetherTrans() const {
        return whetherTrans;
    }
    // Getter for group
    inline [[nodiscard]] const int GetGroup() const {
        return group;
    }
    // Getter for ladenDemurrageCost
    inline [[nodiscard]] const double GetLadenDemurrageCost() const {
        return unit_demurrage_laden_cost;
    }
    // Getter for emptyDemurrageCost
    inline [[nodiscard]] const double GetEmptyDemurrageCost() const {
        return unit_demurrage_empty_cost;
    }
    // Getter for loadingCost
    inline [[nodiscard]] const double GetLoadingCost() const {
        return loadingCost;
    }
    // Getter for dischargeCost
    inline [[nodiscard]] const double GetDischargeCost() const {
        return dischargeCost;
    }
    // Getter for transshipmentCost
    inline [[nodiscard]] const double GetTransshipmentCost() const {
        return transshipmentCost;
    }
    // Getter for turnOverTime
    inline [[nodiscard]] const int GetTurnOverTime() const {
        return container_turnover_time;
    }
    // Getter for rentalCost
    inline [[nodiscard]] const double GetRentalCost() const {
        return rentalCost;
    }
    inline const std::vector<std::string>& GetNextPortSet() const {
        return next_port_set;
    }
    inline const std::vector<std::string>& GetFrontPortSet() const {
        return front_port_set;
    }



    /*
    * Setters 
    */
    // Setter for ID
    inline void SetID(const int newID) {
        ID = newID;
    }
    // Setter for port
    inline void SetPort(const std::string& newPort) {
        port = newPort;
    }
    // Setter for port
    inline void SetRegion(const std::string& _region) {
        region = _region;
    }
    inline void SetPortID(const int portID) {
        ID = portID;
    }
    // Setter for whetherTrans
    inline void SetWhetherTrans(const int newWhetherTrans) {
        whetherTrans = newWhetherTrans;
    }
    // Setter for group
    inline void SetGroup(const int newGroup) {
        group = newGroup;
    }
    // Setter for ladenDemurrageCost
    inline void SetLadenDemurrageCost(double newLadenDemurrageCost) {
        unit_demurrage_laden_cost = newLadenDemurrageCost;
    }
    // Setter for emptyDemurrageCost
    inline void SetEmptyDemurrageCost(double newEmptyDemurrageCost) {
        unit_demurrage_empty_cost = newEmptyDemurrageCost;
    }
    // Setter for loadingCost
    inline void SetLoadingCost(double newLoadingCost) {
        loadingCost = newLoadingCost;
    }
    // Setter for dischargeCost
    inline void SetDischargeCost(double newDischargeCost) {
        dischargeCost = newDischargeCost;
    }
    // Setter for transshipmentCost
    inline void SetTransshipmentCost(double newTransshipmentCost) {
        transshipmentCost = newTransshipmentCost;
    }
    // Setter for turnOverTime
    inline void SetTurnOverTime(const int newTurnOverTime) {
        container_turnover_time = newTurnOverTime;
    }
    // Setter for rentalCost
    inline void SetRentalCost(double newRentalCost) {
        rentalCost = newRentalCost;
    }
    inline void SetNextPortSet(const std::vector<std::string>& newNextPortSet) {
        next_port_set = newNextPortSet;
    }
    inline void SetFrontPortSet(const std::vector<std::string>& newFrontPortSet) {
        front_port_set = newFrontPortSet;
    }

    inline void AddPort2NextPortSet(const std::string& port) {
        next_port_set.push_back(port);
    }
    inline void AddPort2FrontPortSet(const std::string& port) {
        front_port_set.push_back(port);
    }

    bool operator<(const Port& other) const {
        return ID < other.ID;
    }

    Port operator = (const Port& right) {
        ID = right.ID;                            // Unique IDentifier for the port
        port = right.port;                  // Port name
        whetherTrans = right.whetherTrans;                  // Indicates whether the port is a transshipment port
        group = right.group;                         // Port group IDentifier
        unit_demurrage_laden_cost = right.unit_demurrage_laden_cost;			// unit-time demurrage cost of laden container (c4p)
        unit_demurrage_empty_cost = right.unit_demurrage_empty_cost;		// unit-time demurrage cost of empty container (c5p)
        loadingCost = right.loadingCost;                // Cost associated with loading
        dischargeCost = right.dischargeCost;              // Cost associated with discharge
        transshipmentCost = right.transshipmentCost;          // Cost associated with transshipment
        container_turnover_time = right.container_turnover_time;                 // sp // Turnover time for the port
        rentalCost = right.rentalCost;                 // Rental cost for the port

        num_empty_container = right.num_empty_container;		// lp;

        next_port_set = right.next_port_set;
        front_port_set = right.front_port_set;

        return *this;
    }

    bool operator == (const Port& right) {
        if (ID == right.ID)
        {
            if (this->port == right.port) {
                return true;
            }
            return false;
        }
        else
        {
            return false;
        }
    }

    bool operator != (Port const& right) {
        return !(*this == right);
    }
};

static bool fleetdeployment::compare(const std::pair<std::string, Port>& left, const std::pair<std::string, Port>& right) {
    return left.second < right.second;
}

#endif // _PORT_H_