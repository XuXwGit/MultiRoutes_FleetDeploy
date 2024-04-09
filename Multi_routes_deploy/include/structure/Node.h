#ifndef NODE_H_
#define NODE_H_

#include <string>
namespace fleetdeployment
{
	class Node;
}
	// Node class  representing a network node with various attributes.
class  fleetdeployment::Node {
	private:
		int nodeID_;
		int routeID_;
		int call_;
		std::string port_;
		int round_trip_;
		int rotation_;
		int time_;

	public:
		// construct function
		Node() : nodeID_(-1), routeID_(-1), call_(-1), port_(""), round_trip_(-1), rotation_(-1), time_(-1) {}

		// Gets the Node ID.
		inline [[nodiscard]] const int GetNodeID() const { return nodeID_; }
		// Sets the Node ID.
		inline void SetNodeID(const int nodeID) { this->nodeID_ = nodeID; }

		inline [[nodiscard]] int GetCall() const {
			return call_;
		}
		inline void SetCall(const int _call) {
			call_ = _call;
		}

		// Gets the route.
		inline int GetRouteID() const { return routeID_; }
		// Sets the route.
		inline void SetRoute(const int route) { routeID_ = route; }
		inline void SetRouteID(const int route) { routeID_ = route; }

		// Gets the port.
		inline const std::string& GetPort() const { return port_; }
		// Sets the port.
		inline void SetPortString(const std::string& port) { port_ = port; }

		// Gets the round trip time.
		[[nodiscard]] inline int GetRoundTrip() const { return round_trip_; }
		// Sets the round trip time.
		inline void SetRoundTrip(const int round_trip) { round_trip_ = round_trip; }

		// Gets the time.
		[[nodiscard]] inline int GetTime() const { return time_; }
		// Sets the time.
		inline void SetTime(const int time) { time_ = time; }

		// Gets the rotation.
		[[nodiscard]] inline int GetRotation() const { return rotation_; }
		// Sets the rotation.
		inline void SetRotation(const int rotation) { rotation_ = rotation; }

		inline bool operator == (const Node& right)
		{
			return nodeID_ == right.GetNodeID();
		}

		inline bool operator == (const Node& right) const
		{
			return nodeID_ == right.GetNodeID();
		}

		inline bool operator != (const Node& right) const {
			return nodeID_ != right.GetNodeID();
		}
			 
		inline bool operator < (const Node& right)
		{
			return time_ < right.time_;
		}

		inline Node& operator = (const Node& right) {
			this->nodeID_ = right.nodeID_;
			this->call_ = right.call_;
			this->routeID_ = right.routeID_;
			this->port_ = right.port_;
			this->time_ = right.time_;
			this->rotation_ = right.rotation_;
			return *this;
		}

		inline const void printNodeSelf() const
		{
			std::cout << nodeID_
				<< "(" << port_ << ","
				<< routeID_ << ","
				<< call_ << ","
				<< rotation_ << ","
				<< time_ << ")";
		}
};

#endif  // NODE_H_
