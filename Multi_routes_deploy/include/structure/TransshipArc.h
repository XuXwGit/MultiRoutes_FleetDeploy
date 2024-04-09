#ifndef _TRANSSHIPARC_H_
#define _TRANSSHIPARC_H_

#include <string>

#include "Arc.h"

namespace fleetdeployment
{
	class TransshipArc;
}

class  fleetdeployment::TransshipArc : public Arc{
	private:
		int originNodeID;
		int originTime;
		int transshipTime;
		int destination_node_ID;
		int destinationTime;
		int fromRoute;
		int toRoute;
		std::string port;

	public:
		inline const std::string& GetPort() const{
			return port;
		}

		inline void SetPort(const std::string& port) {
			this->port = port;
		}

		inline const int GetOriginNodeID() const {
			return originNodeID;
		}

		inline void SetOriginNodeID(const int originNodeID) {
			this->originNodeID = originNodeID;
		}

		inline const int GetOriginTime() const {
			return originTime;
		}

		void SetOriginTime(const int originTime) {
			this->originTime = originTime;
		}

		inline const int GetTransshipTime() const {
			return transshipTime;
		}

		inline void SetTransshipTime(const int transshipTime) {
			this->transshipTime = transshipTime;
		}

		inline const int GetDestination_node_ID() const {
			return destination_node_ID;
		}

		inline void SetDestination_node_ID(const int destination_node_ID) {
			this->destination_node_ID = destination_node_ID;
		}

		inline const int GetDestinationTime() const {
			return destinationTime;
		}

		inline void SetDestinationTime(const int destinationTime) {
			this->destinationTime = destinationTime;
		}

		inline const int GetFromRoute() const {
			return fromRoute;
		}

		inline void SetFromRoute(const int fromRoute) {
			this->fromRoute = fromRoute;
		}

		inline const int GetToRoute() const {
			return toRoute;
		}

		inline void SetToRoute(const int toRoute) {
			this->toRoute = toRoute;
		}
	};

#endif // _TRANSSHIPARC_H_