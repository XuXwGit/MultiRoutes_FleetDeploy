#ifndef _ARC_H_
#define _ARC_H_

#include <iostream>

#include "Node.h"

namespace fleetdeployment
{
    class Arc;
}

class fleetdeployment::Arc
{
protected:
    int ArcID;
    int TravelTime;
    double cost;
    Node tail;
    Node head;

public:
    // Default constructor
    // member initialize list
    inline Arc() 
        : ArcID(0), TravelTime(0), cost(0.0), tail(Node()), head(Node()) {}
    inline Arc(const Node& tail_, const Node& head_)
		: ArcID(0), TravelTime(0), cost(0.0), tail(tail_), head(head_) {}

    [[nodiscard]] inline int GetArcID() const {
        return ArcID;
    }

    inline void SetArcID(const int id) {
        ArcID = id;
    }

    inline const Node& GetTail() const {
        return tail;
    }

    inline void SetTail(const Node& node) {
        tail = node;
    }

    inline const Node& GetHead() const {
        return head;
    }

    inline void SetHead(const Node& node) {
        head = node;
    }

    [[nodiscard]] inline int GetTravelTime() const {
        return TravelTime;
    }

    inline void SetTravelTime(const int time) {
        TravelTime = time;
    }

    [[nodiscard]] inline double GetCost() const {
        return cost;
    }

    inline void SetCost(double c) {
        cost = c;
    }

    inline const void printArc() const
    {
        std::cout << ArcID << ": ";
        tail.printNodeSelf();
        std::cout << "=>";
        head.printNodeSelf();
        //std::cout << std::endl;
    }

    inline const bool operator == (const Arc& right) const
    {
        return ArcID == right.GetArcID();
    }

    inline const bool operator != (const Arc& right) const
    {
        return ArcID != right.GetArcID();
    }

    inline bool operator < (const Arc& right)
    {
        if (tail.GetTime() == right.tail.GetTime())
            return head.GetTime() < right.GetHead().GetTime();
        return tail.GetTime() < right.tail.GetTime();
    }
};
#endif // !_ARC_H_
