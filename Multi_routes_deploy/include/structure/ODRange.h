#ifndef ODRANGE_H_
#define ODRANGE_H_



namespace fleetdeployment
{
    /// <summary>
    /// this class  include the lower and upper bound
    /// of demand and freight (unit penalty cost)
    /// for a group pairs (origin group and destination group)
    /// </summary>
    class  ODRange {
    public:
        // Constructor
        ODRange(int origin_group, int destination_group,
            int demand_lower_bound, int demand_upper_bound,
            int freight_lower_bound, int freight_upper_bound)
            :
            origin_group_(origin_group),
            destination_group_(destination_group),
            demand_lower_bound_(demand_lower_bound),
            demand_upper_bound_(demand_upper_bound),
            freight_lower_bound_(freight_lower_bound),
            freight_upper_bound_(freight_upper_bound) {}

        // Getter functions
        inline int GetOriginGroup() const { return origin_group_; }
        inline int GetDestinationGroup() const { return destination_group_; }
        inline int GetDemandLowerBound() const { return demand_lower_bound_; }
        inline int GetDemandUpperBound() const { return demand_upper_bound_; }
        inline int GetFreightLowerBound() const { return freight_lower_bound_; }
        inline int GetFreightUpperBound() const { return freight_upper_bound_; }

    private:
        const int origin_group_;
        const int destination_group_;
        const int demand_lower_bound_;
        const int demand_upper_bound_;
        const int freight_lower_bound_;
        const int freight_upper_bound_;
    };




}
#endif  // ODRANGE_H_
