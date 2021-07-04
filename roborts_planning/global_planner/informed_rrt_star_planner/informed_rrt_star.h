#ifndef _INFORMED_RRT_STAR
#define _INFORMED_RRT_STAR

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include "alg_factory/algorithm_factory.h"
#include "state/error_code.h"
#include "costmap/costmap_interface.h"
#include "../global_planner_base.h"

#include <ompl/config.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/Path.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include<ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/SimpleSetup.h>


namespace roborts_global_planner
{
    /**
     * @brief collision check
     * 
     * @param map_x the x coor of point 
     * @param map_y the y coor of point 
     * @return true free 
     * @return false occupied
     */
    bool isObsFree(const double map_x, const double map_y);        



    class InformedRRTStar:public GlobalPlannerBase
    {
        public:
        /**
         * @brief Construct a new Informed R R T Star object
         * 
         * @param costmap_ptr costmap ptr
         */
        InformedRRTStar(CostmapPtr costmap_ptr);
        ~InformedRRTStar();
        /**
         * @brief search a valid path with informed rrt *
         * 
         * @param start  the start position for this plan
         * @param goal 
         * @param path  result for this plan
         * @return roborts_common::ErrorInfo 
         */
        roborts_common::ErrorInfo Plan(const geometry_msgs::PoseStamped &start,
                                const geometry_msgs::PoseStamped &goal,
                                std::vector<geometry_msgs::PoseStamped> &path);
        roborts_common::ErrorInfo SearchPath(const geometry_msgs::PoseStamped &start,
                                const geometry_msgs::PoseStamped &goal,
                                    std::vector<geometry_msgs::PoseStamped> &path);

        private:
        //! inaccessible_cost
        unsigned int inaccessible_cost_;
        //! goal_search_tolerance
        unsigned int goal_search_tolerance_;
        //! gridmap height size
        unsigned int gridmap_height_;
        //! gridmap height width
        unsigned int gridmap_width_;
        //! gridmap cost array
        unsigned char *cost_;
    };

roborts_common::REGISTER_ALGORITHM(GlobalPlannerBase,
                                "informed_rrt_star_planner",
                                InformedRRTStar,
                                std::shared_ptr<roborts_costmap::CostmapInterface>);
}

#endif