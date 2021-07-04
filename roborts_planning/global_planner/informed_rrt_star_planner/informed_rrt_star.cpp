#include "informed_rrt_star.h"
namespace roborts_global_planner
{
    using roborts_common::ErrorCode;
    using roborts_common::ErrorInfo;
    namespace ob = ompl::base;
    namespace og = ompl::geometric;
    std::shared_ptr<roborts_costmap::CostmapInterface> costmap_collision_check;

    class ValidityChecker : public ob::StateValidityChecker
    {
    public:
        ValidityChecker(const ob::SpaceInformationPtr &si) : ob::StateValidityChecker(si) {}

        bool isValid(const ob::State *state) const
        {
            const ob::RealVectorStateSpace::StateType *state2D =
                state->as<ob::RealVectorStateSpace::StateType>();
            double x, y, z;
            x = state2D->values[0];
            y = state2D->values[1];
            return isObsFree(x, y);
        }
    };

    InformedRRTStar::InformedRRTStar(CostmapPtr costmap_ptr) : GlobalPlannerBase::GlobalPlannerBase(costmap_ptr),
                                                            gridmap_width_(costmap_ptr_->GetCostMap()->GetSizeXCell()),
                                                            gridmap_height_(costmap_ptr_->GetCostMap()->GetSizeYCell()),
                                                            cost_(costmap_ptr_->GetCostMap()->GetCharMap())
    {
        costmap_collision_check = costmap_ptr;
    }

    InformedRRTStar::~InformedRRTStar()
    {
        cost_ = nullptr;
    }

    bool isObsFree(const double map_x, const double map_y)
    {
        unsigned int x_index, y_index;
        costmap_collision_check->GetCostMap()->World2Map(map_x, map_y,x_index,y_index);
        unsigned char index_cost = costmap_collision_check->GetCostMap()->GetCost(x_index,y_index);
        return (map_x > costmap_collision_check->GetCostMap()->GetOriginX() && map_x < costmap_collision_check->GetCostMap()->GetSizeXCell() &&
                map_y > costmap_collision_check->GetCostMap()->GetOriginY() && map_y < costmap_collision_check->GetCostMap()->GetSizeYCell() &&
                index_cost < 253);
    }

    /**
 * @brief method for OMPL optimal ---get path length Objective
 * 
 * @param si  state space
 * @return ob::OptimizationObjectivePtr 
 */
    ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr &si)
    {
        return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
    }

    /**
 * @brief method for OMPL optimal Get the Threshold Path Length Obj object
 * 
 * @param si  state space
 * @return ob::OptimizationObjectivePtr 
 */
    ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr &si)
    {
        ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
        obj->setCostThreshold(ob::Cost(245));
        return obj;
    }

    roborts_common::ErrorInfo InformedRRTStar::Plan(const geometry_msgs::PoseStamped &start,
                                                    const geometry_msgs::PoseStamped &goal,
                                                    std::vector<geometry_msgs::PoseStamped> &path)
    {
        unsigned int start_x, start_y, goal_x, goal_y, tmp_goal_x, tmp_goal_y;
        unsigned int valid_goal[2];
        unsigned int shortest_dist = std::numeric_limits<unsigned int>::max();
        bool goal_valid = false;
        //check the validty of start position
        if (!costmap_ptr_->GetCostMap()->World2Map(start.pose.position.x,
                                                start.pose.position.y,
                                                start_x,
                                                start_y))
        {
            ROS_WARN("Failed to transform start pose from map frame to costmap frame");
            return ErrorInfo(ErrorCode::GP_POSE_TRANSFORM_ERROR,
                            "Start pose can't be transformed to costmap frame.");
        }
        //check validty of goal position
        if (!costmap_ptr_->GetCostMap()->World2Map(goal.pose.position.x,
                                                goal.pose.position.y,
                                                goal_x,
                                                goal_y))
        {
            ROS_WARN("Failed to transform goal pose from map frame to costmap frame");
            return ErrorInfo(ErrorCode::GP_POSE_TRANSFORM_ERROR,
                            "Goal pose can't be transformed to costmap frame.");
        }
        //check if goal point in obs space
        //if so, change a near point which is free to replace the goal point_hypotf_hypotf
        if (costmap_ptr_->GetCostMap()->GetCost(goal_x, goal_y) < inaccessible_cost_)
        {
            valid_goal[0] = goal_x;
            valid_goal[1] = goal_y;
            goal_valid = true;
        }
        else
        {
            tmp_goal_x = goal_x;
            tmp_goal_y = goal_y - goal_search_tolerance_;

            while (tmp_goal_y <= goal_y + goal_search_tolerance_)
            {
                tmp_goal_x = goal_x - goal_search_tolerance_;
                while (tmp_goal_x <= goal_x + goal_search_tolerance_)
                {
                    unsigned char cost = costmap_ptr_->GetCostMap()->GetCost(tmp_goal_x, tmp_goal_y);
                    unsigned int dist = fabs(goal_x - tmp_goal_x) + fabs(goal_y - tmp_goal_y);
                    if (cost < inaccessible_cost_ && dist < shortest_dist)
                    {
                        shortest_dist = dist;
                        valid_goal[0] = tmp_goal_x;
                        valid_goal[1] = tmp_goal_y;
                        goal_valid = true;
                    }
                    tmp_goal_x += 1;
                }
                tmp_goal_y += 1;
            }
        }
        //start planning
        ErrorInfo error_info;
        if (!goal_valid)
        {
            error_info = ErrorInfo(ErrorCode::GP_GOAL_INVALID_ERROR);
            path.clear();
        }
        else
        {
            unsigned int start_index, goal_index;
            start_index = costmap_ptr_->GetCostMap()->GetIndex(start_x, start_y);
            goal_index = costmap_ptr_->GetCostMap()->GetIndex(valid_goal[0], valid_goal[1]);

            costmap_ptr_->GetCostMap()->SetCost(start_x, start_y, roborts_costmap::FREE_SPACE);

            if (start_index == goal_index)
            {
                error_info = ErrorInfo::OK();
                path.clear();
                path.push_back(start);
                path.push_back(goal);
            }
            else
            {
                error_info = SearchPath(start, goal, path);
                if (error_info.IsOK())
                {
                    path.back().pose.orientation = goal.pose.orientation;
                    path.back().pose.position.z = goal.pose.position.z;
                }
            }
        }
        return ErrorInfo(ErrorCode::OK);
    }

    roborts_common::ErrorInfo InformedRRTStar::SearchPath(const geometry_msgs::PoseStamped &start,
                                                            const geometry_msgs::PoseStamped &goal,
                                                            std::vector<geometry_msgs::PoseStamped> &path)
        {
        ob::StateSpacePtr space(new ob::RealVectorStateSpace(2));
        ob::RealVectorBounds bounds(2);
        //set the The boundary conditions of the space
        //0  is on behalf of X dim and 1 represents of Y dimss 
        bounds.setLow(0, costmap_ptr_->GetCostMap()->GetOriginX());
        bounds.setLow(1, costmap_ptr_->GetCostMap()->GetOriginY());
        bounds.setHigh(0, costmap_ptr_->GetCostMap()->GetSizeXWorld());
        bounds.setHigh(1, costmap_ptr_->GetCostMap()->GetSizeYWorld());
        space->as<ob::RealVectorStateSpace>()->setBounds(bounds);
        // Construct a space information instance for this state space
        ob::SpaceInformationPtr si(new ob::SpaceInformation(space));
        // Set the object used to check which states in the space are valid
        si->setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(si)));
        si->setup();
        //Setting start state and goal state
        ob::ScopedState<> ompl_start(space);
        ompl_start[0] = start.pose.position.x;
        ompl_start[1] = start.pose.position.y;
        ob::ScopedState<> ompl_goal(space);
        ompl_goal[0] = goal.pose.position.x;
        ompl_goal[1] = goal.pose.position.y;
        //constuct the  problem which is waiting to solve.
        ob::ProblemDefinitionPtr pdef(std::make_shared<ob::ProblemDefinition>(si));
        pdef->setStartAndGoalStates(ompl_start, ompl_goal);
        auto optimization = getThresholdPathLengthObj(si);

        ob::PlannerPtr optimizingPlanner(new og::InformedRRTstar(si));
        optimizingPlanner->setProblemDefinition(pdef);
        optimizingPlanner->setup();
        //try to find a valid path in 200ms
        ob::PlannerStatus solved = optimizingPlanner->solve(0.2);
        if (solved)
        {
            geometry_msgs::PoseStamped iter_pos;
            double current_x,current_y,next_x,next_y,dis,diff_x,diff_y,k;
            iter_pos.pose.orientation.w = 1;
            iter_pos.header.frame_id = "map";
            path.clear();
            og::PathGeometric *og_path = pdef->getSolutionPath()->as<og::PathGeometric>();
            for (size_t path_index = 0; path_index < og_path->getStateCount()-1; path_index++)
            {
                //current state
                const ob::RealVectorStateSpace::StateType *state = og_path->getState(path_index)->as<ob::RealVectorStateSpace::StateType>();                
                current_x = state->values[0];
                current_y = state->values[1];
                //next state
                state = og_path->getState(path_index+1)->as<ob::RealVectorStateSpace::StateType>();                
                next_x = state->values[0];
                next_y = state->values[1];
                /*
                    calculate the Euclidean distance, maybe other space state can be faster 
                    and more infficient such as the Manhattan distance. We have not tried yet.
                */
                diff_x = next_x- current_x;
                diff_y = next_y-current_y;
                dis = std::sqrt(diff_x * diff_x +diff_y*diff_y);
                /*
                    since in some place especially the straight  and free place few point can be sparse which is not a good news for 
                    local plan to purge a global path within our architecture. Thus we insert some point between tow states
                    interpolation: we choose the easiest way to interpolation. This may be not the best approach but it also obtains a efficient path
                */
                if(dis > 0.1)
                {
                    k = dis/0.1;
                    diff_x /= k;
                    diff_y /= k;
                    for(size_t i = 0; i< k;i ++)
                    {
                        iter_pos.pose.position.x = current_x + i* diff_x;
                        iter_pos.pose.position.y = current_y + i* diff_y;
                        path.push_back(iter_pos);
                    }
                    iter_pos.pose.position.x = next_x;
                    iter_pos.pose.position.y = next_y;
                    path.push_back(iter_pos);
                }
                else
                {
                    iter_pos.pose.position.x = current_x;
                    iter_pos.pose.position.y = current_y;
                    path.push_back(iter_pos);
                }
            }
        }
        return ErrorInfo(ErrorCode::OK);
    }
}
