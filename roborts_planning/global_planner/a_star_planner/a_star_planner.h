/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/
#ifndef ROBORTS_PLANNING_GLOBAL_PLANNER_A_STAR_PLANNER_H
#define ROBORTS_PLANNING_GLOBAL_PLANNER_A_STAR_PLANNER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include "proto/a_star_planner_config.pb.h"

#include "alg_factory/algorithm_factory.h"
#include "state/error_code.h"
#include "costmap/costmap_interface.h"

#include "../global_planner_base.h"

namespace roborts_global_planner{

/**
 * @brief Global planner alogorithm class for A star under the representation of costmap
 */
class AStarPlanner : public GlobalPlannerBase {

 public:
  /**
   * @brief 构造一个A*规划器，设置costmap的指针并且制定相关costmap的大小
   * @param costmap_ptr costmap接口指针
   */
  AStarPlanner(CostmapPtr costmap_ptr);
  virtual ~AStarPlanner();
  /**
   * @brief 全局规划主函数
   * @param start 起点坐标
   * @param goal 终点坐标
   * @param path 全局规划路径输出
   * @return ErrorInfo 如果成功则返回success
   */
  roborts_common::ErrorInfo Plan(const geometry_msgs::PoseStamped &start,
                               const geometry_msgs::PoseStamped &goal,
                               std::vector<geometry_msgs::PoseStamped> &path);

 private:
  /**
   * @brief State enumerate for the cell.节点访问状态表
   */
  enum SearchState {
    NOT_HANDLED, /**< The cell is not handled.  该节点还没有被搜索过*/
    OPEN, /**< The cell is in open priority queue. 该节点已经被加入优先级队列中 */
    CLOSED /**< The cell is in close queue. 该节点已被搜索，位于close list中*/
  };
  /**
   * @brief Plan based on 1D Costmap list. Input the index in the costmap and get the plan path. 在1D全局代价地图中规划路径
   * @param start_index start pose index in the 1D costmap list    起点坐标在一维代价地图（list类型）中的索引值
   * @param goal_index goal pose index in the 1D costmap list     终点坐标在一维代价地图（list类型）中的索引值
   * @param path plan path output                                                             输出规划路径
   * @return ErrorInfo which is OK if succeed
   */
  roborts_common::ErrorInfo SearchPath(const int &start_index,
                                     const int &goal_index,
                                     std::vector<geometry_msgs::PoseStamped> &path);
  /**
   * @brief Calculate the cost for the diagonal or parallel movement.   计算对角或平行移动的成本，启发函数h为对角启发
   * @param current_index Index of the current cell as input                     当前坐标索引
   * @param neighbor_index Index of the neighbor cell as input              当前节点的临接点的坐标索引
   * @param move_cost Movement cost as output                                           代价计算
   * @return ErrorInfo which is OK if succeed                                                 计算成功返回success
   */
  roborts_common::ErrorInfo GetMoveCost(const int &current_index,
                                      const int &neighbor_index,
                                      int &move_cost) const;
  /**
   * @brief 计算两个节点之间的曼哈顿距离作为启发式函数的一种A*算法.  （启发函数为曼哈距离）
   * @param index1 Index of the first cell as input
   * @param index2 Index of the second cell as input
   * @param manhattan_distance The Manhattan distance as output            输出为曼哈顿距离
   */
  void GetManhattanDistance(const int &index1,
                            const int &index2,
                            float &manhattan_distance) const;
  /**
   * @brief Get the Euler Distance object
   * 
   * @param index1 index of the first cell as input
   * @param index2 ndex of the second cell as input
   * @param EulerDistance  The EulerDistance distance as output
   */
  void GetEulerDistance(const int &index1,
                                                        const int &index2,
                                                        float &euler_distance) const;
  /**
   * @brief 计算两个点之间的切比雪夫距离
   * @param index1 index of the first cell as input
   * @param index2 index of the second cell as input
   * @param chebyshev_distance The chebyshev distance as output
   */
    void GetChebyshevDistance(const int &index1,
                                                        const int &index2,
                                                        float &chebyshev_distance) const;
      /**
     * @brief 对角启发函数
     * @param index1 index of the first cell as input
     * @param index2 index of the second cell as input
     * @param dialog_distance the dialog_distance
     */
    void GetDialogDistance(const int &index1, const int &index2 ,float &dialog_distance)const;
  /**
   * @brief Get the index of nine neighbor cell from the current cell  从当前单元格中获取9个相邻单元格的索引
   * @param current_index Index of the current cell as input        输入当前位置的坐标索引
   * @param neighbors_index Index of the neighbor cells as out    输出临接节点的坐标索引数组
   */
  void GetNineNeighbors(const int &current_index,
                        std::vector<int> &neighbors_index) const;
  
  void (AStarPlanner::*heuristic_function)(const int &, const int & ,float &)const;


  /**
   * @brief Used for priority queue compare process.
   * @brief 用于优先队列比较  运算符重载，如果index1的预估代价f大于index2的预估代价，返回为true
   */
  struct Compare {
    bool operator()(const int &index1, const int &index2) {
      return AStarPlanner::f_score_.at(index1) > AStarPlanner::f_score_.at(index2);
    }
  };

  //! heuristic_factor_  启发系数
  float heuristic_factor_;
  
  //record the index for start and goal(for Dialog Heuristic)
  int initial_index,terminal_index;
  //heuristic_function_
  std::string heuristic_name;
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
  //! search algorithm related f score, f_score = g_score +  heuristic_cost_estimate 
  static std::vector<int> f_score_;
  //! search algorithm related g score, which refers to the score from start cell to current cell
  std::vector<int> g_score_;
  //! vector that indicates the parent cell index of each cell
  std::vector<int> parent_;
  //! vector that indicates the state of each cell   节点状态表
  std::vector<AStarPlanner::SearchState> state_;
};

std::vector<int> AStarPlanner::f_score_;
roborts_common::REGISTER_ALGORITHM(GlobalPlannerBase,
                                "a_star_planner",
                                AStarPlanner,
                                std::shared_ptr<roborts_costmap::CostmapInterface>);

} //namespace roborts_global_planner

#endif // ROBORTS_PLANNING_GLOBAL_PLANNER_A_STAR_PLANNER_H
