#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include <vector>
#include <utility>
#include <cstdint>

namespace planner_core
{
// 2D grid index
struct CellIndex
{
  int x;
  int y;
 
  CellIndex(int xx, int yy) : x(xx), y(yy) {}
  CellIndex() : x(0), y(0) {}
 
  bool operator==(const CellIndex &other) const
  {
    return (x == other.x && y == other.y);
  }
 
  bool operator!=(const CellIndex &other) const
  {
    return (x != other.x || y != other.y);
  }
};

// Hash function for CellIndex so it can be used in std::unordered_map
struct CellIndexHash
{
  std::size_t operator()(const CellIndex &idx) const
  {
    // A simple hash combining x and y
    return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
  }
};
 
// Structure representing a node in the A* open set
struct AStarNode
{
  CellIndex index;
  double f_score;  // f = g + h
 
  AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};
 
// Comparator for the priority queue (min-heap by f_score)
struct CompareF
{
  bool operator()(const AStarNode &a, const AStarNode &b)
  {
    // We want the node with the smallest f_score on top
    return a.f_score > b.f_score;
  }
};

class PlannerCore {
public:
  PlannerCore() = default;

  void setMap(const std::vector<int8_t> &data,
              int width, int height,
              double resolution,
              double origin_x, double origin_y);

  bool plan(double start_x, double start_y,
            double goal_x, double goal_y,
            std::vector<std::pair<double,double>> &out_path_world);

private:
  std::vector<int8_t> map_data_;
  int width_ = 0;
  int height_ = 0;
  double resolution_ = 0.0;
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;

  bool isInBounds(const CellIndex &c) const;
  bool isFree(const CellIndex &c) const;
  CellIndex worldToMap(double wx, double wy) const;
  void mapToWorld(const CellIndex &c, double &wx, double &wy) const;
  double heuristic(const CellIndex &a, const CellIndex &b) const;
};

}  

#endif  
