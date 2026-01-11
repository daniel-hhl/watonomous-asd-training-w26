#include "planner_core.hpp"
#include <queue>
#include <unordered_map>
#include <cmath>
#include <algorithm>

namespace planner_core
{

    void PlannerCore::setMap(const std::vector<int8_t> &data,
                         int width, int height,
                         double resolution,
                         double origin_x, double origin_y) {
  map_data_ = data;
  width_ = width;
  height_ = height;
  resolution_ = resolution;
  origin_x_ = origin_x;
  origin_y_ = origin_y;
}

bool PlannerCore::isInBounds(const CellIndex &c) const {
  return c.x >= 0 && c.y >= 0 &&
         c.x < width_ && c.y < height_;
}

bool PlannerCore::isFree(const CellIndex &c) const {
  if (!isInBounds(c)) return false;
  int idx = c.y * width_ + c.x;
  int8_t v = map_data_[idx];
  // Adjust threshold if your assignment specifies something else
  return (v >= 0 && v < 50);
}

CellIndex PlannerCore::worldToMap(double wx, double wy) const {
  int ix = static_cast<int>((wx - origin_x_) / resolution_);
  int iy = static_cast<int>((wy - origin_y_) / resolution_);
  return CellIndex(ix, iy);
}

void PlannerCore::mapToWorld(const CellIndex &c, double &wx, double &wy) const {
  wx = origin_x_ + (c.x + 0.5) * resolution_;
  wy = origin_y_ + (c.y + 0.5) * resolution_;
}

double PlannerCore::heuristic(const CellIndex &a, const CellIndex &b) const {
  int dx = a.x - b.x;
  int dy = a.y - b.y;
  return std::sqrt(static_cast<double>(dx*dx + dy*dy));
}

bool PlannerCore::plan(double start_x, double start_y,
                       double goal_x, double goal_y,
                       std::vector<std::pair<double,double>> &out_path_world) {
  out_path_world.clear();

  if (map_data_.empty() || width_ == 0 || height_ == 0) {
    return false;
  }

  CellIndex start = worldToMap(start_x, start_y);
  CellIndex goal  = worldToMap(goal_x, goal_y);

  if (!isFree(start) || !isFree(goal)) {
    return false;
  }

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;

  g_score[start] = 0.0;
  open.emplace(start, heuristic(start, goal));

  const CellIndex neighbors[4] = {
    CellIndex(1,0), CellIndex(-1,0),
    CellIndex(0,1), CellIndex(0,-1)
  };

  bool found = false;

  while (!open.empty()) {
    AStarNode current = open.top();
    open.pop();

    if (current.index == goal) {
      found = true;
      break;
    }

    for (const auto &d : neighbors) {
      CellIndex nb(current.index.x + d.x, current.index.y + d.y);
      if (!isFree(nb)) continue;

      double tentative_g = g_score[current.index] + 1.0;

      auto it = g_score.find(nb);
      if (it == g_score.end() || tentative_g < it->second) {
        g_score[nb] = tentative_g;
        double f = tentative_g + heuristic(nb, goal);
        open.emplace(nb, f);
        came_from[nb] = current.index;
      }
    }
  }

  if (!found) return false;

  // reconstruct in grid space
  std::vector<CellIndex> cells;
  CellIndex cur = goal;
  cells.push_back(cur);
  while (cur != start) {
    cur = came_from[cur];
    cells.push_back(cur);
  }
  std::reverse(cells.begin(), cells.end());

  // convert to world coordinates
  for (const auto &c : cells) {
    double wx, wy;
    mapToWorld(c, wx, wy);
    out_path_world.emplace_back(wx, wy);
  }

  return true;
}

} 
