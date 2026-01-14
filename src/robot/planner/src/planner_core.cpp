#include "planner_core.hpp"
#include <algorithm>
#include <limits>

namespace planner_core
{

// Helper: Check if cell index is within map bounds
static inline bool inBounds(const nav_msgs::msg::OccupancyGrid &map, const CellIndex &c)
{
  return c.x >= 0 && c.y >= 0 &&
         c.x < static_cast<int>(map.info.width) &&
         c.y < static_cast<int>(map.info.height);
}

// Helper: Convert 2D grid index to flat array index
static inline int flatIndex(const nav_msgs::msg::OccupancyGrid &map, const CellIndex &c)
{
  return c.y * static_cast<int>(map.info.width) + c.x;
}

// Convert world coordinates to grid cell index
CellIndex worldToMap(double x, double y, const nav_msgs::msg::OccupancyGrid &map)
{
  const double res = map.info.resolution;
  const double ox  = map.info.origin.position.x;
  const double oy  = map.info.origin.position.y;
  int mx = static_cast<int>((x - ox) / res);
  int my = static_cast<int>((y - oy) / res);
  return CellIndex(mx, my);
}

// Convert grid cell index to world pose (center of cell)
geometry_msgs::msg::Pose indexToPose(const CellIndex &idx, const nav_msgs::msg::OccupancyGrid &map)
{
  geometry_msgs::msg::Pose pose;
  const double res = map.info.resolution;
  const double ox  = map.info.origin.position.x;
  const double oy  = map.info.origin.position.y;

  // Add 0.5 to get center of cell
  pose.position.x = ox + (static_cast<double>(idx.x) + 0.5) * res;
  pose.position.y = oy + (static_cast<double>(idx.y) + 0.5) * res;
  pose.position.z = 0.0;
  return pose;
}

// Check if a cell is free to navigate
// Unknown cells (-1) are treated as free, occupied cells >= threshold are blocked
bool isFree(const nav_msgs::msg::OccupancyGrid &map, const CellIndex &c, int8_t occ_thresh)
{
  if (!inBounds(map, c)) return false;
  const int idx = flatIndex(map, c);
  const int8_t v = map.data[idx];
  // CRITICAL: Treat unknown (-1) as free, and anything below threshold as free
  return (v < 0) || (v < occ_thresh);
}

// CRITICAL FUNCTION: Find nearest free cell within radius
// This handles cases where goals/starts are clicked in obstacles
bool findNearestFree(const nav_msgs::msg::OccupancyGrid &map,
                     const CellIndex &seed, int radius,
                     CellIndex &out, int8_t occ_thresh)
{
  // First check if seed itself is free
  if (isFree(map, seed, occ_thresh)) {
    out = seed;
    return true;
  }
  
  // Search expanding square rings around seed point
  for (int r = 1; r <= radius; ++r) {
    for (int dy = -r; dy <= r; ++dy) {
      for (int dx = -r; dx <= r; ++dx) {
        // Only check the perimeter of the square ring (not interior)
        if (std::abs(dx) != r && std::abs(dy) != r) continue;
        
        CellIndex c{seed.x + dx, seed.y + dy};
        if (isFree(map, c, occ_thresh)) {
          out = c;
          return true;
        }
      }
    }
  }
  return false;
}

// A* pathfinding algorithm
nav_msgs::msg::Path aStarSearch(const nav_msgs::msg::OccupancyGrid &map,
                                const geometry_msgs::msg::Pose &start_pose,
                                const geometry_msgs::msg::Point &goal_point,
                                int8_t occ_thresh)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = map.header.frame_id;

  // Validate map
  if (map.data.empty() || map.info.width == 0 || map.info.height == 0) {
    return path;
  }

  // Convert world coordinates to grid indices
  const CellIndex start_seed = worldToMap(start_pose.position.x, start_pose.position.y, map);
  const CellIndex goal_seed  = worldToMap(goal_point.x, goal_point.y, map);

  if (!inBounds(map, start_seed) || !inBounds(map, goal_seed)) {
    return path; // out of bounds
  }

  // CRITICAL: Relocate start/goal to nearest free cell within 3-cell radius
  // This prevents planning failures when user clicks near/on obstacles
  CellIndex start = start_seed, goal = goal_seed;
  if (!findNearestFree(map, start_seed, 10, start, occ_thresh)) return path;
  if (!findNearestFree(map, goal_seed,  3, goal,  occ_thresh)) return path;

  // A* data structures
  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;  // Parent tracking
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;       // Cost from start
  std::unordered_set<int> closed; // Nodes already processed (by flat index)

  // Heuristic function: Euclidean distance in grid space
  auto heuristic = [&](const CellIndex &a, const CellIndex &b) {
    return std::hypot(double(a.x - b.x), double(a.y - b.y));
  };

  // Initialize start node
  g_score[start] = 0.0;
  open_set.emplace(start, heuristic(start, goal));

  const double SQRT2 = std::sqrt(2.0);
  
  struct Neighbor {
    CellIndex offset;
    double cost;
  };
  
  const Neighbor neighbors[8] = {
    // Cardinal directions (cost = 1.0)
    {{1, 0}, 1.0},   // right
    {{-1, 0}, 1.0},  // left
    {{0, 1}, 1.0},   // up
    {{0, -1}, 1.0},  // down
    // Diagonal directions (cost = √2)
    {{1, 1}, SQRT2},   // up-right
    {{1, -1}, SQRT2},  // down-right
    {{-1, 1}, SQRT2},  // up-left
    {{-1, -1}, SQRT2}  // down-left
  };

  // Main A* loop
  while (!open_set.empty())
  {
    const CellIndex current = open_set.top().index;
    open_set.pop();

    // Check if we reached the goal
    if (current == goal) {
      break; // found a route to goal
    }

    // Skip if already processed (CRITICAL: prevents infinite loops)
    const int cFlat = flatIndex(map, current);
    if (closed.count(cFlat)) continue;
    closed.insert(cFlat);

    // Explore all 8 neighbors
    for (const auto &neighbor : neighbors)
    {
      CellIndex nb{current.x + neighbor.offset.x, current.y + neighbor.offset.y};
      if (!inBounds(map, nb)) continue;
      if (!isFree(map, nb, occ_thresh)) continue;

      double move_cost = neighbor.cost;
      double tentative_g = g_score[current] + move_cost;

      // Update if this is a better path to neighbor
      auto it = g_score.find(nb);
      if (it == g_score.end() || tentative_g < it->second) {
        came_from[nb] = current;
        g_score[nb] = tentative_g;
        const double f = tentative_g + heuristic(nb, goal);
        open_set.emplace(nb, f);
      }
    }
  }

  // Reconstruct path by following parent pointers backwards
  if (came_from.find(goal) == came_from.end() && !(start == goal)) {
    // Could not reach goal - return empty path
    return path;
  }

  // Build reverse path from goal to start
  std::vector<CellIndex> rev;
  CellIndex cur = goal;
  rev.push_back(cur);
  while (cur != start) {
    auto it = came_from.find(cur);
    if (it == came_from.end()) break;
    cur = it->second;
    rev.push_back(cur);
  }
  std::reverse(rev.begin(), rev.end());

  // Convert grid indices to world coordinate poses
  path.poses.reserve(rev.size());
  for (const auto &c : rev) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = map.header.frame_id;
    ps.pose = indexToPose(c, map);
    path.poses.push_back(ps);
  }

  return path;
}

} // namespace planner_core