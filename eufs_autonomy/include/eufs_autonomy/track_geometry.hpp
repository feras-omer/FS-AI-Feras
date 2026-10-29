#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <vector>

// Track geometry shared by the local centerline (lap 1) and the global
// path planner (lap 2+). All points are 2D in the map frame.
namespace track_geometry
{

using Points = std::vector<Eigen::Vector2d>;

// Midpoints between each cone and the nearest cone of the other colour.
// Midpoints closer than merge_dist are averaged into one.
inline Points pairMidpoints(
  const Points &left, const Points &right,
  double max_width, double merge_dist = 1.0)
{
  Points mids;

  auto add_pairs = [&](const Points &from, const Points &to)
  {
    for (const auto &a : from)
    {
      const Eigen::Vector2d *best = nullptr;
      double best_d = max_width;
      for (const auto &b : to)
      {
        double d = (a - b).norm();
        if (d < best_d)
        {
          best_d = d;
          best = &b;
        }
      }
      if (best)
        mids.push_back(0.5 * (a + *best));
    }
  };

  add_pairs(left, right);
  add_pairs(right, left);

  Points merged;
  std::vector<int> counts;
  for (const auto &m : mids)
  {
    bool found = false;
    for (size_t i = 0; i < merged.size(); ++i)
    {
      if ((merged[i] - m).norm() < merge_dist)
      {
        merged[i] = (merged[i] * counts[i] + m) / (counts[i] + 1);
        counts[i]++;
        found = true;
        break;
      }
    }
    if (!found)
    {
      merged.push_back(m);
      counts.push_back(1);
    }
  }
  return merged;
}

// Greedy walk from `start` along `heading`: repeatedly step to the nearest
// unused point that is within max_step and no more than max_turn (rad) off
// the current direction. The first step may be up to max_first_step, since
// cones right beside the car are outside the camera view and never coloured.
// Stops when no point qualifies or the walked length exceeds max_length.
inline Points orderPath(
  const Points &points, const Eigen::Vector2d &start, double heading,
  double max_step, double max_turn, double max_first_step,
  double max_length = 1e9)
{
  Points path;
  std::vector<bool> used(points.size(), false);

  Eigen::Vector2d cur = start;
  Eigen::Vector2d dir(std::cos(heading), std::sin(heading));
  double length = 0.0;

  while (length < max_length)
  {
    int best = -1;
    double best_cost = 1e9;
    for (size_t i = 0; i < points.size(); ++i)
    {
      if (used[i])
        continue;
      Eigen::Vector2d d = points[i] - cur;
      double dist = d.norm();
      if (dist < 1e-3 || dist > (path.empty() ? max_first_step : max_step))
        continue;
      double turn = std::acos(std::clamp(d.dot(dir) / dist, -1.0, 1.0));
      if (turn > max_turn)
        continue;
      // Prefer close points, penalise turning
      double cost = dist * (1.0 + turn);
      if (cost < best_cost)
      {
        best_cost = cost;
        best = static_cast<int>(i);
      }
    }
    if (best < 0)
      break;

    used[best] = true;
    Eigen::Vector2d step = points[best] - cur;
    length += step.norm();
    dir = step.normalized();
    cur = points[best];
    path.push_back(cur);
  }
  return path;
}

}  // namespace track_geometry
