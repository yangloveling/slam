#pragma once

#include <array>
#include <utility>
#include <vector>

namespace robot_common
{
namespace r2
{

struct Block
{
  int id;
  double x;
  double y;
  double h;
};

inline const std::array<Block, 17> & blocks()
{
  static const std::array<Block, 17> value{{
    {0, 2.160, -1.650, 0.00},
    {1, 3.430, -0.450, 0.40},
    {2, 3.430, -1.650, 0.20},
    {3, 3.430, -2.850, 0.40},
    {4, 4.630, -0.450, 0.60},
    {5, 4.630, -1.650, 0.40},
    {6, 4.630, -2.850, 0.20},
    {7, 5.830, -0.450, 0.40},
    {8, 5.830, -1.650, 0.60},
    {9, 5.830, -2.850, 0.40},
    {10, 7.030, -0.450, 0.20},
    {11, 7.030, -1.650, 0.40},
    {12, 7.030, -2.850, 0.20},
    {13, 8.350, -0.450, 0.00},
    {14, 8.350, -1.650, 0.00},
    {15, 8.350, -2.850, 0.00},
    {16, 8.350, -3.950, 0.00},
  }};

  return value;
}

inline bool get_block(int id, Block & out)
{
  if (id < 0 || id >= static_cast<int>(blocks().size())) {
    return false;
  }

  out = blocks()[static_cast<std::size_t>(id)];
  return out.id == id;
}

inline bool get_block_center(int id, double & x, double & y)
{
  Block block{};
  if (!get_block(id, block)) {
    return false;
  }

  x = block.x;
  y = block.y;
  return true;
}

inline bool get_block_height(int id, double & h)
{
  Block block{};
  if (!get_block(id, block)) {
    return false;
  }

  h = block.h;
  return true;
}

inline std::vector<int> exit_blocks()
{
  return {16};
}

inline const std::array<std::pair<int, int>, 1> & exit_approach_edges()
{
  static const std::array<std::pair<int, int>, 1> value{{
    {15, 16},
  }};

  return value;
}

}  // namespace r2
}  // namespace robot_common
