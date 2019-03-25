#ifndef __MAP_TYPES_H__
#define __MAP_TYPES_H__

#include <utility>
#include <vector>

using TimeT = double;

using SpeedT = double;

using PointT = std::pair<int, int>;

using MeasuredPointT = std::pair<PointT, TimeT>;


using NodesT = std::vector<uint8_t>;
#endif // __MAP_TYPES_H__