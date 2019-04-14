#ifndef __MAP_TYPES_H__
#define __MAP_TYPES_H__

#include <utility>
#include <vector>

/// This type is used over all app for measure time with appropriate precision. 
using TimeT = double;

/// This type is used over all app for measure time with appropriate precision.
using SpeedT = double;

/// This type determines Point coordinates on map.
using 
PointT = std::pair<int, int>;

/// This type keep timepoint associated with coordinates on map.
using MeasuredPointT = std::pair<PointT, TimeT>;


using NodesT = std::vector<uint8_t>;
#endif // __MAP_TYPES_H__