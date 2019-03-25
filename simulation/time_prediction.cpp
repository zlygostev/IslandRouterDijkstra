#include "time_prediction.h"
#include <cmath>
#include <algorithm>


bool EvaluationStategy::isDrivable(const PointT& node) const
{
	if (m_overrides.isOutOfRange(node))
		return false;

	auto flags = m_overrides.get(node);
	bool isNotDrivable = (m_overrides.get(node) & (OF_WATER_BASIN | OF_RIVER_MARSH));
	//There is no information in the rules that zero evaluation of lend is forbidden to drive there. 
	//There is separated flags for this case in m_overrides for this information.
	// But check if elevation is 0 and ground is marked as drivable - set it as indrivable to reduce risks of route
	if (!isNotDrivable && m_elevation.get(node) <= 0)
	{
		isNotDrivable = true;
	}
	return !isNotDrivable;
}

TimeT EvaluationStategy::getTimeToNeighbour(const PointT& from, const PointT& to) const
{
	//check if destination node is reachable
	if (!isDrivable(to))
	{
		return unreachable();
	}

	const double distance = getNeighboursDistance(from, to);
	const int16_t deltaH = m_elevation.get(to) - m_elevation.get(from);
	auto key = std::make_pair(deltaH, distance);
	//it could be optimized by use of unordered_map. But we also need to add some hash function.
	static std::map<std::pair<int16_t, double>, TimeT> timesCache; //thread unsafe trick
	auto it = timesCache.find(key);
	if (it != timesCache.end())
	{
		return it->second;
	}
	const double alpha = getAlpha(deltaH);
	const auto tm = distance / cos(alpha) / (1 - sin(alpha));
	timesCache[key] = tm;
	return tm;
}

double lowestTimeCorrection()
{
	// It is the minimum of getTimeToNeighbour .
	// I hardcode it by time optimization reasons.
	return 0.75;
}

double EvaluationStategy::getNeighboursDistance(const PointT& from, const PointT& to)
{
	int64_t dX = to.first - from.first;
	int64_t dY = to.second - from.second;
	if (from == to)
	{
		return 0.0;
	}

	if ((abs(dY) > 1) || (abs(dX) > 1))
	{
		throw std::out_of_range("requested for measure node is not a neighbor");
	}

	if ((abs(dY) == 1) && (abs(dX) == 1))
	{
		return sqrt(2.0);
	}
	return 1.0;
}

namespace 
{
template <typename T> int sign(T val) 
{
	return (T(0) < val) - (val < T(0));
}
}

double EvaluationStategy::getAlpha(int16_t dElevation) const
{
	static const std::vector<double> anglesArray = EvaluationStategy::getAnglesArray(); //thread unsafe
	const int sgn = sign(dElevation);
	const uint8_t index = static_cast<uint8_t>(abs(dElevation));
	if (index > m_maxHightDiff) 
	{
		m_maxHightDiff = index;
		m_maxAngle = anglesArray[index]/ M_PI*180.0;
	}
	return sgn * anglesArray[index];
}

std::vector<double> EvaluationStategy::getAnglesArray() const
 {
	constexpr size_t Values = 256;
	std::vector<double> angles(Values, 0);
	for (size_t id = 0; id < Values; ++id)
	{
		angles[id] = EvaluationStategy::calculateAlpha(id);
	}
	return angles;
}

double EvaluationStategy::calculateAlpha(size_t id)
{
	// See aidTask_pic_results.7z archive to see different routes for different simulation angle calculation

	//See declaration of getAlpha functions for details
	return atan(static_cast<double>(id) / 1.0);

}

TimeT EvaluationStategy::getMinTimeToArrive(const PointT& from, const PointT& to)
{
	auto dX = abs(from.first - to.first);
	auto dY = abs(from.second - to.second);
	auto diagonal = std::min(dX, dY);
	auto straight = abs(dX - dY);
	auto planeTime = 1.0*straight + sqrt(2.0)*diagonal;
	return planeTime * lowestTimeCorrection();
}