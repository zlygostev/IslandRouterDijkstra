#include "maps.h"
#include <algorithm>

void BaseMap::checkBoundaries(const PointT& pnt) const
{
	if (pnt.first >= m_sizeX)
		throw std::out_of_range("Requested X coordinate out of range");

	if (pnt.second >= m_sizeY)
		throw std::out_of_range("Requested Y coordinate out of range");

}

std::list<PointT> BaseMap::getNeighbors(const PointT& pnt) const
{
	bool hasLowerNodes = (pnt.second > 0);
	bool hasLeftNodes = (pnt.first > 0);
	bool hasUpperNodes = (pnt.second < m_sizeY - 1);
	bool hasRightNodes = (pnt.first < m_sizeY - 1);
	std::list<PointT> result;
	if (hasLeftNodes)
	{
		if (hasLowerNodes)
		{
			result.push_back(std::make_pair(pnt.first - 1, pnt.second - 1));
		}
		result.push_back(std::make_pair(pnt.first - 1, pnt.second));
		if (hasUpperNodes)
		{
			result.push_back(std::make_pair(pnt.first - 1, pnt.second + 1));
		}
	}

	if (hasRightNodes)
	{
		if (hasLowerNodes)
		{
			result.push_back(std::make_pair(pnt.first + 1, pnt.second - 1));
		}
		result.push_back(std::make_pair(pnt.first + 1, pnt.second));
		if (hasUpperNodes)
		{
			result.push_back(std::make_pair(pnt.first + 1, pnt.second + 1));
		}
	}

	if (hasLowerNodes)
	{
		result.push_back(std::make_pair(pnt.first, pnt.second - 1));
	}
	if (hasUpperNodes)
	{
		result.push_back(std::make_pair(pnt.first, pnt.second + 1));
	}
	return result;

}

std::list<PointT> RWMap::getReachableNeighbors(PointT& pnt) const
{
	auto result = BaseMap::getNeighbors(pnt);
	auto removeWithDefaults = [&](const PointT& val) -> bool {
									return m_isDefault(get(val));
								};
	auto iterator = std::remove_if(result.begin(), result.end(), removeWithDefaults);
	result.erase(iterator, result.end());
	return result;
}

