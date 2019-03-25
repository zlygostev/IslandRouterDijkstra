#ifndef __MAPS_H__
#define __MAPS_H__

#include "map_types.h"
#include <vector>
#include <list>
#include <functional>

struct BaseMap
{
	BaseMap(size_t sizeX, size_t sizeY) : m_sizeX(sizeX), m_sizeY(sizeY)
	{}

	size_t sizeX() const { return m_sizeX;}

	size_t sizeY() const { return m_sizeY;}

	bool isOutOfRange(const PointT& pnt) const { return  (pnt.second >= m_sizeY) && (pnt.first >= m_sizeX);}

	void checkBoundaries(const PointT& pnt) const;

	std::list<PointT> getNeighbors(const PointT& pnt) const;

protected:
	const size_t m_sizeX;
	const size_t m_sizeY;
};

// It's a wrapper with additional checks and functionality over a input NodesT map data
struct MapExplorer: public BaseMap
{
	MapExplorer(const NodesT&& points, size_t sizeX, size_t sizeY) : 
		BaseMap(sizeX, sizeY),
		m_nodes(std::move(points))
	{
	}

	uint8_t get(const PointT& pnt) const
	{
		checkBoundaries(pnt);
		return m_nodes[m_sizeX * pnt.second + pnt.first];
	}

	const uint8_t* rawData() { return &m_nodes[0]; }

private:
	const NodesT m_nodes;
};

/// Map for route building
struct RWMap: public BaseMap
{
	RWMap(size_t sizeX, size_t sizeY, TimeT defaultValue, std::function<bool(const TimeT&)> isDefault) :
											BaseMap(sizeX, sizeY),
											m_defaultValue(defaultValue),
											m_isDefault(isDefault),
											m_map(sizeX * sizeY, defaultValue)
	{}

	TimeT get(const PointT& pnt) const
	{
		checkBoundaries(pnt);
		return m_map[m_sizeX * pnt.second + pnt.first];
	}

	void put(const PointT& pnt, const TimeT& val)
	{
		checkBoundaries(pnt);
		m_map[m_sizeX * pnt.second + pnt.first] = val;
	}

	std::list<PointT> getReachableNeighbors(PointT& pnt) const;

	void reset() { m_map.assign(m_map.size(), m_defaultValue);}
private:
	const TimeT m_defaultValue;
	std::function<bool(TimeT)> m_isDefault;
	std::vector<TimeT> m_map;
};
#endif // __MAPS_H__