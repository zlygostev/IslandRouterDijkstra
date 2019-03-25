#ifndef __PATH_TIMES_H__
#define __PATH_TIMES_H__

#include "map_types.h"
//#include <unordered_map>
#include <map>

//Keep route with times
struct PathTimes
{
	/// Returns false if no item found. If found returns true and time for get in this point from previous 
	bool extract(const PointT& node, TimeT& time) const
	{
		auto got = m_path.find(node);
		if (got != m_path.end())
		{
			time = got->second;
			return true;
		}
		return false;
	}


	void add(const PointT& pnt, const TimeT& tm)
	{
		m_path[pnt] = tm;
	}

	TimeT getForecastTime() const
	{
		TimeT elapsed = 0.0;
		for (auto node : m_path)
		{
			elapsed += node.second;
		}
		return elapsed;
	}
private:
	// TODO move to unordered_map. It's need hash function for PointT for this
	//std::unordered_map<PointT, TimeT> m_path;
	std::map<PointT, TimeT> m_path;
};

#endif //__PATH_TIMES_H__