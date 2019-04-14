#ifndef __VIEWER_H__
#define __VIEWER_H__

#include "path.h"
#include "visualizer.h"
#include "model.h"

#include <vector>
#include <functional>
#include <ostream>

namespace visualizer 
{

/// Rander map with route to a BMP file (pic.bmp)
struct MapsViewer 
{
	MapsViewer(MapsModel& model): model(model)
	{
	}

	/// Rander map with route to a BMP file (pic.bmp)
	void showRoute(const PathTimes& path, const std::list<PointT>& basePoints)
	{
		auto forecastTime = path.getForecastTime();
		std::cout << "Time forecast for the trip is: " << forecastTime << " island seconds" << std::endl;

		// Show results on view
		std::ofstream of("pic.bmp", std::ofstream::binary);

		visualizer::writeBMP(
			of,
			model.elevation().rawData(),
			model.getSizeX(),
			model.getSizeY(),
			[&](int x, int y, uint8_t elevation) 
			{
				// Marks interesting positions on the map
				if (donut(x, y, basePoints))
				{
					return static_cast<uint8_t>(visualizer::IPV_PATH);
				}

				//Show Path dots
				PointT point(x, y);
				uint8_t pathType = static_cast<uint8_t>(visualizer::IPV_ELEVATION_BEGIN);
				if (extactPathColor(path, model.overrides(), point, pathType))
				{
					return pathType;
				}

				// Signifies water
				if ((model.overrides().get(std::make_pair(x, y)) & (OF_WATER_BASIN | OF_RIVER_MARSH)) ||
					elevation == 0)
				{
					return static_cast<uint8_t>(visualizer::IPV_WATER);
				}

				// Signifies normal ground color
				if (elevation < visualizer::IPV_ELEVATION_BEGIN)
				{
					elevation = static_cast<uint8_t>(visualizer::IPV_ELEVATION_BEGIN);
				}
				return elevation;
			}
		);
		of.flush();
#if __APPLE__
		auto res = system("open pic.bmp");
		(void)res;
#endif
	}
private:
	static bool donut(int x, int y, const std::list<PointT>& basePoints)
	{
		for(auto& pnt: basePoints)
		{
			if (donut(x, y, pnt.first, pnt.second))
				return true;
		}
		return false;
	}

	static bool donut(int x, int y, int x1, int y1)
	{
		int dx = x - x1;
		int dy = y - y1;
		int r2 = dx * dx + dy * dy;
		return r2 >= 150 && r2 <= 400;
	}


	static bool extactPathSurroundings(const PathTimes& path, const MapExplorer& map, const PointT& point, TimeT& tm)
	{
		int pathType = visualizer::IPV_ELEVATION_BEGIN;
		if (path.extract(point, tm))
		{
			return true;
		}
		auto neigbList = map.getNeighbors(point);
		for (auto& pnt : neigbList)
		{
			if (path.extract(pnt, tm))
			{
				return true;
			}
		}
		return false;
	}

	static bool extactPathColor(const PathTimes& path, const MapExplorer& map, const PointT& point, uint8_t& pathType)
	{
		pathType = visualizer::IPV_ELEVATION_BEGIN;
		TimeT tm; //Time elapsed for go throughTimePoint
		if (extactPathSurroundings(path, map, point, tm))
		{
			if (tm < 0.95)
			{
				pathType = static_cast<uint8_t>(visualizer::IPV_QUICK_PATH);
				return true;
			}
			if (tm < 2)
			{
				pathType = static_cast<uint8_t>(visualizer::IPV_NORMAL_PATH);
				return true;
			}
			if (tm < 4)
			{
				pathType = static_cast<uint8_t>(visualizer::IPV_SLOW_PATH);
				return true;
			}
			if (tm >= 4)
			{
				pathType = static_cast<uint8_t>(visualizer::IPV_TOO_SLOW_PATH);
				return true;
			}

		}
		return false;
	}
private:
	MapsModel & model;
};
}
#endif // __ROUTER_H__