#ifndef __ROUTER_H__
#define __ROUTER_H__

#include <utility>
#include <list>

#include <limits>
#include <exception>

#include <maps.h>
#include <path.h>
#include "model.h"
#include "maps_viewer.h"

/*!
This class implements logic of optimal path build.\

The idea is simple
1)	calculate path from the Current point to a new destination
2)  Save results in path history
3)	In our case it loop of 2 iterations by 1) and 2)
4)	Result time is the sum of times on two steps. The result path is composition of both paths.
The main idea of path calculation algorithm is Dijkstra. 
But it is implemented an optimization of Dijkstra - Algorithm A*.
It decreases number of processed nodes on 30%.
A bit details of optimization:
The priority in the queue is enlarged by value of minimal time to arrive from the current point to the finish.
See time_prediction.h for more details of time estimation implementation.
  
Algorithm:
There is a graph as a map with understandable connections between nearest cells.  
The weight of path between two nearest cells could evaluated by formula of time calculation (look at time_prediction.h).
Algorithm is a loop by the queue of cells.
The first cell in the queue is the start point(node) for the algorithm. 
Every node is processed by calculation of time from current cell to neighbors. 
If time to arrive an neighbor is changed to lower, neighbor is add to the processing queue. 
The default values to arrive each neighbor is NaN. 
Arriving time to start point is 0. 
There is no need any "visited nodes list". The Fact of visiting is shown well on m_timeToArrive map.
The next cell for calculation is get from queue by rule that it has the lowest number of prioity in queue.
The algorithm is stop to work if
1)	the queue processing is over
2)	Destination point has value to arrive point that is lower than any value of neighbors in queue.
There is no path to destination point if the value of it DEAFAULT after finish of the queue processing.
*/
template<typename SimulationT, typename QueueT>
struct RouteBuilder
{	
	static const double UNREACHABLE; // is NaN

	bool isUnreachable(const TimeT& val) const 
	{
		//IEEE has a rule for for std::numeric_limits<double>::quiet_NaN : 
		//		if i is Nan than i != i
		return val != val;
	}

	RouteBuilder(MapsModel& model, visualizer::MapsViewer& viewer, const PointT& start):
		m_model(model),
		m_viewer(viewer),
		m_timeToArrive(model.getSizeX(), model.getSizeY(), UNREACHABLE,
						std::function<bool(const TimeT&)>([](TimeT t) {return t != t; })), // t is Nan if t != t
		m_simEngine(model.elevation(), model.overrides(), UNREACHABLE),
		m_totalCheckedItems(0),
		m_enquedItems(0),
		m_cutted(0)
	{
		m_baseRoutePoints.push_back(start);
	}

	/*! Build path to a new point on the map
		\return true if a path is found. In other cases returns false.
	*/
	bool moveTo(const PointT& finishPnt)
	{
		m_timeToArrive.reset();
		if (!m_simEngine.isDrivable(finishPnt))
		{
			return false;
		}

		const auto& curLocation = m_baseRoutePoints.back();
		m_timeToArrive.put(curLocation, 0);
		// Key of the queue - is a priority. It measure minimum estimated time of arrival through this point to the finish
		// Value of the queue is a pair with a Point and time to arrive from start to this point
		QueueT queue; 
		TimeT timeToPoint = 0.0;
		auto minTimeToArrive = m_simEngine.getMinTimeToArrive(curLocation, finishPnt);
		queue.push(timeToPoint + minTimeToArrive, std::make_pair(curLocation, timeToPoint));
		while (!queue.empty() && needProcessQueue(m_timeToArrive.get(finishPnt), queue.front().first))
		{
			m_enquedItems += 1;
			auto curNode = queue.front().second;
			auto& curPoint = curNode.first;
			auto& timeToPoint = curNode.second;
			queue.pop();
			auto curValue = m_timeToArrive.get(curPoint);
			// If the node is processed and has a lower time value - skip it
			if (isLower(curValue, timeToPoint))
			{
				continue;
			}
			m_timeToArrive.put(curPoint, timeToPoint);
			std::list<PointT> neighbors = m_timeToArrive.getNeighbors(curPoint);
			processNeighbors(curPoint, timeToPoint, neighbors, finishPnt, queue);
		}
		m_cutted += queue.size();

		// Check if path is not found
		if (isUnreachable(m_timeToArrive.get(finishPnt)))
		{
			return false;
		}
		//form result path
		auto path = extractPath(curLocation, finishPnt);
		for (auto node : path)
		{
			m_path.add(node.first, node.second);
		}
		//Add stop point Finish Point
		m_baseRoutePoints.push_back(finishPnt);

		return true;

	}

	/// \return detailed path with points and estimation time for drive though each one.
	void showRoute()
	{
		m_viewer.showRoute(m_path, m_baseRoutePoints);
	}
private:
	/* neighbors of current point are add in the queue if it is necessary */
	void processNeighbors(const PointT& curPoint, const TimeT& timeToPoint, const std::list<PointT>& neighbors, const PointT& finishPoint,  QueueT& queue)
	{
		// Let's enqueue neighbors
		for (auto& neighbor : neighbors)
		{
			m_totalCheckedItems += 1;
			// Skip not drivable neighbors
			if (!m_simEngine.isDrivable(neighbor))
			{
				continue;
			}

			TimeT timeForMove = m_simEngine.getTimeToNeighbour(curPoint, neighbor);
			if (isUnreachable(timeForMove))
			{
				continue;
			}
			if (timeForMove <= 0)
			{
				throw std::logic_error("Time to move from one point to another should be > 0");
			}

			auto newTime = timeToPoint + timeForMove;
			auto oldTime = m_timeToArrive.get(neighbor);
			// If time in neighbor node is lower newTime, skip this node
			if (isLower(oldTime, newTime))
			{
				continue;
			}

			m_timeToArrive.put(neighbor, newTime);
			// if Node is already has a value in queue it could be removed (timeFromMap, pnt). But it will fast skipped. So keep code simple
			auto minTimeToArrive = m_simEngine.getMinTimeToArrive(neighbor, finishPoint);
			queue.push(newTime + minTimeToArrive, std::make_pair(neighbor, newTime));
		}
	}
	
	bool findMinValuePoint(const std::list<PointT>& neighbors, PointT& minPoint, TimeT& minValue)
	{
		if (neighbors.empty())
			return false;

		minValue = UNREACHABLE;
		for (auto neighbor : neighbors)
		{
			auto value = m_timeToArrive.get(neighbor);
			if (isUnreachable(minValue))
			{
				minValue = value;
				minPoint = neighbor;
				continue;
			}

			//get min and not NaN
			if (isUnreachable(value))
			{
				continue;
			}

			if (value < minValue)
			{
				minValue = value;
				minPoint = neighbor;
			}
		}
		return !isUnreachable(minValue);
	}

	std::list<std::pair<PointT, SpeedT>> extractPath(const PointT& startPoint, const PointT& finishPnt)
	{
		std::list<std::pair<PointT, TimeT>> path; //or deque. But we don't know items count.
		auto curPoint = finishPnt;
		auto curTime = m_timeToArrive.get(finishPnt);
		PointT nextPoint;
		TimeT nextTime;
		while (curPoint != startPoint)
		{
			std::list<PointT> neighbors = m_timeToArrive.getReachableNeighbors(curPoint);//m_timeToArrive.getReachableNeighbors(curPoint);
			//This simple solution could cut edges of route but doesn't impact on time estimations.
			if (!findMinValuePoint(neighbors, nextPoint, nextTime))
			{
				throw std::logic_error("Can't extract path from backward iteration by valued map from finish point");
			}
			if (nextTime >= curTime)
			{
				throw std::logic_error("Can't create a back way to start point. New points min time is greater then in current point. It's impossible");
			}
			const auto dT = (curTime - nextTime);
			const auto speed = dT;//* 1.0 / m_simEngine.getNeighboursDistance(nextPoint, curPoint);
			path.push_front(std::make_pair(curPoint, speed));
			curPoint = nextPoint;
			curTime = nextTime;
		}
		return path;
	}

	bool isLower(const TimeT& val1, const TimeT& val2)
	{
		return !isUnreachable(val1) && (val1 < val2);
	}

	bool needProcessQueue(const TimeT& value, const TimeT& minQueue)
	{
		if (isUnreachable(value))
		{
			return true;
		}
		if (isUnreachable(minQueue))
		{
			throw std::logic_error("There is NaN in processing queue. Something going wrong.");
			return true;//sure?
		}

		return (value > minQueue);
	}

private:
	MapsModel& m_model;// Source of all start data about the Island
	visualizer::MapsViewer& m_viewer; // can show data to user
	RWMap m_timeToArrive;//<Map with the minimal time to arrive to  a node from start point
	SimulationT m_simEngine;//< Simulation of move between points
	std::list<PointT> m_baseRoutePoints;//< stop points
	PathTimes m_path;//< All point of route with elapsed time for each point
	size_t m_totalCheckedItems;//statistic
	size_t m_enquedItems;//statistic
	size_t m_cutted;//statistic
};

template<typename SimulationT, typename QueueT> 
const double RouteBuilder<SimulationT, QueueT>::UNREACHABLE = std::numeric_limits<double>::quiet_NaN();
#endif // __ROUTER_H__