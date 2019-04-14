#ifndef __TIME_PREDICTION_H__
#define __TIME_PREDICTION_H__

#define _USE_MATH_DEFINES
#include <maps.h>

#include <math.h>
#include <vector>
#include <utility>
#include <map>
#include <stdint.h>
// Bits used in the overrides image bytes
enum OverrideFlags
{
	OF_RIVER_MARSH = 0x10,
	OF_INLAND = 0x20,
	OF_WATER_BASIN = 0x40
};


/*! It is a class that implements phisical modelling of car movement cross over the land. 
It checks possibility of movement between two neighbor points and calculate time for this operation.
Formula of time calculation for movement between two cells.
I came to formula
t = delta (time_by_plain) / (cos(alpha) * (1 - sin(alpha))
where is alpha = Delta(elevations)/256*(pi/3) and
delta(time_by_plain) = 1 if move straight or sqrt(2)  if move diagonally. delta(time_by_plain) � is a time for move by plain.
See the chart of formula in the file -simulation/time_graph_by_delta_alpha_in_pi.png�.

You can find details of the solution in appropriate methods.
*/
struct EvaluationStategy
{
	EvaluationStategy(const MapExplorer& elevation,
		const MapExplorer& overrides, TimeT unreachableValue) :
		m_elevation(elevation),
		m_overrides(overrides),
		m_unreachable(unreachableValue),
		m_maxHightDiff(0),
		m_maxAngle(0.0)
	{}

	/*!
		Returns possibility to get from one point to another and time for movement.
		Notes: This function measure time between neighbor points.
		\param[in] node of map to check.
		\return flag of drivability of this point.
	*/
	bool isDrivable(const PointT& node) const;

	/*!
		Returns possibility to get from one point to another and time for movement.
		Notes: This function measure time between neighbor points.
		\param[in] from Source point for measure. 
		\param[in] to Destination point for measure.
		\return Pair of flag of drivability between points and in positive case 
			time for get from one point to another.

		A few thinks are base for simulation of time - spent for one move.
			1) No significant elevation changes on path - no time changes.
				I mean if delta(elevation) near 0 - consumed time is near 1.
			2) Let's think about measure elevation changes on path. If elevation changes on path - 
				driving path makes longer too.
				Ok, how measure change of length.If we could assume that road on this range is a line, 
				the new way length is 
					L = delta(l) / cos(alpha).
				Here is  delta(l) is a real length of a cell of the map. And alpha is an angle of road line.
			3) How could measure the time t for path for a new cell: 
					t = L / v
				We know how to measure L and alpha but how to measure speed is still tricky question.
				The gravitation that change speed of vehicle has a force  F = mg * sin(alpha). 
				There is no any info about car mass. There is no any info about power of engine. 
				There is no a lot of another thinks for good physical model. 
				So let's simplify - when angle vertical - the speed is 0 and car is not driving forward. 
				When angle is 0 degree the speed is normal = 1 cell for straight way.
				It's great fit for formula
					v = 1 * (1 - sin(alpha)).
		So, result time calculation is
			t = L / v = delta(l) / (cos(alpha) * (1 - sin(alpha)))
			where is delta(l) = 1 if move straight or sqrt(2)  if move diagonally
		If You want to see tables with values of formula see DeltaTimeByDeltaHigh.xlsx file. 
		We use here delta(L) = delta (H)
	*/
	TimeT getTimeToNeighbour(const PointT& from, const PointT& to) const;

	///	If move straight delta(l) = 1, if move diagonally delta(l) = sqrt(2).
	static double getNeighboursDistance(const PointT& from, const PointT& to);

	/// Returns predefined value of unreachable item
	TimeT unreachable() const {	return m_unreachable;}

	/// It calculates time estimation of the most positive scenario to come from one point to another
	TimeT getMinTimeToArrive(const PointT& from, const PointT& to);
private:


	/** Alpha is an angle of road line, that could be calculated as alpha = arctangent(delta(h) / delta(l)).
		There is no exact data about delta(h) and delta(l). We just know that delta(l) is a some 1 
			unit value and h is measured in some another units.
		That's why I will suggest to calculate  alpha as
		delta(l) = delta(h). 
		I made experiments (see aidTask_pic_results.7z, DeltaTimeByDeltaHigh.xlsx). 
		I like the result formula and penalty for big angles on this map.
		
		But, it also not a bad with delta(l) = delta(h). 
		It something as speed 40 km/h and 8 units for island second. 
		
		Route in this case is trivial - see images (aidTask_pic_results.7z in root directory)
		There is excel file DeltaTimeByDeltaHigh.xlsx with tables by formula Time(dH).
		Look at aidTask_pic_results.7z archive to see routes for different simulation angle calculations.
	*/
	double getAlpha(int16_t dElevation) const;

	// Get Angles array  Ar[dh] = angles
	std::vector<double> getAnglesArray() const;

	static double calculateAlpha(size_t id);

private:
	const MapExplorer& m_elevation;///< info about elevations on map
	const MapExplorer& m_overrides;///< info about ground type
	TimeT m_unreachable; ///< const with value of unreachable destination time
	mutable int8_t m_maxHightDiff; ///< statistic metric for investigation
	mutable double m_maxAngle; ///< statistic metric for investigation
	
};

#endif // __TIME_PREDICTION_H__