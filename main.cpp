#include "maps_viewer.h"
#include "model.h"
#include "router.h"

#include <map_types.h>
#include <prioritized_queue.h>
#include <time_prediction.h>
#include <string>
#include <exception>

// Some constants
enum {
    ROVER_X = 159,
    ROVER_Y = 1520,
    BACHELOR_X = 1303,
    BACHELOR_Y = 85,
    WEDDING_X = 1577,
    WEDDING_Y = 1294
};


int main(int argc, char** argv)
{
	try
	{
		std::string pname = argv[0];
		//Initialize and Keep all data about maps
		MapsModel model(pname);

		// Can show data to users
		// Now It's console and pic.bmp
		// See aidTask_pic_results.7z archive to see different routes for different simulation angle calculation
		visualizer::MapsViewer viewer(model);

		// RouteBuilder could be a Controller in MVC architecture
		//Can coordinate and do commands
		RouteBuilder<EvaluationStategy, PrioritizedQueue<TimeT, MeasuredPointT>> router(model, viewer,
																			//elevation, overrides,  //Maps 
																			std::make_pair(ROVER_X, ROVER_Y)); //Start point
		//Drive to me
		if (!router.moveTo(std::make_pair(BACHELOR_X, BACHELOR_Y)))
		{
			throw std::logic_error("Car can't drive to You, sorry");
		}

		//Drive to the wedding
		if (!router.moveTo(std::make_pair(WEDDING_X, WEDDING_Y)))
		{
			throw std::logic_error("Car can't drive You to the wedding, sorry");
		}
		//Show results at pic.bmp and in the console
		router.showRoute();

	}
	catch (const std::exception& ex)
	{
		std::cout << ex.what() << std::endl;
	}
    return 0;
}

