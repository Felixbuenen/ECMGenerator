#include "Application.h"

#include "ECMDataTypes.h"
#include "Environment.h"

namespace ECM
{
	namespace WindowApplication
	{
		bool Application::InitializeTestEnvironment()
		{
			std::vector<Segment> walkableArea;
			walkableArea.push_back(Segment(-500, -500, 500, -500));
			walkableArea.push_back(Segment(500, -500, 500, 500));
			walkableArea.push_back(Segment(-500, 500, 500, 500));
			walkableArea.push_back(Segment(-500, 500, -500, -500));

			std::vector<Segment> obstacle{
				Segment(-200, -250, -200, 250),
					Segment(-200, 250, 200, 250),
					Segment(200, 250, 200, -250),
					Segment(200, -250, 100, -250),
					Segment(100, -250, 100, 150),
					Segment(100, 150, -100, 150),
					Segment(100, 150, -100, 150),
					Segment(-100, 150, -100, -250),
					Segment(-100, -250, -200, -250)
			};

			Environment env;
			env.AddWalkableArea(walkableArea);
			env.AddObstacle(obstacle);

			return true;
		}

	} // Visualisation
} // ExplicitCorridorMap