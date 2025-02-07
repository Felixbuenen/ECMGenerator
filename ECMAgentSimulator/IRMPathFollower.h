#pragma once

namespace ECM {

	struct Vec2;
	struct Point;

	class ECM;
	class ECMGraph;

	namespace Simulation {

		struct PathComponent;

		// Reference: https://webspace.science.uu.nl/~gerae101/pdf/fdg09.pdf
		class IRMPathFollower
		{
		public:
			//void Initialize();
			bool FindAttractionPoint(const ECM& ecm, const ECMGraph& ecmGraph, const Point& position, const PathComponent& path, Point& outPoint);
		
		private:
		
		};
	}
}


