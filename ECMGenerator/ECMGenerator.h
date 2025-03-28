#pragma once

#include <memory>

namespace ECM {

	class ECM;
	class ECMGraph;
	class Environment;
	class MedialAxis;

	struct Point;

	class ECMGenerator
	{
		// generates the ECM model

	public:
		static ECM* GenerateECM(const Environment& environment, ECM* ecm = nullptr); // takes in a set of points/segments/obstacles and returns the ECM model 

	private:

		static void ConstructECMGraph(ECM& ecm, const Environment& environment);
		static void GetClosestPointsToSource(const Environment& environment, int sourceIdx, const Point& p1, const Point& p2, bool isPoint, bool isStartPoint, Point& outClosestP1, Point& outClosestP2);

	};

}