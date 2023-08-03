#pragma once

#include <memory> // TODO: check of memory inlcuded moet zijn, of kunnen we het forward declaren?

namespace ECM {

	class ECM;
	class ECMGraph;
	class Environment;
	class MedialAxis;

	class ECMGenerator
	{
		// generates the ECM model

	public:
		static std::shared_ptr<ECM> GenerateECM(const Environment& environment); // takes in a set of points/segments/obstacles and returns the ECM model 
		static void AddObstacle(ECM* ecm, int obstacle);

	private:

		static void ConstructECMGraph(ECM& ecm, const Environment& environment);

	};

}