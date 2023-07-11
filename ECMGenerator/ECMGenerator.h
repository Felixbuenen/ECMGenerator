#pragma once

#include <memory> // TODO: check of memory inlcuded moet zijn, of kunnen we het forward declaren?

class ECM;
class ECMGraph;
class Environment;
class MedialAxis;

class ECMGenerator
{
	// generates the ECM model

public:
	std::shared_ptr<ECM> GenerateECM(const Environment& environment) const; // takes in a set of points/segments/obstacles and returns the ECM model 
	void AddObstacle(ECM* ecm, int obstacle) const;

private:

	void ConstructECMGraph(ECM& ecm, const Environment& environment) const;

};

