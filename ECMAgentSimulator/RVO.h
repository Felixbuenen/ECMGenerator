#pragma once

#include "ECMDataTypes.h"
#include "UtilityFunctions.h"

#include "Simulator.h"

#include <vector>

namespace ECM {

	namespace Simulation {

		class Simulator;

		struct PositionComponent;
		struct VelocityComponent;
		struct ClearanceComponent;

		// ORCA: https://gamma.cs.unc.edu/ORCA/publications/ORCA.pdf
		// The white paper contains a clear description of the model, with conservative collision time as the selector condition, as well as the adaptation to a
		// shape-agnostic implementation. I also found it interesting to implement the 3 described avoid-exceptions to create more dynamic / realistic behavior for
		// game scenarios.

		// A constraint in ORCA is defined by a half-plane. We represent the half-plane line by a point on this line. The direction of the half-plane is indicated by the normal N.
		class Constraint
		{
		public:
			void Init(const Point& pointOnLine, const Vec2& normal)
			{
				m_N = normal;
				m_PointOnLine = pointOnLine;

				m_isVertical = (normal.y < Utility::EPSILON) && (normal.y > -Utility::EPSILON);

				// check if the line is vertical
				if (m_isVertical)
				{
					m_xIntercept = pointOnLine.x;
				}
				else
				{
					m_Slope = -(normal.x / normal.y);
					m_yIntercept = pointOnLine.y - m_Slope * pointOnLine.x;
					m_xIntercept = m_yIntercept / m_Slope;
				}
			}

			bool Contains(const Point& p) const
			{
				return Utility::MathUtility::Dot(m_N, (p - m_PointOnLine)) >= 0;
			}

			void SetNormal(const Vec2& normal) { m_N = normal; }
			void SetPointOnLine(const Point& pointOnLine) { m_PointOnLine = pointOnLine; }

			inline Vec2 Normal() const { return m_N; }
			inline Point PointOnLine() const { return m_PointOnLine; }
			inline float Slope() const { return m_Slope; }
			inline float YIntercept() const { return m_yIntercept; }
			inline float XIntercept() const { return m_xIntercept; }
			inline bool IsVertical() const { return m_isVertical; }


		private:
			Vec2 m_N;
			Point m_PointOnLine;
			float m_Slope;
			float m_xIntercept;
			float m_yIntercept;
			bool m_isVertical;
		};

		// TODO:
		// - change class name/script name to ORCA
		// - static obstacles are not yet taken into account; implement
		class RVO 
		{
		public:

			// get the RVO velocities based on the given simulator object
			void GetRVOVelocity(Simulator* simulator, const Entity& entity, float stepSize, float maxSpeed, int nNeighbors, Vec2& outVelocity);

		private:
			void GenerateConstraints(Simulator* simulator, const Entity& entity, const std::vector<Entity>& neighbors, float stepSize, int& outNConstraints, std::vector<Constraint>& outConstraints);
			bool RandomizedLP(int nConstraints, const std::vector<Constraint>& constraints, const Vec2& prefVel, const float maxSpeed, Vec2& outVelocity) const; // given a set of half planes and the preferredVelocity, solve the LP
		
		private:
			float m_lookAhead = 1.0f;

		};

	}
}