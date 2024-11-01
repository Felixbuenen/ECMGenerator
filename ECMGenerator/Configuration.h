#pragma once

#include <limits>

// contains constants used by the ECM engine

namespace ECM
{
	namespace Utility
	{
		static const float MAX_FLOAT = std::numeric_limits<float>::max();
		static const float MIN_FLOAT = std::numeric_limits<float>::min();

		static const float EPSILON = 0.0001f;
		static const float HALF_EPSILON = 0.001f;
	}
}