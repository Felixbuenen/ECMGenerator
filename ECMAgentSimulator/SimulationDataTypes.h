#pragma once
namespace ECM {
	namespace Simulation {

		struct PositionComponent {
			float* x;
			float* y;
		};

		struct VelocityComponent {
			float* dx;
			float* dy;
		};
	}
}