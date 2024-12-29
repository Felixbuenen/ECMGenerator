#pragma once

#include "ECMDataTypes.h"

typedef union SDL_Event;

namespace ECM {

	namespace WindowApplication {

		class Application;

		enum SimAreaDrag 
		{
			NONE,
			SPAWN,
			GOAL
		};

		enum PanelMode
		{
			BUILD,
			CONNECT
		};

		// UI panel that allows the user to drag and drop simulation areas and to build connections between spawn- and goal areas.
		class SimAreaPanel
		{
		public:
			void Render();
			void Update(SDL_Event& e, Application& app);

		private:
			bool m_IsDragging = false;
			SimAreaDrag m_DragType = SimAreaDrag::NONE;
			PanelMode m_PanelMode = PanelMode::BUILD;
		};
	}

}