#pragma once

#include "ECMDataTypes.h"
#include "UI.h"

typedef union SDL_Event;

namespace ECM {

	namespace WindowApplication {

		class Application;
		struct ApplicationState;

		class MainMenu : public UIWidget
		{
		public:
			MainMenu(Application* application);

			void Render() override;
			void Update(SDL_Event& e) override { }

		private:
			ApplicationState* m_ApplicationState;

		};
	}
}
