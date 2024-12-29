#pragma once

#include "ECMDataTypes.h"

typedef union SDL_Event;

namespace ECM {
	namespace WindowApplication {

		class Application;

		// Generic UI widget class
		class UIWidget
		{
		public:
			UIWidget(Application* application) 
				: m_Application(application) { }

			virtual void Render() = 0;
			virtual void Update(SDL_Event& e) = 0;

		protected:
			Application* m_Application;
		};

		// Class managing UI (ImGui-based)
		class UI
		{
		public:
			~UI();
			
			void Initialize(Application* application);
			void CleanUp();

			void SetStyle();

			void HandleInput(SDL_Event& e);
			void Update(SDL_Event& e);
			void Render();

		private:
			void InitializeWidgets(Application* application);

			std::vector<UIWidget*> m_Widgets;
		};

	}
}