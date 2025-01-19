#pragma once

#include "Area.h"

//#include "Application.h"

// TODO: een API die alle logic managet omtrent het modeleren van een environment.
// - add simulation areas
// - scale / reposition areas
// - set area connections
// - ...

// TODO: voeg een AddSpawn/GoalArea functie toe aan de Environment class. 
// verschil met EnvEditor class is dat de EnvEditor alles omtrent de editor doet, dus bijv. area dragging, de interactie
//  waarmee je twee areas kan connecten, etc... Dit wordt dus een Interface voor de UI om bewerkingen aan de omgeving te maken.
// Het is dus belangrijk dat de EnvEditor een referentie naar een omgeving heeft!

typedef union SDL_Event;

namespace ECM {

	namespace WindowApplication {

		class Application;

		class EnvironmentEditor {

		public:
			EnvironmentEditor(Application* application);

			void HandleInput(SDL_Event& e);
			void Update();
			void Render();

			void StartDragArea(Simulation::SimAreaType areaType);
			void StopDragArea();

		private:
			Simulation::SimAreaType m_CurrentDragArea;
			Application* m_App;

			void DragAreaInternal();
			void HandleDropArea(const Point& screenPos);
		};

	}
}