#pragma once

//#include "Area.h"
#include "Gizmo.h"

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

	namespace Simulation {
		struct Area;
		enum SimAreaType;
	}

	namespace WindowApplication {

		enum TransformMode
		{
			TRANSLATE,
			SCALE
		};

		class Application;

		class EnvironmentEditor {

		public:
			EnvironmentEditor() { }

			void Initialize(Application* application);

			void HandleInput(SDL_Event& e);
			void Update();
			void Render();

			void StartDragArea(Simulation::SimAreaType areaType);
			void StopDragArea();

			void SelectArea(Simulation::Area* area);

		private:
			Application* m_App;

			// state
			Simulation::SimAreaType m_CurrentDragArea;
			Simulation::Area* m_SelectedArea;
			TransformMode m_TransformMode;
			Gizmo* m_ActiveGizmo;
			TranslateGizmo m_TranslateGizmo;
			ScaleGizmo m_ScaleGizmo;
			
			// UI event handlers
			void HandleLeftClickGeneral(const Point& screenPos);
			void HandleDropArea(const Point& screenPos);

			// methods
			void DragAreaInternal();
		};

	}
}