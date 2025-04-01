#pragma once

//#include "Area.h"
#include "Gizmo.h"

//#include "Application.h"

// TODO: voeg een AddSpawn/GoalArea functie toe aan de Environment class. 
// verschil met EnvEditor class is dat de EnvEditor alles omtrent de editor doet, dus bijv. area dragging, de interactie
//  waarmee je twee areas kan connecten, etc... Dit wordt dus een Interface voor de UI om bewerkingen aan de omgeving te maken.
// Het is dus belangrijk dat de EnvEditor een referentie naar een omgeving heeft!

typedef union SDL_Event;

namespace ECM {

	namespace Simulation {
		struct Area;
		struct AreaConnection;
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
			~EnvironmentEditor();

			void Initialize(Application* application);

			void HandleInput(SDL_Event& e);
			void Update();
			void Render();

			void SwitchEditorState();

			void StartDragArea(Simulation::SimAreaType areaType);
			void StopDragArea();
			void SetDragAreaConnection(bool drag = true);

			void SelectArea(Simulation::Area* area);
			void SelectAreaConnection(Simulation::AreaConnection areaConnection);

		private:
			Application* m_App;

			// state
			Simulation::SimAreaType m_CurrentDragArea;
			Simulation::Area* m_SelectedArea;
			Simulation::AreaConnection* m_SelectedAreaConnection; // TODO (?): make a 'Selectable' object
			TransformMode m_TransformMode;
			Gizmo* m_ActiveGizmo;
			TranslateGizmo m_TranslateGizmo;
			ScaleGizmo m_ScaleGizmo;

			// area connection dragging state
			bool m_IsDraggingAreaConnection;
			Simulation::Area* m_ConnectionDragSourceArea;
			Simulation::Area* m_ConnectionDragHoverArea;
			
			// UI event handlers
			void HandleLeftButtonDownGeneral(const Point& screenPos);
			void HandleLeftButtonUpGeneral(const Point& screenPos);
			void HandleDropArea(const Point& screenPos);
			void HandleDeleteObject();

			void SelectAreaAtPosition(const Point& screenPos);
			void DeselectArea();

			// methods
			void RenderDragArea();
			void RenderDragAreaConnection();
		};

	}
}