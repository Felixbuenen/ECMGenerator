#pragma once

#include "ECMDataTypes.h"
#include "UI.h"

typedef union SDL_Event;

namespace ECM {

	namespace WindowApplication {

		//class Application;
		//class ECMRenderer;
		//
		//enum PanelMode
		//{
		//	BUILD,
		//	CONNECT
		//};
		//
		//// UI panel that allows the user to drag and drop simulation areas and to build connections between spawn- and goal areas.
		//class SimAreaPanel : public UIWidget
		//{
		//public:
		//	SimAreaPanel(Application* application);
		//
		//	void Render() override;
		//	void Update(SDL_Event& e) override;
		//
		//private:
		//	bool m_IsDragging = false;
		//	SimAreaDrag m_DragType = SimAreaDrag::NONE;
		//	PanelMode m_PanelMode = PanelMode::BUILD;
		//	ECMRenderer* m_EcmRenderer;
		//};
	}

}