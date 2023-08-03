#pragma once

#include "SDL.h"

/*
* WINDOW AND INTERACTION STRUCTURE
* > Application: starts the SDL window, reads (or manually sets up) environment and creates the ECM. Contains main loop.
* > ApplicationState: can be a struct, owned by the application to maintain a certain state (e.g. start / goal positions)
* > ECMRenderer: responsible for drawing to the SDL window.
* > InputHandler: receives input and processes it. E.g., when clicking left mouse, update application state to contain new start position.
*/

// TODO: key/event binding struct. 

namespace ECM {
	namespace WindowApplication {

		class InputHandler
		{

		};

	}
}