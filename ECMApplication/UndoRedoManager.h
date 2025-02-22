#pragma once

#include "ECMDataTypes.h"
#include "UI.h"

typedef union SDL_Event;

namespace ECM {

	namespace WindowApplication {

		class ICommand;

		class UndoRedoManager
		{
		public:
			UndoRedoManager(int stackSize);
			~UndoRedoManager(); // clears the stack

			void Invoke(ICommand* cmd, bool alreadyExecuted = false);
			void Undo();
			void Redo();

		private:
			std::vector<ICommand*> m_CommandStack;
			int m_CurrentIdx;
		};
	}
}