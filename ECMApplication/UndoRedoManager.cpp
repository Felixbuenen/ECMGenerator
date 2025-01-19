#include "UndoRedoManager.h"

#include "Command.h"

namespace ECM {

	namespace WindowApplication {

		UndoRedoManager::UndoRedoManager(int stackSize)
		{
			m_CommandStack.resize(stackSize);
			m_CurrentIdx = -1;
		}

		UndoRedoManager::~UndoRedoManager()
		{
			for (ICommand* c : m_CommandStack)
			{
				delete c;
			}
		}

		void UndoRedoManager::Execute(ICommand* cmd)
		{
			cmd->Execute();

			// update stack with new command
			if (m_CurrentIdx == m_CommandStack.size() - 1)
			{
				// pop the oldest command in the stack
				delete m_CommandStack[0];

				for (int i = 0; i < m_CommandStack.size()-1; i++)
				{
					m_CommandStack[i] = m_CommandStack[i + 1];
				}
			}
			else
			{
				m_CurrentIdx++;
			}

			m_CommandStack[m_CurrentIdx] = cmd;

			// after the execution of a new command, remove any commands after the new command
			for (int i = m_CurrentIdx+1; i < m_CommandStack.size(); i++)
			{
				if (m_CommandStack[i] != nullptr)
				{
					delete m_CommandStack[i];
				}
			}
		}

		void UndoRedoManager::Undo()
		{
			m_CommandStack[m_CurrentIdx]->Undo();
			m_CurrentIdx--;
		}

		void UndoRedoManager::Redo()
		{
			if (m_CurrentIdx < m_CommandStack.size() - 1)
			{
				if (m_CommandStack[m_CurrentIdx + 1] != nullptr)
				{
					m_CurrentIdx++;
					m_CommandStack[m_CurrentIdx]->Execute();
				}
			}
		}
	}
}
