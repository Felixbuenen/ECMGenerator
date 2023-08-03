#pragma once

namespace ECM
{
	// not sure if 'visualisation' is a good name, but I want to detach the core ECM logic from the application/visualisation part
	namespace WindowApplication
	{
		class ApplicationState
		{
		public:
			ApplicationState() { } // prevent making instance
		};

		class Application
		{
		public:
			Application() { }

			bool InitializeEnvironment(const char* file);
			bool InitializeTestEnvironment();

			bool InitializeApplication(const char* title);

			const ApplicationState& GetApplicationState() const { return m_ApplicationState; }

		private:
			ApplicationState m_ApplicationState;
		};
	}
}