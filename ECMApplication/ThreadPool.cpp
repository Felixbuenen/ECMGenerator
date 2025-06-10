#include "ThreadPool.h"

ThreadPool::ThreadPool(size_t nThreads)
{
	m_Stop = false;

	// create pool of threads
	for (size_t i = 0; i < nThreads; i++)
	{
		m_Pool.emplace_back(std::thread([this]() {

			// worker threads loop infinitely, waiting for tasks to pick up
			while (true)
			{
				std::function<void()> task; // tasks need to be parameter-less and return void

				{
					std::unique_lock<std::mutex> lock(m_QueueMutex);

					// if the queue is empty and we don't want to stop the pool -> block this thread, release the lock, and wait for a 
					//  notify when there are new tasks in the queue or the pool must be stopped.
					m_CV.wait(lock, [this]() { return m_Stop || !m_TaskQueue.empty(); });

					// return if pool must be stopped and the task queue is empty
					if (m_Stop && m_TaskQueue.empty()) return;

					task = std::move(m_TaskQueue.front());
					m_TaskQueue.pop();
				}

				task();
			}
			}
		));
	}
}

ThreadPool::~ThreadPool()
{
	// stop the threadpool
	{
		std::lock_guard<std::mutex> lock(m_QueueMutex);
		m_Stop = true;
	}

	// notify all waiting worker threads and wait for them to finish
	m_CV.notify_all();
	for (std::thread& worker : m_Pool)
		if (worker.joinable())
			worker.join();
}