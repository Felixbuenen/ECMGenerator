#pragma once

#include <condition_variable>
#include <mutex>
#include <vector>
#include <queue>
#include <thread>
#include <future>
#include <utility>
#include <functional>
#include <tuple>
#include <utility>

class ThreadPool
{
public:
	ThreadPool(size_t nThreads);
	~ThreadPool();

	template<class F, class... Args>
	auto AddTask(F&& f, Args&&... args) -> std::future<std::invoke_result_t<F, Args...>>
	{
		using ReturnType = std::invoke_result_t<F, Args...>;

		// task is a (shared) ptr of a packaged task. The packaged task wraps a callable (in this case of type ReturnType()) so that
		//  it can be invoked asynchronously. This allows us to return its std::future. In this case we package a callable bind, aka we bind
		//  the Args list to the function f. It's a ptr, because A) packaged_task can't be copied and B) even if we move the PT, then the lambda (std::function)
		//  that we want to add to the task queue can't be copied because of the PT. 
		auto task = std::make_shared<std::packaged_task<ReturnType()>>(
			std::bind(std::forward<F>(f), std::forward<Args>(args)...)
		);

		std::future<ReturnType> res = task.get_future();
		{
			std::unique_lock<std::mutex> lock(m_QueueMutex);
			m_TaskQueue.emplace([task = std::move(task)]() mutable { task(); }); // the lambda actually calls the bounded task
		}
		m_CV.notify_one();
		return res;
	}

private:
	std::vector<std::thread> m_Pool; // worker threads
	std::queue<std::function<void()>> m_TaskQueue; // tasks to be picked up by workers

	bool m_Stop;
	std::mutex m_QueueMutex;
	std::condition_variable m_CV;
};