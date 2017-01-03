#ifndef PRAY_PARALLEL_WORKER_H
#define PRAY_PARALLEL_WORKER_H

#include <thread>
#include <stack>

struct ThreadPool
{
	using thread_index = unsigned;

	ThreadPool(size_t num_threads) : threads(num_threads), idle_list(num_threads)
	{
		for(auto i=0; i<num_threads; ++i)
		{
			threads[i].thread = std::thread([=]{ worker_func(i); });
		}
	}

	~ThreadPool()
	{
		interrupt = true;
		for(auto &t : threads) t.wake_up();
		for(auto &t : threads) t.thread.join();
	}

	bool do_work(std::function<void(void*)> function, void *argument)
	{
		thread_index index;
		auto idle_avaliable = idle_list.get_idle(&index);
		if(!idle_avaliable) return false;
		auto &worker_thread = threads[index];
		worker_thread.work_function = function;
		worker_thread.work_argument = argument;
		worker_thread.wake_up();
		return true;
	}

	private:

	//TODO: make this lock-free
	struct IdleList
	{
		std::stack<thread_index, std::vector<thread_index>> elements;
		std::mutex lock;

		IdleList(size_t num_threads)
		{
			for(auto i=0; i<num_threads; ++i) elements.push(i);
		}

		bool get_idle(thread_index *out_index)
		{
			std::lock_guard<std::mutex> lock_guard(lock);
			if(elements.empty()) return false;
			*out_index = elements.top();
			elements.pop();
			return true;
		}

		void add_idle(thread_index index)
		{
			std::lock_guard<std::mutex> lock_guard(lock);
			elements.push(index);
		}
	};

	struct WorkerThread
	{
		std::thread thread;

		std::mutex mutex;
		bool wake = false;

		std::condition_variable cv;

		// called by this->thread
		void sleep()
		{
			std::unique_lock<std::mutex> lock(mutex);
			cv.wait(lock, [&]{ return wake; });
			wake = false;
		}

		// called by other thread
		void wake_up()
		{
			// not sure whether we need this lock here...
			std::unique_lock<std::mutex> lock(mutex);
			wake = true;
			lock.unlock();

			cv.notify_one();
		}

		std::function<void(void*)> work_function;
		void *work_argument;
	};

	std::vector<WorkerThread> threads;
	std::atomic<bool> interrupt{false};

	IdleList idle_list;

	void worker_func(thread_index index)
	{
		auto &worker_thread = threads[index];

		for(;;)
		{
			worker_thread.sleep();

			if(interrupt) break;

#if DEBUG
			ASSERT(worker_thread.work_function);
#endif

			worker_thread.work_function(worker_thread.work_argument);

#if DEBUG
			worker_thread.work_function = std::function<void(void*)>();
#endif
		}
	}
};

struct ParallelWorker
{
};

#endif
