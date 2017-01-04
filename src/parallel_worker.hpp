#ifndef PRAY_PARALLEL_WORKER_H
#define PRAY_PARALLEL_WORKER_H

#include <thread>
#include <stack>

struct ThreadPool
{
	using thread_index = unsigned;
	static constexpr thread_index thread_index_invalid = std::numeric_limits<thread_index>::max();

	ThreadPool(const size_t num_threads) : threads(num_threads)
	{
		ASSERT(num_threads >= 1u);

		for(auto i=0; i<num_threads; ++i)
		{
			threads[i].thread = std::thread([=]{ worker_func(i); });

			// When doing the idle_insert here instead of at the beginning of worker_func we don't need
			// to wait for the threads to be executed before using the pool. It's still correct
			// since the wake is stored as long as the thread has not tried to sleep.
			idle_insert(i);
		}
	}

	~ThreadPool()
	{
		interrupt = true;
		for(auto &t : threads) t.wake_up();
		for(auto &t : threads) t.thread.join();
	}

	bool do_work(std::function<void(void*)> function, void *argument) // force inline?
	{
		thread_index index;
		auto idle_avaliable = idle_get(&index);
		if(!idle_avaliable) return false;
		auto &worker_thread = threads[index];
		worker_thread.work_function = function;
		worker_thread.work_argument = argument;
		worker_thread.wake_up();
		return true;
	}

	// don't use this function while worker threads are executing, it's not thread safe...
	void debug_print_idle_list()
	{
		for(auto i = idle_head.load(); i != thread_index_invalid; i = threads[i].idle_next)
		{
			std::cout << i << " ";
		}
		std::cout << "\n";
	}

	private:

	// maybe parts of this struct should be allocated differently so that there's no change of cache trashing
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
			std::unique_lock<std::mutex> lock(mutex);
			wake = true;
			lock.unlock();

			cv.notify_one();
		}

		// cppreference: std::function is a polymorphic type... How about a non-polymorphic type? :)
		std::function<void(void*)> work_function;
		void *work_argument;

		thread_index idle_next;
	};

	std::vector<WorkerThread> threads;
	std::atomic<bool> interrupt{false};

	void worker_func(const thread_index index)
	{
		auto &worker_thread = threads[index];

		for(;;)
		{
			//TODO: It would be great to be able to queue one work item so that we don't yield when there's work to do.
			worker_thread.sleep();

			if(interrupt) break;

#if DEBUG
			ASSERT(worker_thread.work_function);
#endif

			worker_thread.work_function(worker_thread.work_argument);

#if DEBUG
			worker_thread.work_function = std::function<void(void*)>();
#endif

			// This is only correct as long as the thread is only waked when it was in the idle list (i.e. after idle_get).
			// We break this condition only on deconstruction but then immideately break out of the loop, so this should be fine...
			idle_insert(index);
		}
	}

	// This is a lock-free single linked list. Everything is implemented in idle_*
	std::atomic<thread_index> idle_head{thread_index_invalid};

	// Tries to get a thread from the idle list. Returns false when no idle thread is avaliable.
	bool idle_get(thread_index *out_index) // force inline?
	{
		auto head = idle_head.load();
		if(head == thread_index_invalid) return false;

		while(!idle_head.compare_exchange_weak(head, threads[head].idle_next))
		{
			if(head == thread_index_invalid) return false;
		}

		*out_index = head;
		return true;
	}

	// Inserts a thread into the idle list.
	void idle_insert(const thread_index index)
	{
		threads[index].idle_next = idle_head.load();
		while(!idle_head.compare_exchange_weak(threads[index].idle_next, index));
	}
};

struct ParallelWorker
{
};

#endif
