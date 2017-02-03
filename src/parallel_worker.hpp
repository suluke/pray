#ifndef PRAY_PARALLEL_WORKER_H
#define PRAY_PARALLEL_WORKER_H

#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <stack>

struct ThreadPool
{
	using thread_index = unsigned;
	static constexpr thread_index thread_index_invalid = std::numeric_limits<thread_index>::max();

	// cppreference: std::function is a polymorphic type... How about a non-polymorphic type? :)
	using work_function = std::function<void(void*)>;
	using work_argument = void*;

	ThreadPool(const size_t num_threads) : threads(num_threads)
	{
		ASSERT(num_threads >= 1u);

		for(auto i=0u; i<num_threads; ++i)
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

	size_t size()
	{
		return threads.size();
	}

	bool reserve_idle(thread_index *out_index) // force inline?
	{
		return idle_get(out_index);
	}

	void run_on_reserved(thread_index index, const work_function &function, const work_argument &argument = nullptr)
	{
		auto &worker_thread = threads[index];
		worker_thread.work_item.function = function;
		worker_thread.work_item.argument = argument;
		worker_thread.wake_up();
	}

	bool do_work(const work_function &function, const work_argument &argument = nullptr)
	{
		thread_index index;
		if(!reserve_idle(&index)) return false;
		run_on_reserved(index, function, argument);
		return true;
	}

	void wait_for_all_idle()
	{
		return idle_wait_all();
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

	// maybe parts of this struct should be allocated differently so that there's no chance of cache trashing
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
			cv.wait(lock, [this]{ return wake; });
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

		struct WorkItem
		{
			work_function function;
			work_argument argument;
		} work_item;

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

#ifdef DEBUG
			ASSERT(worker_thread.work_item.function);
#endif

			worker_thread.work_item.function(worker_thread.work_item.argument);

#ifdef DEBUG
			worker_thread.work_item.function = std::function<void(void*)>();
#endif

			// This is only correct as long as the thread is only waked when it was in the idle list (i.e. after idle_get).
			// We break this condition only on deconstruction but then immideately break out of the loop, so this should be fine...
			idle_insert(index);
		}
	}

	// This is a lock-free single linked list. Everything is implemented in idle_*
	std::atomic<thread_index> idle_head{thread_index_invalid};

	std::atomic<unsigned> idle_count{0};
	std::mutex idle_wait_all_mutex;
	std::condition_variable idle_wait_all_cv;

	// Tries to get a thread from the idle list. Returns false when no idle thread is avaliable.
	bool idle_get(thread_index *out_index) // force inline?
	{
		auto head = idle_head.load();
		if(head == thread_index_invalid) return false;

		while(!idle_head.compare_exchange_weak(head, threads[head].idle_next))
		{
			if(head == thread_index_invalid) return false;
		}

		--idle_count;

		*out_index = head;
		return true;
	}

	// Inserts a thread into the idle list.
	void idle_insert(const thread_index index)
	{
		threads[index].idle_next = idle_head.load();
		while(!idle_head.compare_exchange_weak(threads[index].idle_next, index));

		const auto new_idle_count = ++idle_count;
		// no race condition here, only one can be the last one
		if(new_idle_count == threads.size())
		{
			idle_wait_all_cv.notify_all();
		}
	}

	void idle_wait_all()
	{
		if(idle_count == threads.size()) std::cout << "shit\n";

		std::unique_lock<std::mutex> lock(idle_wait_all_mutex);
		idle_wait_all_cv.wait(lock, [this]{ return idle_count == threads.size(); });
	}
};

template<class Args>
struct ParallelRecursion
{
	ParallelRecursion(ThreadPool &thread_pool) : thread_pool(thread_pool), argument_storage(thread_pool.size()) {}
	~ParallelRecursion() {}

	using function_t = std::function<void(const Args&, ParallelRecursion<Args>&)>;

	// call this function first, blocks until all work is done
	void run(const function_t &function, const Args &args)
	{
		recurse(function, args);
		thread_pool.wait_for_all_idle();
	}

	// call this function from the worker threads
	void recurse(const function_t &function, const Args &args, bool predicate = true)
	{
		ThreadPool::thread_index thread;
		if(predicate && thread_pool.reserve_idle(&thread))
		{
			// we successfully reserved an idle thread, use it to run the function asynchronously
			argument_storage[thread] = args;
			thread_pool.run_on_reserved(thread, [=](void *a) { function(*static_cast<Args*>(a), *this); }, &argument_storage[thread]);
		}
		else
		{
			// call function synchronously
			function(args, *this);
		}
	}

	private:
	ThreadPool &thread_pool;

	// this probably causes cash trashing...
	std::vector<Args> argument_storage;
};

#endif
