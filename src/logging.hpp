#ifndef PRAY_LOGGING_HPP
#define PRAY_LOGGING_HPP

#include "pray/Config.h"
#ifdef WITH_TIMING
 #include <chrono>
#endif

struct StageLogger {
#ifdef WITH_TIMING
	using timepoint_t = std::chrono::time_point<std::chrono::high_resolution_clock>;
	timepoint_t begin;
	timepoint_t preprocess_begin;
	timepoint_t render_begin;
	timepoint_t output_begin;
	timepoint_t end;
#endif
	void start() {
#ifdef WITH_TIMING
		begin = std::chrono::high_resolution_clock::now();
#endif
	}
	void finish() {
#ifdef WITH_TIMING
		end = std::chrono::high_resolution_clock::now();
#endif
	}
	void startPreprocessing() {
		std::cout << "Preprocessing..." << std::endl;
#ifdef WITH_TIMING
		preprocess_begin = std::chrono::high_resolution_clock::now();
#endif
	}
	void startRendering() {
		std::cout << "Rendering..." << std::endl;
#ifdef WITH_TIMING
		render_begin = std::chrono::high_resolution_clock::now();
#endif
	}
	void startOutput() {
#ifndef DISABLE_OUTPUT
		std::cout << "Saving..." << std::endl;
#endif
#ifdef WITH_TIMING
		output_begin = std::chrono::high_resolution_clock::now();
#endif
	}
	void log() {
#ifdef WITH_TIMING
		std::cout << "Preprocess Time: " << std::chrono::duration_cast<std::chrono::milliseconds>(render_begin - preprocess_begin).count() << "ms\n";
		std::cout << "Render Time: " << std::chrono::duration_cast<std::chrono::milliseconds>(output_begin - render_begin).count() << "ms\n";
		std::cout << "Total Time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "ms\n";
#endif
	}
};
#ifdef WITH_CONFDUMP
#include <iomanip>
#include <cstring>
#define STR(x)   #x
#define print_opt(x) do {\
		auto f(std::cout.flags());\
		std::cout << std::left << std::setw(27) << #x ": ";\
		strcmp(#x, STR(x)) ? std::cout << "ON\n" : std::cout << "OFF\n";\
		std::cout.flags(f);\
	} while(false)
	
	static void dump_config() {
		std::cout << "####### Configuration: #######\n";
		print_opt(WITH_OMP);
		print_opt(WITH_CUDA);
		print_opt(WITH_SSE);
		print_opt(WITH_BIH);
		print_opt(WITH_BIH_SMART_TRAVERSION);
		print_opt(WITH_BIH_SMART_TRAVERSION_ITERATIVE);
		print_opt(WITH_SUBSAMPLING);
		print_opt(WITH_TIMING);
		print_opt(WITH_DEBUG_TOOL);
		print_opt(WITH_CONFDUMP);
		print_opt(DISABLE_OUTPUT);
		std::cout << "##############################\n";
	}
#undef STR
#undef print_opt
#else
	static void dump_config() {}
#endif
#endif // PRAY_LOGGING_HPP
