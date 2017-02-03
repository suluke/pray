#ifndef PRAY_LOGGING_HPP
#define PRAY_LOGGING_HPP

#include "pray/Config.h"
#include <iostream>
#ifdef WITH_TIMING
  #include <chrono>
#endif
#ifdef WITH_PROGRESS
  #include <atomic>
  #include <future>
  #include <iomanip>
#endif

struct StageLogger {
  const RenderOptions &opts;
  StageLogger(const RenderOptions &opts) : opts(opts) {}
#ifdef WITH_PROGRESS
  static constexpr unsigned progressWidth = 3;
  const Image *image;
  std::atomic<bool> renderFinished{false};
  std::thread progressPrinter;
  ~StageLogger() {
    progressPrinter.join();
  }
#endif // WITH_PROGRESS
#ifdef WITH_TIMING
  using timepoint_t = std::chrono::time_point<std::chrono::high_resolution_clock>;
  timepoint_t begin;
  timepoint_t preprocess_begin;
  timepoint_t render_begin;
  timepoint_t output_begin;
  timepoint_t end;
#endif
  void start() {
    std::cout << "Loading " << opts.filename << std::endl;
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
#ifndef DISABLE_RENDERING
    std::cout << "Rendering..." << std::endl;
#endif
#ifdef WITH_PROGRESS
		progressPrinter = std::thread([](const StageLogger *logger) {
			while (!logger->renderFinished) {
				std::this_thread::sleep_for(std::chrono::milliseconds{200});
				
				unsigned int writtenPixelsSum = logger->image->writtenPixels;

				auto percent = writtenPixelsSum * 100 / (logger->opts.resolution.w * logger->opts.resolution.h);
				if (percent == 100) break;
				auto f(std::cout.flags());
				std::cout << std::string(StageLogger::progressWidth, '\b');
				std::cout << std::right << std::setw(StageLogger::progressWidth - 1); // -1 because of %
				std::cout << percent << "%" << std::flush;
				std::cout.flags(f);
			}
		}, this);
#endif // WITH_PROGRESS
#ifdef WITH_TIMING
    render_begin = std::chrono::high_resolution_clock::now();
#endif
  }
  void startOutput() {
#ifdef WITH_PROGRESS
    std::cout << std::string(progressWidth, '\b') << std::setw(progressWidth) << "100%" << std::endl;
    renderFinished = true;
#endif // WITH_PROGRESS
#ifndef DISABLE_OUTPUT
    std::cout << "Saving..." << std::endl;
#endif
#ifdef WITH_TIMING
    output_begin = std::chrono::high_resolution_clock::now();
#endif
  }
  void log() const {
#ifdef WITH_TIMING
    std::cout << "Preprocess Time: " << std::chrono::duration_cast<std::chrono::milliseconds>(render_begin - preprocess_begin).count() << "ms\n";
    std::cout << "Render Time: " << std::chrono::duration_cast<std::chrono::milliseconds>(output_begin - render_begin).count() << "ms\n";
    std::cout << "Total Time: " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "ms\n";
#endif
  }
  void dump_config() const;
};
#ifdef WITH_CONFDUMP
#include <iomanip>
#include <cstring>
#define STR(x)   #x

#define print_opt(x) do {                              \
    auto f(std::cout.flags());                         \
    std::cout << std::left << std::setw(16) << #x ": ";\
    if (strcmp(#x, STR(x))) {                          \
      if (strcmp(STR(x), "")) {                        \
        std::cout << STR(x)"\n";                       \
      } else {                                         \
        std::cout << "ON\n";                           \
      }                                                \
    } else {                                           \
      std::cout << "OFF\n";                            \
    }                                                  \
    std::cout.flags(f);                                \
  } while(false)
  
  void StageLogger::dump_config() const {
    std::cout << "####### Configuration: #######\n";
    print_opt(WITH_OMP);
    print_opt(WITH_CUDA);
    print_opt(WITH_SSE);
    print_opt(WITH_SSE_PT);
    print_opt(ACCELERATOR);
    print_opt(SAMPLER);
    print_opt(WITH_BIH_PARALLEL_BUILD);
    print_opt(WITH_CHEATS);
    print_opt(WITH_TIMING);
    print_opt(WITH_CONFDUMP);
    print_opt(WITH_PROGRESS);
    print_opt(DISABLE_RENDERING);
    print_opt(DISABLE_OUTPUT);
    std::cout << "##############################\n";
#ifdef DEBUG
    std::cout << "Warning: This is a Debug build and might be very slow!\n";
#endif
  }
#undef STR
#undef print_opt
#else
  void StageLogger::dump_config() const {}
#endif
#endif // PRAY_LOGGING_HPP
