#include "scene.hpp"
#include "image.hpp"
#include "dummy_acceleration.hpp"
#include "bih.hpp"
#include "cpu_tracer.hpp"
#include "ray.hpp"
#include "sse_ray.hpp"

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
		std::cout << std::left << std::setw(18) << #x ": ";\
		strcmp(#x, STR(x)) ? std::cout << "ON\n" : std::cout << "OFF\n";\
		std::cout.flags(f);\
	} while(false)
	
	static void dump_config() {
		std::cout << "#### Configuration: ####\n";
		print_opt(WITH_OMP);
		print_opt(WITH_CUDA);
		print_opt(WITH_SSE);
		print_opt(WITH_BIH);
		print_opt(WITH_SUBSAMPLING);
		print_opt(WITH_TIMING);
		print_opt(WITH_DEBUG_TOOL);
		print_opt(WITH_CONFDUMP);
		print_opt(DISABLE_OUTPUT);
		std::cout << "########################\n";
	}
#undef STR
#undef print_opt
#else
	static void dump_config() {}
#endif

// Sorry for this...
#ifdef WITH_BIH
#define accel_t Bih
#else
#define accel_t DummyAcceleration
#endif

#ifdef WITH_SSE
static void trace(const Scene &scene, ImageView &img, StageLogger &logger) {
	if (scene.lights.size() < 2) {
		CpuTracer<SSERay, accel_t<SSERay>> tracer(scene);

		logger.startPreprocessing();
		tracer.preprocess();

		logger.startRendering();
		tracer.render(img);
	} else {
		CpuTracer<Ray, accel_t<Ray>> tracer(scene);

		logger.startPreprocessing();
		tracer.preprocess();

		logger.startRendering();
		tracer.render(img);
	}
}
#else
using ray_t = Ray;
static void trace(const Scene &scene, ImageView &img, StageLogger &logger) {
	CpuTracer<ray_t, accel_t<ray_t>> tracer(scene);

	logger.startPreprocessing();
	tracer.preprocess();

	logger.startRendering();
	tracer.render(img);
}
#endif

using namespace std;
int main(int argc, char *argv[])
{
	if (argc != 3)
	{
		cerr << "usage: " << argv[0] << " <input.json> <output.bmp>\n";
		return 1;
	}
	dump_config();

#ifdef DEBUG
	cout << "Warning: This is a Debug build and might be very slow!\n";
#endif

	Scene scene;

	cout << "Loading..." << endl;
	IntDimension2 image_resolution = IntDimension2(1920, 1080);
	if(!scene.load(argv[1], &image_resolution)) return 1;

	Image image(image_resolution);

	if(scene.triangles.empty())
	{
		// when the scene is empty, just fill the image with the background color and get outta here, no need to do any ray tracing stuff
		image.fill(scene.background_color);
		image.save(argv[2]);
		return 0;
	}

	StageLogger logger;
	logger.start();

	ImageView img(image, 0, image_resolution.h);

	trace(scene, img, logger);

	logger.startOutput();
#ifndef DISABLE_OUTPUT
	image.save(argv[2]);
#endif

	logger.finish();
	logger.log();

	return 0;
}
