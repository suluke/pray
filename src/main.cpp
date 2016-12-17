#include "scene.hpp"
#include "image.hpp"
#include "dummy_acceleration.hpp"
#include "bih.hpp"
#include "cpu_tracer.hpp"

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

#ifdef WITH_SSE
	#include "sse_ray.hpp"
	using ray_t = SSERay;
#else
	#include "ray.hpp"
	using ray_t = Ray;
#endif
#ifdef WITH_BIH
	using accel_t = Bih<ray_t>;
#else
	using accel_t = DummyAcceleration<ray_t>;
#endif

using namespace std;
int main(int argc, char *argv[])
{
	if (argc != 3)
	{
		cerr << "usage: " << argv[0] << " <input.json> <output.bmp>\n";
		return 1;
	}

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
	CpuTracer<ray_t, accel_t> tracer(scene);

	logger.startPreprocessing();
	tracer.preprocess();

	logger.startRendering();
	tracer.render(img);

	logger.startOutput();
#ifndef DISABLE_OUTPUT
	image.save(argv[2]);
#endif

	logger.finish();
	logger.log();

	return 0;
}
