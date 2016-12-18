#include "scene.hpp"
#include "image.hpp"
#include "dummy_acceleration.hpp"
#include "bih.hpp"
#include "cpu_tracer.hpp"
#include "ray.hpp"
#include "sse_ray.hpp"
#include "logging.hpp" // This should always be last

// Sorry for this...
#ifdef WITH_BIH
#define accel_t Bih
#else
#define accel_t DummyAcceleration
#endif

#ifdef WITH_SSE
static void trace(const Scene &scene, ImageView &img, StageLogger &logger) {
	//~ if (scene.lights.size() < 2)
	{
		CpuTracer<SSERay, accel_t<SSERay>> tracer(scene);

		logger.startPreprocessing();
		tracer.preprocess();

		logger.startRendering();
		tracer.render(img);
	}
	//~ else {
		//~ CpuTracer<Ray, accel_t<Ray>> tracer(scene);

		//~ logger.startPreprocessing();
		//~ tracer.preprocess();

		//~ logger.startRendering();
		//~ tracer.render(img);
	//~ }
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
