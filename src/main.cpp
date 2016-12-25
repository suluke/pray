#include "scene.hpp"
#include "image.hpp"
#include "dummy_acceleration.hpp"
#include "bih.hpp"
#include "cpu_tracer.hpp"
#include "cpu_pathtracer.hpp"
#include "ray.hpp"
#include "sse_ray.hpp"
#include "logging.hpp" // This should always be last

#ifdef WITH_SSE
using ray_t = SSERay;
#else
using ray_t = Ray;
#endif

#ifdef WITH_BIH
using accel_t = Bih<ray_t>;
#else
using accel_t = DummyAcceleration<ray_t>;
#endif

static void traceWhitted(const Scene &scene, ImageView &img, const accel_t &accel) {
	CpuTracer<ray_t, accel_t> tracer(scene, accel);
	//tracer.acceleration_structure.printAnalysis();
	tracer.render(img);
}

static void tracePath(const Scene &scene, ImageView &img, const accel_t &accel, const RenderOptions::Path &opts) {
	std::cout << "FIXME: Path tracing not implemented" << std::endl;
	CpuPathTracer<Ray, accel_t> tracer(scene, opts, accel);
	//tracer.acceleration_structure.printAnalysis();
	tracer.render(img);
}

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
	RenderOptions opts;
	if(!scene.load(argv[1], &opts)) return 1;

	Image image(opts.resolution);

	if(scene.triangles.empty())
	{
		// when the scene is empty, just fill the image with the background color and get outta here, no need to do any ray tracing stuff
		image.fill(scene.background_color);
		image.save(argv[2]);
		return 0;
	}

	StageLogger logger;
	logger.start();

	ImageView img(image, 0, opts.resolution.h);

	logger.startPreprocessing();
	accel_t accel;
	accel.build(scene);

	logger.startRendering();
	switch (opts.method) {
		case RenderOptions::WHITTED: {
			traceWhitted(scene, img, accel);
			break;
		}
		case RenderOptions::PATH: {
			tracePath(scene, img, accel, opts.path_opts);
			break;
		}
		default:
			;// ???
	}

	logger.startOutput();
#ifndef DISABLE_OUTPUT
	image.save(argv[2]);
#endif

	logger.finish();
	logger.log();

	return 0;
}
