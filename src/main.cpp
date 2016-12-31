#include "scene.hpp"
#include "image.hpp"
#include "dummy_acceleration.hpp"
#include "bih.hpp"
#include "cpu_tracer.hpp"
#include "cpu_pathtracer.hpp"
#include "ray.hpp"
#include "sse_ray.hpp"
#include "logging.hpp" // This should always be last

template<class scene_t>
struct PrayTypes {
#ifdef WITH_SSE
	using ray_t = SSERay<scene_t>;
#else
	using ray_t = Ray<scene_t>;
#endif
#ifdef WITH_BIH
	using accel_t = Bih<ray_t, scene_t>;
#else
	using accel_t = DummyAcceleration<ray_t, scene_t>;
#endif
};
template<>
struct PrayTypes<PathScene> {
	using scene_t = PathScene;
	using ray_t = Ray<scene_t>;
#ifdef WITH_BIH
	using accel_t = Bih<ray_t, scene_t>;
#else
	using accel_t = DummyAcceleration<ray_t, scene_t>;
#endif
};

using WhittedTypes = PrayTypes<WhittedScene>;
using PathTypes = PrayTypes<PathScene>;

static void traceScene(const WhittedScene &scene, ImageView &img, const WhittedTypes::accel_t &accel, const RenderOptions &opts) {
	CpuTracer<WhittedTypes::ray_t, WhittedTypes::accel_t> tracer(scene, accel);
	//tracer.acceleration_structure.printAnalysis();
	tracer.render(img);
}

static void traceScene(const PathScene &scene, ImageView &img, const PathTypes::accel_t &accel, const RenderOptions &opts) {
	CpuPathTracer< PathTypes::ray_t, PathTypes::accel_t > tracer(scene, opts.path_opts, accel);
	//tracer.acceleration_structure.printAnalysis();
	tracer.render(img);
}

template<class scene_t>
static int trace(const char *outpath, RenderOptions &opts) {
	Image image(opts.resolution);

	StageLogger logger;
	logger.start();

	ImageView img(image, 0, opts.resolution.h);

	scene_t scene;
	if (!LoadScene(opts, &scene)) return 1;
	if (scene.triangles.empty()) {
		image.fill(scene.background_color);
		image.save(outpath);
		return 0;
	}

	logger.startPreprocessing();
	typename PrayTypes<scene_t>::accel_t accel;
	accel.build(scene);

	logger.startRendering();
	traceScene(scene, img, accel, opts);

	logger.startOutput();
#ifndef DISABLE_OUTPUT
	image.save(outpath);
#endif

	logger.finish();
	logger.log();
	return 0;
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

	cout << "Loading..." << endl;
	RenderOptions opts;
	if(!LoadJob(argv[1], &opts)) return 1;

	switch (opts.method) {
		case RenderOptions::WHITTED: {
			return trace<WhittedScene>(argv[2], opts);
		}
		case RenderOptions::PATH: {
			return trace<PathScene>(argv[2], opts);
		}
		default:
			return 1;
	}
	return 0;
}
