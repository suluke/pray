#include "pray/Config.h" // This should always be first
#include "scene.hpp"
#include "image.hpp"
#include "dummy_acceleration.hpp"
#include "bih.hpp"
#include "kdtree.hpp"
#include "cpu_tracer.hpp"
#include "cpu_pathtracer.hpp"
#include "ray.hpp"
#include "sse_ray.hpp"
#include "sampler.hpp"

#ifdef WITH_CUDA
	#include "cuda_pathtracer.hpp"
	#include "cuda_bih.hpp"
	#include "cuda_dummy_acceleration.hpp"
#endif

#include "logging.hpp" // This should always be last

#include <thread>
#include <queue>
#include <mutex>

template<class scene_t>
struct PrayTypes {
#ifdef WITH_SSE
	using ray_t = SSERay<scene_t>;
#else
	using ray_t = Ray<scene_t>;
#endif
	using accel_t = ACCELERATOR<ray_t, scene_t>;
#ifdef WITH_CUDA
	using accel_cuda_t = ACCELERATOR_CUDA;
#endif
	template <class tracer_t>
	using sampler_t = SAMPLER<scene_t, tracer_t, ray_t>;
};
#ifndef WITH_SSE_PT
template<>
struct PrayTypes<PathScene> {
	using scene_t = PathScene;
	using ray_t = Ray<scene_t>;
	using accel_t = ACCELERATOR<ray_t, scene_t>;
#ifdef WITH_CUDA
	using accel_cuda_t = ACCELERATOR_CUDA;
#endif
	template <class tracer_t>
	using sampler_t = SAMPLER<scene_t, tracer_t, ray_t>;
};
#endif // not WITH_SSE_PT

using WhittedTypes = PrayTypes<WhittedScene>;
using PathTypes = PrayTypes<PathScene>;

static void traceScene(const WhittedScene &scene, Image &image, const WhittedTypes::accel_t &accel, const RenderOptions &opts) {
  ImageView img(image, 0, opts.resolution.h);
  
	auto tracer = CpuTracer<WhittedTypes::ray_t, WhittedTypes::accel_t>(scene, accel);
	using sampler = WhittedTypes::sampler_t<decltype(tracer)>;
	sampler::render(scene, img, tracer);
}

static void traceScene(const PathScene &scene, Image &image, const PathTypes::accel_t &accel, const RenderOptions &opts) {
#ifdef WITH_CUDA
	std::mutex views_mutex;
	std::queue<ImageView> views;
	static const unsigned int view_height = 8 * 8;
	
	for (unsigned int i = 0; i < opts.resolution.h; i += view_height)
	{
		ImageView img(image, i, std::min(i + view_height, opts.resolution.h));
		views.push(img);
	}
	
	auto cpuTracer = CpuPathTracer< PathTypes::ray_t, PathTypes::accel_t >(scene, opts.path_opts, accel);
	using sampler = PathTypes::sampler_t<decltype(cpuTracer)>;
	auto cudaTracer  = CudaPathTracer< PathTypes::accel_cuda_t>(scene, opts.path_opts, accel.pod);
	
	int cuda_num = 0;
	int cpu_num = 0;
	
	std::thread cpuThread([&views, &views_mutex, &cpuTracer, &scene, &cpu_num] () {
		while (1)
		{
			views_mutex.lock();
			if (views.size() < 1)
				break;
			ImageView currentView = views.front();
			views.pop(); // remove the element we got
			views_mutex.unlock();
			
			sampler::render(scene, currentView, cpuTracer);
			
			cpu_num++;
		}
	});
	std::thread cudaThread([&views, &views_mutex, &cudaTracer, &cuda_num] () {
		while (1)
		{
			views_mutex.lock();
			if (views.size() < 1)
				break;
			ImageView currentView = views.front();
			views.pop(); // remove the element we got
			views_mutex.unlock();
			
			cudaTracer.render(currentView);
			
			cuda_num++;
		}
	});
	
	cpuThread.join();
	cudaThread.join();
	
	#ifdef DEBUG
		std::cout << cpu_num << " parts process by CPU / " << cuda_num << " parts processed by GPU\n";
	#endif
#else
  ImageView img(image, 0, opts.resolution.h);
	
	auto cpuTracer = CpuPathTracer< PathTypes::ray_t, PathTypes::accel_t >(scene, opts.path_opts, accel);
	using sampler = PathTypes::sampler_t<decltype(cpuTracer)>;
	sampler::render(scene, img, cpuTracer);
#endif
}

template<class scene_t>
static int trace(const char *outpath, RenderOptions &opts, StageLogger &logger) {
	Image image(opts.resolution);
#ifdef WITH_PROGRESS
	logger.image = &image;
#endif
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
	traceScene(scene, image, accel, opts);

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
	if (argc != 3) {
		cerr << "usage: " << argv[0] << " <input.json> <output.bmp>\n";
		return 1;
	}
	RenderOptions opts;
	StageLogger logger(opts);
	logger.dump_config();
	logger.start();
	if(!LoadJob(argv[1], &opts)) return 1;

	switch (opts.method) {
		case RenderOptions::WHITTED: {
			return trace<WhittedScene>(argv[2], opts, logger);
		}
		case RenderOptions::PATH: {
			return trace<PathScene>(argv[2], opts, logger);
		}
		default:
			return 1;
	}
	return 0;
}
