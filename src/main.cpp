#include "scene.hpp"
#include "image.hpp"
#include "dummy_acceleration.hpp"
#include "bih.hpp"
#include "cpu_tracer.hpp"
#include "math.hpp"

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

int main(int argc, char *argv[])
{
	if (argc != 3)
	{
		std::cerr << "usage: " << argv[0] << " <input.json> <output.bmp>\n";
		return 1;
	}

	Scene scene;

	IntDimension2 image_resolution = IntDimension2(1920, 1080);
	if(!scene.load(argv[1], &image_resolution)) return 1;
#ifdef DEBUG
	time_t start1 = startTime();
#endif
	Image image(image_resolution);
	ImageView img(image, 0, image_resolution.h);
	CpuTracer<ray_t, accel_t> tracer(scene);
#ifdef DEBUG
	time_t start3 = startTime();
#endif
	tracer.preprocess();
#ifdef DEBUG
	time_t end1 = startTime();
	time_t start4 = startTime();
#endif
	tracer.render(img);
#ifdef DEBUG
	time_t end2 = startTime();
#endif
	image.save(argv[2]);

#ifdef DEBUG
	time_t end3 = startTime();
	std::cout <<"Preprocess Time: " << diffTime(start3,end1) << "\n";
	std::cout <<"Render Time: " << diffTime(start4,end2) << "\n";
	std::cout <<"Total Time: " << diffTime(start1,end3) << "\n";
#endif
}
