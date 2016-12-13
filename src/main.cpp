#include "scene.hpp"
#include "image.hpp"
#include "dummy_acceleration.hpp"
#include "bih.hpp"
#include "cpu_tracer.hpp"

#ifdef WITH_SSE
	#include "ray.hpp"
	using ray_t = Ray;
	//~ #include "sse_ray.hpp"
	//~ using ray_t = SSERay;
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

	Image image(image_resolution);
	ImageView img(image, 0, image_resolution.h);

	CpuTracer<ray_t, accel_t> tracer(scene);

	tracer.preprocess();
	tracer.render(img);

	image.save(argv[2]);
}
