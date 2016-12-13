#include "scene.hpp"
#include "image.hpp"
#include "dummy_acceleration.hpp"
#include "bih.hpp"
#include "cpu_tracer.hpp"
#include "ray.hpp"

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

	CpuTracer<Ray, Bih<Ray>> tracer(scene);
	//CpuTracer<Ray, DummyAcceleration<Ray>> tracer(scene);

	tracer.preprocess();

	tracer.render(img);

	image.save(argv[2]);
}
