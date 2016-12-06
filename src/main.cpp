#include "scene.hpp"
#include "image.hpp"

#include "cpu_tracer.hpp"

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

	CpuTracer tracer(scene);
	tracer.render(image);

	image.save(argv[2]);
}
