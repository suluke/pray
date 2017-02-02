#ifndef PRAY_IMAGE_H
#define PRAY_IMAGE_H
#pragma once

#include "pray/Config.h"
#include <vector>
#include "math.hpp"
#ifdef WITH_PROGRESS
	#include <atomic>
#endif

struct Image
{
	const IntDimension2 resolution;
	std::vector<uint8_t> pixels;
	using dim_t = IntDimension2::dim_t;
	
#ifdef WITH_PROGRESS
	std::atomic<unsigned> writtenPixels{0};
#endif

	Image(const IntDimension2 &res) : resolution(res), pixels(3 * resolution.w * resolution.h) {}

	void setPixel(dim_t x, dim_t y, const Color &c)
	{
		ASSERT(x < resolution.w); ASSERT(y < resolution.h);
		ASSERT(c.r >= 0.f); ASSERT(c.g >= 0.f); ASSERT(c.b >= 0.f);

#ifdef WITH_PROGRESS
		writtenPixels++;
#endif

		pixels[3 * (y * resolution.w + x) + 0] = fast_round(std::min(c.r, 1.f) * 255.f);
		pixels[3 * (y * resolution.w + x) + 1] = fast_round(std::min(c.g, 1.f) * 255.f);
		pixels[3 * (y * resolution.w + x) + 2] = fast_round(std::min(c.b, 1.f) * 255.f);
  }

	Color getPixel(dim_t x, dim_t y) {
		ASSERT(x < resolution.w); ASSERT(y < resolution.h);
		return Color{((float)pixels[3 * (y * resolution.w + x) + 0]) / 255.f,
								((float)pixels[3 * (y * resolution.w + x) + 1]) / 255.f,
								((float)pixels[3 * (y * resolution.w + x) + 2]) / 255.f};
	}

	void fill(const Color &c)
	{
		// no need to optimize...
		for(dim_t y=0; y<resolution.h; ++y)
		for(dim_t x=0; x<resolution.w; ++x)
		{
			setPixel(x, y, c);
		}
	}

	bool save(const std::string &filename) const;
};

/**
 * Defines a sub-view on the image (buffer).
 * In other words: Same as an image, but only operating on a part of a real image
 */
struct ImageView {
	Image &img;
	using dim_t = Image::dim_t;
	dim_t min_y;
	dim_t max_y;
	const IntDimension2 resolution;
	ImageView(Image &img, dim_t min_y, dim_t max_y) : img(img), min_y(min_y), max_y(max_y), resolution(img.resolution.w, max_y - min_y) {
		ASSERT(min_y < max_y); // don't allow empty views
	}

	void setPixel(dim_t x, dim_t y, const Color &c)
	{
		ASSERT(y + min_y < max_y);
		img.setPixel(x, y + min_y, c);
	}

	Color getPixel(dim_t x, dim_t y) {
		return img.getPixel(x,y);
	}

	dim_t getGlobalY(dim_t y) {
		return y + min_y;
	}
};

inline void writeColorToImage(const Color &c, ImageView &img, IntDimension2::dim_t x, IntDimension2::dim_t y) {
	img.setPixel(x, y, c);
}

#endif
