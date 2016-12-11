#ifndef PRAY_IMAGE_H
#define PRAY_IMAGE_H
#pragma once

#include <vector>
#include "math.hpp"

struct ImageView;
struct Color
{
	float r, g, b;

	constexpr Color() : r(0.f), g(0.f), b(0.f){}
	constexpr Color(float r, float g, float b) : r(r), g(g), b(b) {}

	constexpr Color operator+(const Color &a) const { return Color(r+a.r, g+a.g, b+a.b); }
	constexpr Color operator*(const Color &a) const { return Color(r*a.r, g*a.g, b*a.b); }
	constexpr Color operator*(float a) const { return Color(r*a, g*a, b*a); }

	Color &operator+=(const Color &a) { return *this = *this + a; }
	void writeToImage(ImageView &img, IntDimension2::dim_t x, IntDimension2::dim_t y) const;
};

struct Image
{
	const IntDimension2 resolution;
	std::vector<uint8_t> pixels;
	using dim_t = IntDimension2::dim_t;

	Image(const IntDimension2 &res) : resolution(res), pixels(3 * resolution.w * resolution.h) {}

	void setPixel(dim_t x, dim_t y, const Color &c)
	{
		ASSERT(x < resolution.w); ASSERT(y < resolution.h);
		ASSERT(c.r >= 0.f); ASSERT(c.g >= 0.f); ASSERT(c.b >= 0.f);

		pixels[3 * (y * resolution.w + x) + 0] = std::round(std::min(c.r, 1.f) * 255.f);
		pixels[3 * (y * resolution.w + x) + 1] = std::round(std::min(c.g, 1.f) * 255.f);
		pixels[3 * (y * resolution.w + x) + 2] = std::round(std::min(c.b, 1.f) * 255.f);
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

	dim_t getGlobalY(dim_t y) {
		return y + min_y;
	}
};

inline void Color::writeToImage(ImageView &img, IntDimension2::dim_t x, IntDimension2::dim_t y) const {
	img.setPixel(x, y, *this);
}

#endif
