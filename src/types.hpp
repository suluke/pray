#ifndef PRAY_TYPES_HPP
#define PRAY_TYPES_HPP
#pragma once

#include "pray/Config.h"
#include "dummy_acceleration.hpp"
#include "bih.hpp"
#include "kdtree.hpp"
#include "ray.hpp"
#include "sse_ray.hpp"
#include "sampler.hpp"

template<class scene_t>
struct PrayTypes {
#ifdef WITH_SSE
	using ray_t = SSERay<scene_t>;
#else
	using ray_t = Ray<scene_t>;
#endif
	using accel_t = ACCELERATOR<ray_t, scene_t>;
	using dummy_accel_t = DummyAcceleration<ray_t, scene_t>;
	template <class tracer_t>
	using sampler_t = SAMPLER<scene_t, tracer_t, ray_t>;
};
#ifndef WITH_SSE_PT
template<>
struct PrayTypes<PathScene> {
	using scene_t = PathScene;
	using ray_t = Ray<scene_t>;
	using accel_t = ACCELERATOR<ray_t, scene_t>;
	using dummy_accel_t = DummyAcceleration<ray_t, scene_t>;
	template <class tracer_t>
	using sampler_t = SAMPLER<scene_t, tracer_t, ray_t>;
};
#endif // not WITH_SSE_PT

using WhittedTypes = PrayTypes<WhittedScene>;
using PathTypes = PrayTypes<PathScene>;

#endif // PRAY_TYPES_HPP
