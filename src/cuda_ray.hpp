#ifndef PRAY_CUDA_RAY_HPP
#define PRAY_CUDA_RAY_HPP
#pragma once

#include "cuda_scene.hpp"

#include <cuda_runtime_api.h>
#include <cuda.h>

#include <pmmintrin.h>

struct CudaRay {
	using scene_t = CudaScene;
	using dim_t = IntDimension2::dim_t;
	using intersect_t = TriangleIndex;
	using color_t = Color;
	using location_t = Vector3;
	using vec3_t = Vector3;
	using distance_t = float;
	using angle_t = float;
	using bool_t = bool;
	using material_t = MaterialIndex;

	using dim = ConstDim2<1, 1>;

	static constexpr unsigned subrays_count = 1u;

	const Vector3 origin;
	const Vector3 direction;
	const Vector3 dir_inv;

	__device__ CudaRay(location_t origin, Vector3 direction) : origin(origin), direction(direction), dir_inv(1.f / direction.x, 1.f / direction.y, 1.f / direction.z) {}

	__device__ inline bool_t intersectTriangle(const Triangle &triangle, distance_t *out_distance) const
	{
		// http://www.cs.virginia.edu/~gfx/Courses/2003/ImageSynthesis/papers/Acceleration/Fast%20MinimumStorage%20RayTriangle%20Intersection.pdf
		const Vector3 &t_v1 = triangle.vertices[0];
		const Vector3 &t_v2 = triangle.vertices[1];
		const Vector3 &t_v3 = triangle.vertices[2];

		const Vector3 e1 = t_v2 - t_v1;
		const Vector3 e2 = t_v3 - t_v1;

		const Vector3 p = direction.cross(e2);

		const float det = p.dot(e1);

		if(det == approx(0.f)) return false;

		const Vector3 t = origin - t_v1;

		const Vector3 q = t.cross(e1);

		const float u = p.dot(t) / det;
		const float v = q.dot(direction) / det;

		if(u >= 0.f && v >= 0.f && u + v <= 1.f)
		{
			const float distance = q.dot(e2) / det;
			*out_distance = distance;
			return distance >= 0.f;
		}

		return false;
	}
	
	__device__ inline bool_t intersectAABB(const AABox3 &aabb) const
	{
		// http://psgraphics.blogspot.de/2016/02/new-simple-ray-box-test-from-andrew.html
		float t_min = std::numeric_limits<float>::lowest(), t_max = std::numeric_limits<float>::max();
		for(int i=0; i<3; ++i)
		{
			float i_d = 1.f / direction[i];
			float t0 = (aabb.min[i] - origin[i]) * i_d;
			float t1 = (aabb.max[i] - origin[i]) * i_d;
			if(i_d < 0.f) cuda::swap(t0, t1);
			t_min = cuda::max(t_min, t0);
			t_max = cuda::min(t_max, t1);
			if(t_max < t_min) return false;
		}
		return true;
	}

	__device__ location_t getIntersectionPoint(distance_t intersection_distance) const {
		return origin + direction * intersection_distance;
	}

	__device__ Vector3 getSubrayDirection(unsigned subray) const { ASSERT(subray == 0); return direction; }

	__device__ void isDirectionSignEqualForAllSubrays(Vector3 test_sign, bool *out_result) const {
		const auto sign = direction.sign();
		out_result[0] = test_sign.x == sign.x;
		out_result[1] = test_sign.y == sign.y;
		out_result[2] = test_sign.z == sign.z;
	}

	__device__ bool_t intersectAxisPlane(float plane, unsigned axis, distance_t maximum_distance) const {
		const auto o = origin[axis];
		const auto d_inv = 1.f / direction[axis];
		const auto p = plane;

		// solve o + t*d = p -> t = (p - o) / d
		const auto t = (p - o) * d_inv;

		// no t > 0 test for now, breaks if the camera is inside the scene aabb...
		return t < maximum_distance;
	}

	__device__ static inline color_t getMaterialColors(const scene_t &scene, intersect_t triangle) {
		const auto material_index = scene.triangles[triangle].material_index;
		ASSERT(material_index != MaterialIndex_Invalid);
		return scene.materials[material_index].color;
	}

	__device__ static inline vec3_t getNormals(const scene_t &scene, intersect_t triangle) {
		return scene.triangles[triangle].calculateNormal();
	}

	__device__ static void addBackgroundcolor(color_t &result_color, const intersect_t intersected_triangle, const Color bg) {
		// can be no-op because the tracer already returns the background color if there have not been __ANY__ intersections
	}

	__device__ static inline distance_t max_distance() {
		return std::numeric_limits<distance_t>::max();
	}

	__device__ static inline void updateIntersections(intersect_t *intersected_triangle, TriangleIndex triangle_index, distance_t *minimum_distance, distance_t distance) {
		if(distance < *minimum_distance)
		{
			*intersected_triangle = triangle_index;
			*minimum_distance = distance;
		}
	}

	__device__ static inline intersect_t getNoIntersection() {
		return TriangleIndex_Invalid;
	}

	/**
	 * True if angle between v1 and v2 is larger than 90 degrees
	 */
	__device__ static inline bool_t isOppositeDirection(const vec3_t v1, const vec3_t v2) {
		return v1.dot(v2) <= 0.f;
	}
};

#endif // PRAY_CUDA_RAY_HPP
