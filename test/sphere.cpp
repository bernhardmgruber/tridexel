#include <boost/container/static_vector.hpp>
#include <gtest/gtest.h>
#include <glm/glm.hpp>

#include <optional>

#include "../src/tridexel.h"
#include "../src/IO.h"

namespace {
	struct Sphere {
		vec3 center;
		float radius;
	};

	auto solveQuadraticEquation(double a, double b, double c) -> boost::container::static_vector<double, 2> {
		const double discriminat = std::pow(b, 2) - 4 * a * c;
		if (discriminat < 0)
			return {};

		if (discriminat == 0)
			return { -b / 2 * a };

		const auto x1 = (-b - std::sqrt(discriminat)) / 2 * a;
		const auto x2 = (-b + std::sqrt(discriminat)) / 2 * a;
		return { x1, x2 };
	}
}

TEST(sphere, extract) {
	const auto sphere = Sphere{ vec3{0, 0, 0}, 5 };
	const auto box = BoundingBox{ sphere.center - sphere.radius, sphere.center + sphere.radius };

	//std::vector<vec3> hits;

	const auto triangles = tridexel(box, 200, [&](Ray ray, HitCallback hc) {
		// from https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-sphere-intersection
		// solve quadratic equation
		const auto L = ray.origin - sphere.center;
		const auto a = 1.0;
		const auto b = 2 * glm::dot(ray.direction, L);
		const auto c = glm::dot(L, L) - std::pow(sphere.radius, 2);
		const auto solutions = solveQuadraticEquation(a, b, c);
		for (const auto& t : solutions) {
			const auto point = ray.origin + (float)t * ray.direction;
			//hits.push_back(point);
			const auto normal = glm::normalize(point - sphere.center);
			hc(glm::dot(ray.origin, ray.direction) + (float)t, normal);
		}
	});

	//savePoints("hits.ply", hits);
	saveTriangles("sphere.stl", triangles);
}
