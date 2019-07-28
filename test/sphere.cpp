#include <boost/container/static_vector.hpp>
#include <gtest/gtest.h>
#include <glm/glm.hpp>

#include <optional>

#include "../src/tridexel.h"
#include "../src/IO.h"

namespace {
	struct Sphere {
		glm::vec3 center;
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

	void extractSphere(int resolution) {
		const auto sphere = Sphere{glm::vec3{0, 0, 0}, 5};
		const auto box = BoundingBox{sphere.center - sphere.radius, sphere.center + sphere.radius};
		const auto triangles = tridexel(box, resolution, [&](Ray ray, HitCallback hc) {
			// from https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-sphere-intersection
			// solve quadratic equation
			const auto L = ray.origin - sphere.center;
			const auto a = 1.0;
			const auto b = 2 * glm::dot(ray.direction, L);
			const auto c = glm::dot(L, L) - std::pow(sphere.radius, 2);
			const auto solutions = solveQuadraticEquation(a, b, c);
			for (const auto& t : solutions) {
				const auto point = ray.origin + (float)t * ray.direction;
				const auto normal = glm::normalize(point - sphere.center);
				hc(glm::dot(ray.origin, ray.direction) + (float)t, normal);
			}
		});
		saveTriangles("sphere" + std::to_string(resolution) + ".stl", triangles);
	}
}

TEST(sphere, extract5) {
	extractSphere(5);
}

TEST(sphere, extract10) {
	extractSphere(10);
}

TEST(sphere, extract100) {
	extractSphere(100);
}

TEST(sphere, extract200) {
	extractSphere(200);
}
