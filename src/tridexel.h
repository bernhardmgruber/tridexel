#pragma once

#include <glm/vec3.hpp>
#include <glm/geometric.hpp>

#include <vector>
#include <array>
#include <functional>

struct Ray {
	glm::vec3 origin;
	glm::vec3 direction;
};

struct BoundingBox {
	glm::vec3 lower;
	glm::vec3 upper;
};

struct Triangle : std::array<glm::vec3, 3> {};

using HitCallback = std::function<void(float, glm::vec3)>;
using RaycastCallback = std::function<void(Ray, HitCallback)>;

auto tridexel(BoundingBox box, unsigned int resolution, RaycastCallback rcc) -> std::vector<Triangle>;
