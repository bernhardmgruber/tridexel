#pragma once

#include <glm/vec3.hpp>
#include <glm/geometric.hpp>

#include <vector>
#include <array>
#include <functional>

#include "raycast.h"

using glm::vec3;
using glm::uvec3;

struct BoundingBox {
	vec3 lower;
	vec3 upper;
};

struct Plane {
	Plane(vec3 n, float d) : n(n), d(d) {}
	Plane(vec3 n, vec3 p) : n(n), d(glm::dot(n, p)) {}

	vec3 n;
	float d;
};

struct Triangle : std::array<vec3, 3> {
};

struct Node {
	float depth;
	vec3 normal;
};

struct Segment {
	Node start;
	Node end;
};

struct Dexel {
	void regularize() {
		// TODO: improve this
		if (nodes.size() % 2 != 0)
			nodes.clear();
	}

	auto segments() const -> std::vector<Segment> {
		assert(nodes.size() % 2 == 0);
		std::vector<Segment> segs;
		segs.reserve(nodes.size() / 2);
		for (size_t i = 0; i < nodes.size(); i += 2)
			segs.push_back({nodes[i], nodes[i + 1]});
		return segs;
	}

	std::vector<Node> nodes;
};

struct DexelImage {
	unsigned int axis0;
	unsigned int axis1Res;
	unsigned int axis2Res;
	std::vector<Dexel> dexels;

	auto dexel(unsigned int axis1, unsigned int axis2) -> Dexel& {
		return dexels[axis1Res * axis1 + axis2];
	}

	auto dexel(unsigned int axis1, unsigned int axis2) const -> const Dexel& {
		return dexels[axis1Res * axis1 + axis2];
	}
};

class TriDexelImage {
public:
	TriDexelImage(uvec3 resolution)
		: resolution(resolution) {
		for (auto axis0 : { 0, 1, 2 }) {
			const auto axis1 = (axis0 + 1) % 3;
			const auto axis2 = (axis0 + 2) % 3;
			images[axis0].axis0 = axis0;
			images[axis0].axis1Res = resolution[axis1];
			images[axis0].axis2Res = resolution[axis2];
			images[axis0].dexels.resize(resolution[axis1] * resolution[axis2]);
		}
	}
	
	std::array<DexelImage, 3> images;
	uvec3 resolution;
};

struct boool {
	bool value;

	auto operator=(bool b) -> boool& {
		value = b;
		return *this;
	}

	operator bool() const {
		return value;
	}
};

struct Cell {
	BoundingBox box;
	std::array<bool, 8> occupancies;
	std::array<std::vector<Segment>, 12> edges;

	auto realPoint(unsigned int i) const -> vec3 {
		const auto& u = box.upper;
		const auto& l = box.lower;
		switch (i) {
		case 0: return {l.x, l.y, l.z};
		case 1: return {u.x, l.y, l.z};
		case 2: return {l.x, u.y, l.z};
		case 3: return {u.x, u.y, l.z};
		case 4: return {l.x, l.y, u.z};
		case 5: return {u.x, l.y, u.z};
		case 6: return {l.x, u.y, u.z};
		case 7: return {u.x, u.y, u.z};
		default: std::terminate();
		}
	}

	auto empty() const {
		return std::all_of(begin(occupancies), end(occupancies), [](bool o) { return o == false; });
	}
};

class TriDexelGrid {
public:
	TriDexelGrid(TriDexelImage& image, BoundingBox box)
		: image(image), box(box) {
		const auto r = image.resolution + 1u;
		occupancies.resize(r.x * r.y * r.z);
	}

	auto& occupancy(uvec3 index) {
		const auto& r = image.resolution;
		return occupancies[index.z * (r.x * r.y) + index.y * r.x + index.x];
	}

	auto cell(uvec3 index) -> Cell;

	TriDexelImage& image;
	BoundingBox box;

	std::vector<boool> occupancies;
};

using HitCallback = std::function<void(float, vec3)>;
using RaycastCallback = std::function<void(Ray, HitCallback)>;

auto tridexel(BoundingBox box, unsigned int resolution, RaycastCallback rcc) -> std::vector<Triangle>;
