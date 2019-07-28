#include "IO.h"

#include <fstream>

namespace {
	auto isFinite(glm::vec3 v) -> bool {
		return std::isfinite(v.x) || std::isfinite(v.y) || std::isfinite(v.z);
	}

	auto isFinite(const Triangle& t) -> bool {
		return isFinite(t[0]) || isFinite(t[1]) || isFinite(t[2]);
	}
}

void saveTriangles(std::filesystem::path path, const std::vector<Triangle>& triangles) {
	if (path.has_parent_path())
		create_directories(path.parent_path());
	std::ofstream f{path, std::ios::binary};
	const char header[80] = "STL whatever";
	f.write(header, sizeof(header));

	uint32_t count = static_cast<uint32_t>(triangles.size());
	f.write(reinterpret_cast<const char*>(&count), sizeof(count));

	const uint16_t attributeCount = 0;
	for (const auto& t : triangles) {
		const auto normal = glm::normalize(glm::cross(t[0] - t[1], t[0] - t[2]));
		if (!isFinite(t)) {
			count--;
			continue;
		}
		f.write(reinterpret_cast<const char*>(&normal), sizeof(normal));
		f.write(reinterpret_cast<const char*>(&t), sizeof(t));
		f.write(reinterpret_cast<const char*>(&attributeCount), sizeof(attributeCount));
	}

	f.seekp(sizeof(header), std::ios::beg);
	f.write(reinterpret_cast<const char*>(&count), sizeof(count));

	f.close();
}

void savePoints(std::filesystem::path path, const std::vector<glm::vec3>& points) {
	if (path.has_parent_path())
		create_directories(path.parent_path());
	std::ofstream f{path, std::ios::binary};
	f << "ply\n";
	f << "format binary_little_endian 1.0\n";
	f << "element vertex " << points.size() << "\n";
	f << "property float x\n";
	f << "property float y\n";
	f << "property float z\n";
	f << "end_header\n";
	f.write(reinterpret_cast<const char*>(points.data()), points.size() * sizeof(glm::vec3));
	f.close();
}
