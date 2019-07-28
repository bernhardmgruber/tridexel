#pragma once

#include <filesystem>

#include "../src/tridexel.h"

void saveTriangles(std::filesystem::path path, const std::vector<Triangle>& triangles);
void savePoints(std::filesystem::path path, const std::vector<glm::vec3>& points);
