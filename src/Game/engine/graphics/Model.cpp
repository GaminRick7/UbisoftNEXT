#include "Model.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include "../../math/vec3.h"

bool Model::loadFromOBJ(std::string fileName)
{
	std::ifstream file(fileName);
	if (!file.is_open())
	{
		return false;
	}

	std::vector<Vec3> vertices;
	std::string line;
	bool isZeroBased = false;
	bool indexFormatDetected = false;

	while (std::getline(file, line))
	{
		// skip empty lines and comments
		if (line.empty() || line[0] == '#')
		{
			continue;
		}

		std::istringstream iss(line);
		std::string type;
		iss >> type;

		// parse vertex position
		if (type == "v")
		{
			float x, y, z;
			if (iss >> x >> y >> z)
			{
				vertices.push_back(Vec3(x, y, z));
			}
		}
		// parse face
		else if (type == "f")
		{
			std::string vertexData;
			int indices[3];

			int vertexCount = 0;
			while (iss >> vertexData && vertexCount < 3)
			{
				std::istringstream vertexStream(vertexData);
				int vertexIndex;

				// extract vertex index -- stops at '/'
				if (vertexStream >> vertexIndex)
				{
					// detect if file uses 0-based indexing
					if (!indexFormatDetected && vertexCount == 0)
					{
						isZeroBased = (vertexIndex == 0);
						indexFormatDetected = true;
					}
					
					// convert to 0-based indexing
					if (isZeroBased)
					{
						indices[vertexCount] = vertexIndex;
					}
					else
					{
						// obj usually uses 1-based indexing
						indices[vertexCount] = vertexIndex - 1;
					}
					vertexCount++;
				}
			}

			if (vertexCount == 3)
			{
				Triangle triangle;
				triangle.vertices[0] = vertices[indices[0]];
				triangle.vertices[1] = vertices[indices[1]];
				triangle.vertices[2] = vertices[indices[2]];
				triangles.push_back(triangle);
			}
		}
	}

	file.close();

	if (triangles.empty())
	{
		std::cerr << "No triangles loaded from OBJ file: " << fileName << std::endl;
		return false;
	}

	// generate bounds from all triangle vertices
	if (!triangles.empty())
	{
		Vec3 minBounds = triangles[0].vertices[0];
		Vec3 maxBounds = triangles[0].vertices[0];

		// find min/max across all vertices in all triangles
		for (const auto& triangle : triangles)
		{
			for (int i = 0; i < 3; i++)
			{
				const Vec3& vertex = triangle.vertices[i];
				
				if (vertex.x < minBounds.x) minBounds.x = vertex.x;
				if (vertex.y < minBounds.y) minBounds.y = vertex.y;
				if (vertex.z < minBounds.z) minBounds.z = vertex.z;
				
				if (vertex.x > maxBounds.x) maxBounds.x = vertex.x;
				if (vertex.y > maxBounds.y) maxBounds.y = vertex.y;
				if (vertex.z > maxBounds.z) maxBounds.z = vertex.z;
			}
		}

		bounds = AABB(minBounds, maxBounds);
	}

	return true;
}