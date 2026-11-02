// Deterministic stdin/stdout bridge to the same CPU OHM occupancy integrator.
// Little-endian protocol: magic OHMDEM01; requests uint64 N, origin[3],
// N world endpoints[3] (float64); replies uint64 M, M occupied centres[3].
// A clean EOF ends the session. No ROS transport or simulated clock is needed.
#include <ohm/OccupancyMap.h>
#include <ohm/RayMapperOccupancy.h>
#include <ohm/Voxel.h>
#include <ohm/VoxelOccupancy.h>
#include <ohm/MapLayout.h>
#include <glm/glm.hpp>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <vector>

static void read_exact(char *data, std::size_t size) {
  if (!std::cin.read(data, size)) throw std::runtime_error("Truncated request");
}

int main(int argc, char **argv) {
  try {
    if (argc != 2) throw std::runtime_error("Usage: ohm_offline_mapper RESOLUTION_METRES");
    const double resolution = std::stod(argv[1]);
    const std::uint16_t endian = 1;
    if (*reinterpret_cast<const char *>(&endian) != 1 || sizeof(double) != 8)
      throw std::runtime_error("Protocol requires little-endian IEEE float64");
    if (!std::isfinite(resolution) || resolution <= 0)
      throw std::runtime_error("Invalid resolution");
    ohm::OccupancyMap map(resolution);
    ohm::RayMapperOccupancy mapper(&map);
    std::cout.write("OHMDEM01", 8).flush();
    while (std::cin.peek() != EOF) {
      std::uint64_t count;
      read_exact(reinterpret_cast<char *>(&count), sizeof(count));
      if (count > 2000000) throw std::runtime_error("Too many endpoints in one request");
      double origin[3];
      read_exact(reinterpret_cast<char *>(origin), sizeof(origin));
      for (double value : origin)
        if (!std::isfinite(value)) throw std::runtime_error("Nonfinite origin");
      std::vector<glm::dvec3> rays;
      rays.reserve(count * 2);
      for (std::uint64_t i = 0; i < count; ++i) {
        double point[3];
        read_exact(reinterpret_cast<char *>(point), sizeof(point));
        for (double value : point)
          if (!std::isfinite(value)) throw std::runtime_error("Nonfinite endpoint");
        rays.emplace_back(origin[0], origin[1], origin[2]);
        rays.emplace_back(point[0], point[1], point[2]);
      }
      if (!rays.empty()) mapper.integrateRays(rays.data(), rays.size(), nullptr, nullptr, 0);
      std::vector<double> centres;
      ohm::Voxel<float> occupancy(&map, map.layout().occupancyLayer());
      for (auto it = map.begin(); it != map.end(); ++it) {
        occupancy.setKey(it.key());
        if (ohm::isOccupied(occupancy)) {
          const auto centre = map.voxelCentreGlobal(it.key());
          centres.insert(centres.end(), {centre.x, centre.y, centre.z});
        }
      }
      const std::uint64_t size = centres.size() / 3;
      std::cout.write(reinterpret_cast<const char *>(&size), sizeof(size));
      std::cout.write(reinterpret_cast<const char *>(centres.data()), centres.size() * sizeof(double));
      std::cout.flush();
      if (!std::cout) throw std::runtime_error("Output stream closed");
    }
  } catch (const std::exception &error) {
    std::cerr << "offline OHM: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
