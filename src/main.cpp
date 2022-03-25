#include "render.h"
#include "route_model.h"
#include "route_planner.h"
#include <fstream>
#include <io2d.h>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

using namespace std::experimental;

/**
 * @brief Read contents from a file into a byte vector.
 *
 * @param path path to file
 * @return file contents as byte vector
 */
static std::optional<std::vector<std::byte>> ReadFile(const std::string &path) {
  std::ifstream is{path, std::ios::binary | std::ios::ate};
  if (!is)
    return std::nullopt;

  auto size = is.tellg();
  std::vector<std::byte> contents(size);

  is.seekg(0);
  is.read((char *)contents.data(), size);

  if (contents.empty())
    return std::nullopt;
  return std::move(contents);
}

/**
 * @brief Check whether user input is in valid range of [0, 100]
 *
 * @param user_input_x x coordinate input
 * @param user_input_y y coordinate input
 * @return true if input is valid
 * @return false if input is invalid
 */
bool validateInput(int user_input_x, int user_input_y) {
  if (user_input_x < 0 || user_input_x > 100) {
    std::cout << "Your input for x is invalid, please enter a value in range "
                 "[0, 100].\n";
    return false;
  }
  if (user_input_y < 0 || user_input_y > 100) {
    std::cout << "Your input for y is invalid, please enter a value in range "
                 "[0, 100].\n";
    return false;
  }
  return true;
}

/**
 * @brief Get user input for start and end (x,y) coordinates.
 *
 * @param start_x start x coordinate
 * @param start_y start y coordinate
 * @param end_x end x coordinate
 * @param end_y end y coordinate
 */
void getInput(float &start_x, float &start_y, float &end_x, float &end_y) {
  bool inputIsValid = false;

  while (!inputIsValid) {
    std::cout << "Please enter initial x and y coordinate:\n";
    std::cin >> start_x >> start_y;
    inputIsValid = validateInput(start_x, start_y);
    std::cout << "Your input: x = " << start_x << ", y = " << start_y
              << std::endl;
  }

  inputIsValid = false;
  while (!inputIsValid) {
    std::cout << "Please enter final x and y coordinate:\n";
    std::cin >> end_x >> end_y;
    std::cout << "Your input: x = " << end_x << ", y = " << end_y << std::endl;
    inputIsValid = validateInput(end_x, end_y);
  }
}

/**
 * @brief Main application logic.
 *
 * @param argc argument count
 * @param argv argument vector
 * @return int
 */
int main(int argc, const char **argv) {

  // Parse command-line arguments
  std::string osm_data_file = "";
  if (argc > 1) {
    for (int i = 1; i < argc; ++i)
      if (std::string_view{argv[i]} == "-f" && ++i < argc)
        osm_data_file = argv[i];
  } else {
    std::cout << "To specify a map file use the following format: "
              << std::endl;
    std::cout << "Usage: [executable] [-f filename.osm]" << std::endl;
    osm_data_file = "../map.osm";
  }

  std::vector<std::byte> osm_data;

  if (osm_data.empty() && !osm_data_file.empty()) {
    std::cout << "Reading OpenStreetMap data from the following file: "
              << osm_data_file << std::endl;
    auto data = ReadFile(osm_data_file);
    if (!data)
      std::cout << "Failed to read." << std::endl;
    else
      osm_data = std::move(*data);
  }

  // initialize and get user input coordinates
  float start_x, start_y, end_x, end_y;
  getInput(start_x, start_y, end_x, end_y);

  // build model object
  RouteModel model{osm_data};

  // create RoutePlanner object and perform A* search
  RoutePlanner route_planner{model, start_x, start_y, end_x, end_y};
  route_planner.AStarSearch();

  // print total distance to stdout
  std::cout << "Distance: " << route_planner.GetDistance() << " meters. \n";

  // render results of search
  Render render{model};
  auto display = io2d::output_surface{400,
                                      400,
                                      io2d::format::argb32,
                                      io2d::scaling::none,
                                      io2d::refresh_style::fixed,
                                      30};
  display.size_change_callback([](io2d::output_surface &surface) {
    surface.dimensions(surface.display_dimensions());
  });
  display.draw_callback(
      [&](io2d::output_surface &surface) { render.Display(surface); });
  display.begin_show();
}
