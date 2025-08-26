#include "mission_state.hpp"

#include <rclcpp/logging.hpp>
#include <yaml-cpp/yaml.h>

MissionState::MissionState(PanelLayout &&layout) : layout_{layout} {
	for (const auto &[k, _] : layout_.markers) {
		visited_map_[k] = false;
	}
}

PanelLayout PanelLayout::fake_read() {
	PanelLayout ret;
	ret.board_height = 0.6;
	ret.board_width  = 0.4;

	MarkerInfo m0{.u = 0.0, .v = 0.0, .id = 0};
	MarkerInfo m1{.u = 0.26, .v = 0.0, .id = 1};
	MarkerInfo m2{.u = 0.0, .v = 0.383, .id = 2};

	ret.markers[0] = m0;
	ret.markers[1] = m1;
	ret.markers[2] = m2;

	return ret;
}

std::optional<PanelLayout> PanelLayout::read_yaml(
    const std::filesystem::path &yaml_path, rclcpp::Logger logger
) {
	PanelLayout ret{};
	try {
		YAML::Node config = YAML::LoadFile(yaml_path);
		ret.marker_size   = config["marker_size"].as<double>();
		ret.board_width   = config["board_width"].as<double>();
		ret.board_height  = config["board_height"].as<double>();
		YAML::Node m      = config["markers"];
		// markers_.clear();
		for (auto it : m) {
			int        id = it.first.as<int>();
			MarkerInfo mi;
			mi.u            = it.second["u"].as<double>();
			mi.v            = it.second["v"].as<double>();
			ret.markers[id] = mi;
		}
		YAML::Node g = config["gizmos"];
		for (auto it : g) {
			const auto id = it.first.as<std::string>();
			GizmoInfo  mi;
			mi.u           = it.second["u"].as<double>();
			mi.v           = it.second["v"].as<double>();
			ret.gizmos[id] = mi;
		}
		RCLCPP_INFO(
		    logger,
		    "Loaded %zu markers from %s",
		    ret.markers.size(),
		    yaml_path.c_str()
		);
	} catch (const std::exception &e) {
		RCLCPP_FATAL(
		    logger, "Failed to load YAML '%s': %s", yaml_path.c_str(), e.what()
		);
		throw; // Kill the program
		return {};
	}
	return ret;
}
