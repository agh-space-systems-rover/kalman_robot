#pragma once

#include <cstddef>
#include <filesystem>
#include <limits>
#include <map>
#include <optional>
#include <rclcpp/logger.hpp>
#include <string>

struct MarkerInfo {
	double u{0.0};
	double v{0.0};
	size_t id = std::numeric_limits<size_t>::max();
};

struct PanelLayout {
	std::map<size_t, MarkerInfo> markers;
	double                       board_width;
	double                       board_height;
	double marker_size;
	static PanelLayout           fake_read();
	static std::optional<PanelLayout>
	read_yaml(const std::filesystem::path &path, rclcpp::Logger logger);
};

struct MissionState {
	PanelLayout            layout_;
	std::map<size_t, bool> visited_map_;

	MissionState(PanelLayout &&layout);
};

struct MissionHelper {
	// Assume mission state is loaded
	MissionState state;

	// :-P
	inline std::optional<size_t> get_unvisited_marker() {
		const auto &visited = state.visited_map_;
		for (const auto &[k, v] : visited) {
			if (!v) {
				return k;
			}
		}
		return {};
	}

	inline std::string to_string() const {
		std::string ret{};
		const auto &visited = state.visited_map_;
		for (const auto &[k, v] : visited) {
			std::string visited_str = v ? "visited" : "not visited";
			ret += "[ " + std::to_string(k) + " " + visited_str + " ] ";
		}
		return ret;
	}
};
