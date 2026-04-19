#pragma once

#include <string>

class MissionFeedbackHandle {
  public:
	virtual ~MissionFeedbackHandle() = default;
	virtual void publish_progress(const std::string &progress) = 0;
};
