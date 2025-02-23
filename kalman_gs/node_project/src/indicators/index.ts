import TopicHealthMonitors from './topic-health-monitor';
import GamepadSelector from './gamepad-selector';
import OpenSettings from './open-settings';
import RebootPC from './reboot-pc';
import RosHealth from './ros-health';
import ThemeSelector from './theme-selector';

// Add new indicators here:
export const IndicatorComponents = [
  TopicHealthMonitors,
  GamepadSelector,
  RebootPC,
  RosHealth,
  OpenSettings
];
