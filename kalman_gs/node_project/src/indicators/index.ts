import EStopButton from './e-stop-button';
import GamepadSelector from './gamepad-selector';
import HealthStatus from './health-status';
import Interstellar from './interstellar';
import OpenSettings from './open-settings';
import RebootPC from './reboot-pc';
import RosHealth from './ros-health';
import RefreshButton from './refresh-button';

// Add new indicators here:
export const IndicatorComponents = [
  Interstellar,
  RefreshButton,
  EStopButton,
  GamepadSelector,
  RebootPC,
  HealthStatus,
  RosHealth,
  OpenSettings
];
