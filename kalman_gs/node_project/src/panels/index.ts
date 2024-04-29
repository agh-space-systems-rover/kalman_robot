import Imu from './imu';
import Map from './map';
import MapHeader from './map.header';
import Supervisor from './supervisor';
import Wheels from './wheels';
import { IconDefinition } from '@fortawesome/fontawesome-svg-core';
import {
  faBullseye,
  faCube,
  faLocationDot,
  faWheelchair
} from '@fortawesome/free-solid-svg-icons';

// Add new panels here:
export type PanelID = 'imu' | 'wheels' | 'supervisor' | 'map';
export const defaultPanel: PanelID = 'map';
export const panelInfos = {
  imu: {
    Component: Imu,
    name: 'IMU',
    icon: faCube
  },
  wheels: {
    Component: Wheels,
    name: 'Wheels',
    icon: faWheelchair
  },
  supervisor: {
    Component: Supervisor,
    name: 'Supervisor',
    icon: faBullseye
  },
  map: {
    Component: Map,
    HeaderComponent: MapHeader,
    name: 'Map',
    icon: faLocationDot
  }
} as Panels;

// type definitions for the panels above
export type Panels = {
  [id in PanelID]: PanelInfo;
};
export type PanelInfo = {
  Component: React.ComponentType<any>;
  HeaderComponent?: React.ComponentType<any>;
  name: string;
  icon: IconDefinition;
};
