import Arms from './arm';
import Feeds from './feeds';
import Imu from './imu';
import Map from './map';
import MapHeader from './map.header';
import Supervisor from './supervisor';
import Ueuos from './ueuos';
import Waypoints from './waypoints';
import Wheels from './wheels';
import { IconDefinition } from '@fortawesome/fontawesome-svg-core';
import {
  faBullseye,
  faCube,
  faLocationDot,
  faVideo,
  faDiagramProject,
  faFlag,
  faGear,
  faPalette
} from '@fortawesome/free-solid-svg-icons';

// Add new panels here:
export type PanelID = 'imu' | 'wheels' | 'supervisor' | 'map' | 'feeds' | 'arm' | 'waypoints' | 'ueuos';
export const defaultPanel: PanelID = 'map';
export const panelInfos: Panels = {
  imu: {
    Component: Imu,
    name: 'IMU',
    icon: faCube
  },
  wheels: {
    Component: Wheels,
    name: 'Wheels',
    icon: faGear
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
  },
  feeds: {
    Component: Feeds,
    name: 'Feeds',
    icon: faVideo
  },
  arm: {
    Component: Arms,
    name: 'Arm',
    icon: faDiagramProject
  },
  waypoints: {
    Component: Waypoints,
    name: 'Waypoints',
    icon: faFlag
  },
  ueuos: {
    Component: Ueuos,
    name: 'UEUOS',
    icon: faPalette
  }
};

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
