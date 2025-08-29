import Arms from './arm';
import Feeds from './feeds';
import Imu from './imu';
import Map from './map';
import MapHeader from './map.header';
import Mobile from './mobile';
import Supervisor from './supervisor';
import Ueuos from './ueuos';
import Waypoints from './waypoints';
import Wheels from './wheels';
import Science from './science';
import { IconDefinition } from '@fortawesome/fontawesome-svg-core';
import {
  faBullseye,
  faCube,
  faLocationDot,
  faVideo,
  faDiagramProject,
  faFlag,
  faGear,
  faMobileScreenButton,
  faPalette,
  faMicroscope,
  faMagicWandSparkles
} from '@fortawesome/free-solid-svg-icons';
import { Component } from 'react';
import ArmAutonomy from './arm_autonomy';

// Add new panels here:
export type PanelID = 'imu' | 'wheels' | 'supervisor' | 'map' | 'feeds' | 'arm' | 'waypoints'| 'mobile' | 'ueuos' | 'science' | 'arm_autonomy';
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
  mobile: {
    Component: Mobile,
    name: 'Mobile',
    icon: faMobileScreenButton
  },
  ueuos: {
    Component: Ueuos,
    name: 'UEUOS',
    icon: faPalette
  },
  science: {
    Component: Science,
    name: 'Science',
    icon: faMicroscope
  },
  arm_autonomy: {
    Component: ArmAutonomy,
    name: 'Arm Autonomy',
    icon: faMagicWandSparkles
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
