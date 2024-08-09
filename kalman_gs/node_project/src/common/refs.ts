import { createRef } from 'react';

import Splash from '../components/splash';
import Alerts from '../components/alerts';
import Settings from '../components/settings';

const splashRef = createRef<Splash>();
const alertsRef = createRef<Alerts>();
const settingsRef = createRef<Settings>();

export { splashRef, alertsRef, settingsRef };
