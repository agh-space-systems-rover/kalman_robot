import { createRef } from 'react';

import Alerts from '../components/alerts';
import Settings from '../components/settings';
import Splash from '../components/splash';

const splashRef = createRef<Splash>();
const alertsRef = createRef<Alerts>();
const settingsRef = createRef<Settings>();

export { splashRef, alertsRef, settingsRef };
