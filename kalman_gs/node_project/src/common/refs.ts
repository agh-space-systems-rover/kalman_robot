import { createRef } from 'react';



import Alerts from '../components/alerts';
import Modal from '../components/modal';
import Settings from '../components/settings';
import Splash from '../components/splash';


const splashRef = createRef<Splash>();
const alertsRef = createRef<Alerts>();
const settingsRef = createRef<Settings>();
const modalRef = createRef<Modal>();

export { splashRef, alertsRef, settingsRef, modalRef };
