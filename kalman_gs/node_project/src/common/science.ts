import { AutonomyButton, ButtonTypes, ContainerState, WeightTypes } from '../panels/science';
import { ros } from './ros';
import { Service, Topic } from 'roslib';
import { CONTAINER1_CLOSE, CONTAINER1_OPEN, SCIENCE_AUTONOMY_PAUSE, SCIENCE_AUTONOMY_RESET, SCIENCE_AUTONOMY_START, SCIENCE_REQUEST_DRILL, SCIENCE_REQUEST_ROCKS, SCIENCE_REQUEST_SAMPLE, SCIENCE_TARE_DRILL, SCIENCE_TARE_ROCKS, SCIENCE_TARE_SAMPLE } from './ros-interfaces';

export type UiData = { // weights
  [WeightTypes.DRILL]: string,
  [WeightTypes.ROCKS]: string,
  [WeightTypes.SAMPLE]: string,
}

export const uiData: UiData = {
  [WeightTypes.DRILL]: "0 g",
  [WeightTypes.ROCKS]: "0 g",
  [WeightTypes.SAMPLE]: "0 g",
}

export const translateFrame = {
  [0]: WeightTypes.DRILL,
  [1]: WeightTypes.ROCKS,
  [2]: WeightTypes.SAMPLE,
}

export const translateAutonomy = {
  [AutonomyButton.RESET]: SCIENCE_AUTONOMY_RESET,
  [AutonomyButton.PAUSE]: SCIENCE_AUTONOMY_PAUSE,
  [AutonomyButton.PLAY]: SCIENCE_AUTONOMY_START,
}

type WeightMessage = {
  which_weight: number,
  weight: number,
}

function updateUi() { // call when updated uiData
  window.dispatchEvent(new Event('science-updated'));
}

let setScience;

window.addEventListener('ros-connect', () => {
  setScience = new Service({
    ros: ros,
    name: '/request_science',
    serviceType: 'kalman_interfaces/Science'
  });

  const weights = new Topic({
    ros : ros,
    name : '/science/weights',
    messageType : 'kalman_interfaces/ScienceWeight'
  });

  weights.subscribe((message: WeightMessage)=> {
    uiData[translateFrame[message.which_weight]] = String(Math.round(message.weight*1000)/1000) + " g";
    updateUi();
  });
  
});

export function onContainerClicked(state: ContainerState, container: number) {
  if(setScience == undefined)
    return;
  
  const req = {
    cmd: CONTAINER1_CLOSE + container*2 + state
  };
  updateUi(); // because of weird react glitches
  setScience.callService(req, () => {}, undefined);
}

export function onButtonClicked(whichWeight: WeightTypes, buttonType: ButtonTypes) {
  if (!setScience) return;

  const commands = {
      [WeightTypes.DRILL]: { [ButtonTypes.TARE]: SCIENCE_TARE_DRILL, [ButtonTypes.REQUEST]: SCIENCE_REQUEST_DRILL },
      [WeightTypes.ROCKS]: { [ButtonTypes.TARE]: SCIENCE_TARE_ROCKS, [ButtonTypes.REQUEST]: SCIENCE_REQUEST_ROCKS },
      [WeightTypes.SAMPLE]: { [ButtonTypes.TARE]: SCIENCE_TARE_SAMPLE, [ButtonTypes.REQUEST]: SCIENCE_REQUEST_SAMPLE },
  };

  const cmd = commands[whichWeight]?.[buttonType];
  if (cmd) {
      setScience.callService({ cmd }, () => {}, undefined);
  }
}


export function onAutonomyClicked(whatButton: AutonomyButton) {
  if(setScience == undefined)
    return;

  const cmd = translateAutonomy[whatButton];
  setScience.callService({ cmd }, () => {}, undefined);
}