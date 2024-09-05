import { ButtonTypes, ContainerState, WeightTypes } from '../panels/science';
import { ros } from './ros';
import { Service, Topic } from 'roslib';
import { CONTAINER1_CLOSE, CONTAINER1_OPEN, SCIENCE_AUTONOMY_DRILL, SCIENCE_REQUEST_DRILL, SCIENCE_REQUEST_ROCKS, SCIENCE_REQUEST_SAMPLE, SCIENCE_TARE_DRILL, SCIENCE_TARE_ROCKS, SCIENCE_TARE_SAMPLE } from './ros-interfaces';

export type UiData = { // weights
  [WeightTypes.Drill]: string,
  [WeightTypes.Rocks]: string,
  [WeightTypes.Sample]: string,
}

export const uiData: UiData = {
  [WeightTypes.Drill]: "0 g",
  [WeightTypes.Rocks]: "0 g",
  [WeightTypes.Sample]: "0 g",
}

export const translateFrame = {
  [0]: WeightTypes.Drill,
  [1]: WeightTypes.Rocks,
  [2]: WeightTypes.Sample,
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
    cmd: CONTAINER1_CLOSE + container*2 + (state^1)
  };

  setScience.callService(req, () => {}, undefined);
}

export function onButtonClicked(whichWeight: WeightTypes, buttonType: ButtonTypes) {
  // send tare/weight/autonomy to driver
  if(setScience == undefined)
    return;
  let cmd = 0;

  switch(whichWeight)
  {
    case WeightTypes.Drill:
      if(buttonType == ButtonTypes.Tare) {
        cmd = SCIENCE_TARE_DRILL;
      }
      else if(buttonType == ButtonTypes.Autonomy) {
        cmd = SCIENCE_AUTONOMY_DRILL;
      }
      else if(buttonType == ButtonTypes.Request) {
        cmd = SCIENCE_REQUEST_DRILL;
      }
      break;

    case WeightTypes.Rocks:
      if(buttonType == ButtonTypes.Tare) {
        cmd = SCIENCE_TARE_ROCKS;
      }
      else if(buttonType == ButtonTypes.Request) {
        cmd = SCIENCE_REQUEST_ROCKS;
      }
      break;

    case WeightTypes.Sample:
      if(buttonType == ButtonTypes.Tare) {
        cmd = SCIENCE_TARE_SAMPLE;
      }
      else if(buttonType == ButtonTypes.Request) {
        cmd = SCIENCE_REQUEST_SAMPLE;
      }
      break;   
  }

  setScience.callService({ cmd }, () => {}, undefined);

  console.log("clicked "  + buttonType + " of weight " + whichWeight);
}
