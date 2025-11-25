import styles from './supervisor.module.css';

import { mapMarker } from '../common/map-marker';
import { alertsRef } from '../common/refs';
import { ros } from '../common/ros';
import {
  SupervisorGpsArUcoSearch,
  SupervisorGpsArUcoSearchFeedback,
  SupervisorGpsGoal,
  SupervisorGpsGoalFeedback,
  SupervisorGpsYoloSearch,
  SupervisorGpsYoloSearchFeedback,
  SupervisorTfGoal,
  SupervisorTfGoalFeedback
} from '../common/ros-interfaces';
import {
  faArrowsLeftRight,
  faArrowsUpDown,
  faBarcode,
  faBrain,
  faCopy,
  faLocationCrosshairs,
  faLocationDot
} from '@fortawesome/free-solid-svg-icons';
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome';
import { useCallback, useEffect, useRef, useState } from 'react';
import { Action } from 'roslib';

import Button from '../components/button';
import Dropdown from '../components/dropdown';
import Input from '../components/input';
import Label from '../components/label';

const NO_FEEDBACK_TIMEOUT = 20000;
const TF_GOAL_FRAMES = ['base_link', 'odom', 'map', 'utm'];

// Add new mission types here:
// Search for "Add new mission types here" in the code to find more such places.
let tfGoalClient: Action<SupervisorTfGoal, SupervisorTfGoalFeedback, {}>;
let gpsGoalClient: Action<SupervisorGpsGoal, SupervisorGpsGoalFeedback, {}>;
let gpsArUcoSearchClient: Action<SupervisorGpsArUcoSearch, SupervisorGpsArUcoSearchFeedback, {}>;
let gpsYoloSearchClient: Action<SupervisorGpsYoloSearch, SupervisorGpsYoloSearchFeedback, {}>;
window.addEventListener('ros-connect', () => {
  tfGoalClient = new Action({
    ros: ros,
    name: '/supervisor/tf_goal',
    actionType: 'kalman_interfaces/SupervisorTfGoal'
  });
  gpsGoalClient = new Action({
    ros: ros,
    name: '/supervisor/gps_goal',
    actionType: 'kalman_interfaces/SupervisorGpsGoal'
  });
  gpsArUcoSearchClient = new Action({
    ros: ros,
    name: '/supervisor/gps_aruco_search',
    actionType: 'kalman_interfaces/SupervisorGpsArUcoSearch'
  });
  gpsYoloSearchClient = new Action({
    ros: ros,
    name: '/supervisor/gps_yolo_search',
    actionType: 'kalman_interfaces/SupervisorGpsYoloSearch'
  });
});
type CurrentGoal = {
  client: typeof tfGoalClient | typeof gpsGoalClient | typeof gpsArUcoSearchClient | typeof gpsYoloSearchClient;
  id: string;
};
let currentGoal: CurrentGoal = null;
type LastFeedback = {
  missionType: number;
  feedback:
    | SupervisorTfGoalFeedback
    | SupervisorGpsGoalFeedback
    | SupervisorGpsArUcoSearchFeedback
    | SupervisorGpsYoloSearchFeedback;
};
let lastFeedback: LastFeedback = null;
let noFeedbackCheckTimeout: NodeJS.Timeout = null;

type Props = {
  props: {
    missionType: number;
  };
};

function formatLatitude(lat: number): string {
  if (lat < 0) {
    return (-lat).toFixed(5) + '°S';
  } else {
    return lat.toFixed(5) + '°N';
  }
}

function formatLongitude(long: number): string {
  if (long < 0) {
    return (-long).toFixed(5) + '°W';
  } else {
    return long.toFixed(5) + '°E';
  }
}

function formatSnakeCaseState(state: string): string {
  const words = state.split('_');
  for (let i = 0; i < words.length; i++) {
    words[i] = words[i].charAt(0).toUpperCase() + words[i].slice(1);
  }
  return words.join(' ');
}

export default function Supervisor({ props }: Props) {
  if (props.missionType === undefined) {
    props.missionType = 0;
  }

  const [missionType, setMissionType] = useState(props.missionType);
  const [_, setRerenderCounter] = useState(0);

  const rerender = useCallback(() => {
    setRerenderCounter((count) => count + 1);
  }, [setRerenderCounter]);

  useEffect(() => {
    window.addEventListener('supervisor-rerender', rerender);
    return () => {
      window.removeEventListener('supervisor-rerender', rerender);
    };
  }, [rerender]);

  const rerenderAllSupervisorPanels = useCallback(() => {
    window.dispatchEvent(new Event('supervisor-rerender'));
  }, []);

  const inputRefs = Array.from({ length: 4 }, () => useRef<Input>(null));
  const tfGoalFrameIndex = useRef<number>(0);

  const setLatLongFromMapMarker = useCallback(
    (latRefI: number, longRefI: number) => {
      if (inputRefs[latRefI].current && inputRefs[longRefI].current) {
        const latTrunc = parseFloat(mapMarker.latitude.toFixed(8));
        const longTrunc = parseFloat(mapMarker.longitude.toFixed(8));
        inputRefs[latRefI].current.setValue(latTrunc);
        inputRefs[longRefI].current.setValue(longTrunc);
      }
    },
    [inputRefs]
  );

  const style = getComputedStyle(document.body);
  const redBg = style.getPropertyValue('--red-background');
  const greenBg = style.getPropertyValue('--green-background');
  const blueBg = style.getPropertyValue('--blue-background');
  const magentaBg = style.getPropertyValue('--magenta-background');
  const bgColor = style.getPropertyValue('--background');
  const darkBg = style.getPropertyValue('--dark-background');

  return (
    <div className={styles['supervisor']}>
      <div className={styles['supervisor-rows']}>
        <div className={styles['supervisor-row']}>
          <Dropdown
            className={styles['supervisor-row-item']}
            tooltip='Select the type of mission to start.'
            items={[
              // Add new mission types here:
              {
                icon: faLocationCrosshairs,
                text: 'TF Goal'
              },
              {
                icon: faLocationDot,
                text: 'GPS Goal'
              },
              {
                icon: faBarcode,
                text: 'GPS ArUco Search'
              },
              {
                icon: faBrain,
                text: 'GPS YOLO Search'
              }
            ]}
            defaultItemIndex={missionType}
            onChange={(i) => {
              setMissionType(i);
              props.missionType = i;
            }}
          />
        </div>

        {/* Add new mission types here: */}

        {missionType === 0 && (
          <>
            <div className={styles['supervisor-row']}>
              <Label color={redBg}>X</Label>
              <Input ref={inputRefs[0]} type='float' placeholder='Front' />
              <Label color={greenBg}>Y</Label>
              <Input ref={inputRefs[1]} type='float' placeholder='Left' />
              <Label color={blueBg}>Z</Label>
              <Input ref={inputRefs[2]} type='float' placeholder='Up' />
            </div>
            <div className={styles['supervisor-row']}>
              <Label color={magentaBg}>&nbsp;Frame ID&nbsp;</Label>
              <Dropdown
                className={styles['supervisor-row-item']}
                tooltip='The frame of reference for the goal location.'
                items={TF_GOAL_FRAMES.map((frameId) => ({
                  text: frameId
                }))}
                defaultItemIndex={tfGoalFrameIndex.current}
                onChange={(i) => {
                  tfGoalFrameIndex.current = i;
                }}
              />
            </div>
          </>
        )}

        {missionType === 1 && (
          <>
            <div className={styles['supervisor-row']}>
              <Label color={greenBg}>
                <FontAwesomeIcon icon={faArrowsUpDown} />
              </Label>
              <Input ref={inputRefs[0]} type='float' placeholder='Latitude' />
              <Label color={redBg}>
                <FontAwesomeIcon icon={faArrowsLeftRight} />
              </Label>
              <Input ref={inputRefs[1]} type='float' placeholder='Longitude' />
              <Button tooltip='Set goal location from map marker.' onClick={() => setLatLongFromMapMarker(0, 1)}>
                <FontAwesomeIcon icon={faLocationDot} />
              </Button>
            </div>
          </>
        )}

        {missionType === 2 && (
          <>
            <div className={styles['supervisor-row']}>
              <Label color={greenBg}>
                <FontAwesomeIcon icon={faArrowsUpDown} />
              </Label>
              <Input ref={inputRefs[0]} type='float' placeholder='Initial Latitude' />
              <Label color={redBg}>
                <FontAwesomeIcon icon={faArrowsLeftRight} />
              </Label>
              <Input ref={inputRefs[1]} type='float' placeholder='Initial Longitude' />
              <Button tooltip='Set initial location from map marker.' onClick={() => setLatLongFromMapMarker(0, 1)}>
                <FontAwesomeIcon icon={faLocationDot} />
              </Button>
            </div>
            <div className={styles['supervisor-row']}>
              <Label color={magentaBg}>&nbsp;Marker ID&nbsp;</Label>
              <Input ref={inputRefs[2]} type='float' placeholder='e.g. 42' className={styles['supervisor-row-item']} />
            </div>
          </>
        )}

        {missionType === 3 && (
          <>
            <div className={styles['supervisor-row']}>
              <Label color={greenBg}>
                <FontAwesomeIcon icon={faArrowsUpDown} />
              </Label>
              <Input ref={inputRefs[0]} type='float' placeholder='Initial Latitude' />
              <Label color={redBg}>
                <FontAwesomeIcon icon={faArrowsLeftRight} />
              </Label>
              <Input ref={inputRefs[1]} type='float' placeholder='Initial Longitude' />
              <Button tooltip='Set initial location from map marker.' onClick={() => setLatLongFromMapMarker(0, 1)}>
                <FontAwesomeIcon icon={faLocationDot} />
              </Button>
            </div>
            <div className={styles['supervisor-row']}>
              <Label color={magentaBg}>&nbsp;Class Name&nbsp;</Label>
              <Input ref={inputRefs[2]} placeholder='e.g. bottle' className={styles['supervisor-row-item']} />
            </div>
          </>
        )}

        <div className={styles['supervisor-row']}>
          <Button
            tooltip='WARNING: The robot will start moving!'
            className={
              styles['supervisor-row-item'] +
              (lastFeedback !== null ? ` ${styles['supervisor-start-button-above-feedback']}` : '') +
              (currentGoal !== null && lastFeedback === null ? ` ${styles['supervisor-start-button-disabled']}` : '')
            }
            onClick={() => {
              if (currentGoal !== null) {
                if (noFeedbackCheckTimeout) {
                  clearTimeout(noFeedbackCheckTimeout);
                }
                currentGoal.client.cancelGoal(currentGoal.id);
                currentGoal = null;
                rerenderAllSupervisorPanels();
                return;
              }

              // Create the goal and choose an appropriate client.
              // Add new mission types here:
              let goal: SupervisorTfGoal | SupervisorGpsGoal | SupervisorGpsArUcoSearch | SupervisorGpsYoloSearch;
              let client: typeof currentGoal.client;
              if (missionType === 0) {
                const x: number = inputRefs[0].current.getValue() || 0;
                const y: number = inputRefs[1].current.getValue() || 0;
                const z: number = inputRefs[2].current.getValue() || 0;
                let frameId: string = TF_GOAL_FRAMES[tfGoalFrameIndex.current];

                goal = {
                  location: {
                    header: {
                      frame_id: frameId
                    },
                    point: { x, y, z }
                  }
                };
                client = tfGoalClient;
              } else if (missionType === 1) {
                const latitude: number = inputRefs[0].current.getValue();
                const longitude: number = inputRefs[1].current.getValue();

                if (latitude === undefined || longitude === undefined) {
                  alertsRef.current?.pushAlert('Please provide both latitude and longitude.');
                  return;
                }

                goal = {
                  location: {
                    latitude: latitude,
                    longitude: longitude
                  }
                };
                client = gpsGoalClient;

                console.log('goal', goal);
              } else if (missionType === 2) {
                const initialLatitude: number = inputRefs[0].current.getValue();
                const initialLongitude: number = inputRefs[1].current.getValue();
                const markerId: number = inputRefs[2].current.getValue();

                if (!initialLatitude || !initialLongitude || markerId === undefined) {
                  alertsRef.current?.pushAlert('Please provide initial latitude, longitude, and marker ID.');
                  return;
                }

                goal = {
                  initial_location: {
                    latitude: initialLatitude,
                    longitude: initialLongitude
                  },
                  marker_id: markerId
                };
                client = gpsArUcoSearchClient;
              } else if (missionType === 3) {
                const initialLatitude: number = inputRefs[0].current.getValue();
                const initialLongitude: number = inputRefs[1].current.getValue();
                const className: string = inputRefs[2].current.getValue();

                if (!initialLatitude || !initialLongitude || !className) {
                  alertsRef.current?.pushAlert('Please provide initial latitude, longitude, and class name.');
                  return;
                }

                goal = {
                  initial_location: {
                    latitude: initialLatitude,
                    longitude: initialLongitude
                  },
                  object_class: className
                };
                client = gpsYoloSearchClient;
              }

              // Check whether the client is available.
              if (client === undefined) {
                alertsRef.current?.pushAlert('Failed to communicate with ROS action server.');
                return;
              }

              // Send the goal.
              currentGoal = {
                client: client,
                id: client.sendGoal(
                  goal,
                  (_) => {
                    if (noFeedbackCheckTimeout) {
                      clearTimeout(noFeedbackCheckTimeout);
                    }
                    alertsRef.current?.pushAlert('Mission finished successfully.', 'success');
                    currentGoal = null;
                    rerenderAllSupervisorPanels();
                  },
                  (feedback: typeof lastFeedback.feedback) => {
                    lastFeedback = {
                      missionType: missionType,
                      feedback: feedback
                    };
                    rerenderAllSupervisorPanels();
                  }
                )
              };

              // Clear current feedback.
              lastFeedback = null;

              // Start a timer to check if feedback was received after a short while.
              // If not, show an error message and cancel the goal.
              noFeedbackCheckTimeout = setTimeout(() => {
                clearTimeout(noFeedbackCheckTimeout);
                if (currentGoal !== null && lastFeedback === null) {
                  alertsRef.current?.pushAlert(
                    'No mission feedback was received after a while. Mission will be canceled. Please ensure that the stack is running and check for any ROS errors in the terminal.'
                  );
                  currentGoal.client.cancelGoal(currentGoal.id);
                  currentGoal = null;
                  rerenderAllSupervisorPanels();
                }
              }, NO_FEEDBACK_TIMEOUT);
              rerenderAllSupervisorPanels();
            }}
          >
            {currentGoal === null ? <>&nbsp;Start Mission&nbsp;</> : <>&nbsp;Cancel Mission&nbsp;</>}
          </Button>
        </div>

        <div
          className={
            styles['supervisor-feedback'] + (currentGoal === null ? ` ${styles['supervisor-feedback-old']}` : '')
          }
        >
          {(() => {
            // Add new mission types here:

            if (lastFeedback?.missionType === 0 || lastFeedback?.missionType === 1) {
              const feedback = lastFeedback.feedback as SupervisorTfGoalFeedback & SupervisorGpsGoalFeedback;

              return (
                <>
                  <div className={styles['feedback-row']}>
                    <Label color={bgColor} className={styles['feedback-key']}>
                      State
                    </Label>
                    <div className={styles['feedback-value']}>
                      <Label color={darkBg} className={styles['supervisor-row-item']}>
                        {formatSnakeCaseState(feedback.state)}
                      </Label>
                    </div>
                  </div>
                </>
              );
            }

            if (lastFeedback?.missionType === 2 || lastFeedback?.missionType === 3) {
              const feedback = lastFeedback.feedback as SupervisorGpsYoloSearchFeedback &
                SupervisorGpsArUcoSearchFeedback;

              const poiFound = feedback.marker_found || feedback.object_found;
              const poiLatitude = feedback.object_location?.latitude || feedback.marker_location?.latitude;
              const poiLongitude = feedback.object_location?.longitude || feedback.marker_location?.longitude;

              return (
                <>
                  <div className={styles['feedback-row']}>
                    <Label color={bgColor} className={styles['feedback-key']}>
                      State
                    </Label>
                    <div className={styles['feedback-value']}>
                      <Label color={darkBg} className={styles['supervisor-row-item']}>
                        {formatSnakeCaseState(feedback.state)}
                      </Label>
                    </div>
                  </div>
                  <div className={styles['feedback-row']}>
                    <Label color={bgColor} className={styles['feedback-key']}>
                      {lastFeedback.missionType === 2 ? 'Marker' : 'Object'}
                    </Label>
                    {poiFound ? (
                      <div className={styles['feedback-value']}>
                        <Label color={darkBg} className={styles['supervisor-row-item']}>
                          {formatLatitude(poiLatitude)}
                        </Label>
                        <Label color={darkBg} className={styles['supervisor-row-item']}>
                          {formatLongitude(poiLongitude)}
                        </Label>
                        <Button
                          tooltip='Copy to clipboard.'
                          onClick={() => {
                            navigator.clipboard.writeText(`${poiLatitude}, ${poiLongitude}`);
                          }}
                        >
                          <FontAwesomeIcon icon={faCopy} />
                        </Button>
                      </div>
                    ) : (
                      <div className={styles['feedback-value']}>
                        <Label color={darkBg} className={styles['supervisor-row-item']}>
                          Not Found
                        </Label>
                      </div>
                    )}
                  </div>
                </>
              );
            }
          })()}
        </div>
      </div>
    </div>
  );
}
