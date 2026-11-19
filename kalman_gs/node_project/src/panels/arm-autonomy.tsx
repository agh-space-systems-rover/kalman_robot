import styles from './arm-autonomy.module.css';

import {
  addArmAutonomyListener,
  applyHomography,
  cancelArmAutonomyGoal,
  decodeRosImage,
  detectionCenter,
  detectionLabel,
  Detection2D,
  ensureArmAutonomySubscriptions,
  getArmAutonomyDetections,
  getArmAutonomyHomography,
  getArmAutonomyImage,
  ImageXY,
  inferPanelMarkerPositions,
  sendArmAutonomyGoal,
  SendXY
} from '../common/arm-autonomy';
import { alertsRef } from '../common/refs';
import { ros } from '../common/ros';
import { CSSProperties, useCallback, useEffect, useRef, useState } from 'react';

import Button from '../components/button';

const LOG_PREFIX = '[arm-autonomy panel]';
type MissionKind = 'approach' | 'interact' | 'compact_herman';

function formatXY(xy: ImageXY | SendXY | null, decimalPlaces = 1): string {
  if (!xy) {
    return '—';
  }
  return `${xy.x.toFixed(decimalPlaces)}, ${xy.y.toFixed(decimalPlaces)}`;
}

function detectionColorStyle(det: Detection2D): CSSProperties {
  const classId = det.results[0]?.hypothesis.class_id ?? 'detection';
  let hash = 2166136261;
  for (let index = 0; index < classId.length; index++) {
    hash ^= classId.charCodeAt(index);
    hash = Math.imul(hash, 16777619) >>> 0;
  }

  const hue = (hash % 180) * 2;
  return {
    '--detection-color': `hsl(${hue} 85% 60%)`,
    '--detection-fill': `hsl(${hue} 85% 60% / 0.2)`,
    '--detection-hover-fill': `hsl(${hue} 85% 60% / 0.4)`
  } as CSSProperties;
}

export default function ArmAutonomyPanel() {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [imageSize, setImageSize] = useState({ width: 0, height: 0 });
  const [detections, setDetections] = useState<Detection2D[]>([]);
  const [selectedIndex, setSelectedIndex] = useState<number | null>(null);
  const [imageXY, setImageXY] = useState<ImageXY | null>(null);
  const [sendXY, setSendXY] = useState<SendXY | null>(null);
  const [hasHomography, setHasHomography] = useState(false);
  const [hasImage, setHasImage] = useState(false);
  const [rosConnected, setRosConnected] = useState(false);
  const [decodeError, setDecodeError] = useState<string | null>(null);
  const [feedPaused, setFeedPaused] = useState(false);
  const [activeMission, setActiveMission] = useState<MissionKind | null>(null);
  const [calibrationActive, setCalibrationActive] = useState(false);
  const [calibrationFeedback, setCalibrationFeedback] = useState('Ready');
  const calibrationGenerationRef = useRef(0);
  const [missionFeedback, setMissionFeedback] = useState<Record<MissionKind, string>>({
    approach: 'Ready',
    interact: 'Select a detection',
    compact_herman: 'Ready'
  });
  const activeGoalRef = useRef<{ kind: MissionKind; id: string } | null>(null);
  const missionGenerationRef = useRef<Record<MissionKind, number>>({
    approach: 0,
    interact: 0,
    compact_herman: 0
  });

  const drawImageData = useCallback((imageData: ImageData) => {
    const canvas = canvasRef.current;
    if (!canvas) {
      console.warn(`${LOG_PREFIX} canvas ref missing during draw`);
      return false;
    }
    canvas.width = imageData.width;
    canvas.height = imageData.height;
    const ctx = canvas.getContext('2d');
    if (!ctx) {
      console.warn(`${LOG_PREFIX} canvas 2d context missing`);
      return false;
    }
    ctx.putImageData(imageData, 0, 0);
    return true;
  }, []);

  const refreshFromRos = useCallback(() => {
    if (feedPaused) {
      return;
    }

    const image = getArmAutonomyImage();
    if (image) {
      const imageData = decodeRosImage(image);
      if (imageData) {
        const drawn = drawImageData(imageData);
        if (drawn) {
          setImageSize({ width: imageData.width, height: imageData.height });
          setHasImage(true);
          setDecodeError(null);
        }
      } else {
        setHasImage(false);
        setDecodeError(`unsupported encoding: ${image.encoding}`);
        console.warn(`${LOG_PREFIX} decode failed`, { encoding: image.encoding, width: image.width, height: image.height });
      }
    } else {
      setHasImage(false);
    }

    setDetections(getArmAutonomyDetections());
    setHasHomography(getArmAutonomyHomography() !== null);
  }, [drawImageData, feedPaused]);

  useEffect(() => {
    console.log(`${LOG_PREFIX} mounted`);
    ensureArmAutonomySubscriptions();
    return () => console.log(`${LOG_PREFIX} unmounted`);
  }, []);

  useEffect(() => {
    const interval = setInterval(() => {
      const connected = ros.isConnected;
      setRosConnected(connected);
    }, 500);
    return () => clearInterval(interval);
  }, []);

  useEffect(() => {
    refreshFromRos();
    return addArmAutonomyListener(refreshFromRos);
  }, [refreshFromRos]);

  function selectDetection(index: number) {
    const det = detections[index];
    if (!det) {
      return;
    }
    const center = detectionCenter(det);
    setSelectedIndex(index);
    setImageXY(center);

    const homography = getArmAutonomyHomography();
    const send = homography ? applyHomography(homography, center.x, center.y) : null;
    setSendXY(send);

    console.log(`${LOG_PREFIX} detection clicked`, {
      index,
      label: detectionLabel(det),
      imageXY: center,
      sendXY: send,
      hasHomography: homography !== null
    });
  }

  function selectedClassId(): string | null {
    if (selectedIndex === null) {
      return null;
    }
    return detections[selectedIndex]?.results[0]?.hypothesis.class_id?.trim() || null;
  }

  function setFeedback(kind: MissionKind, feedback: string) {
    setMissionFeedback((current) => ({ ...current, [kind]: feedback }));
  }

  function startMission(kind: MissionKind) {
    if (activeGoalRef.current) {
      return;
    }

    const targetXY = kind === 'compact_herman' ? sendXY ?? { x: 0, y: 0 } : sendXY;
    if (!targetXY) {
      return;
    }

    const behaviorTree =
      kind === 'interact' ? selectedClassId() : kind === 'approach' ? 'approach' : 'compact_herman';
    if (!behaviorTree) {
      setFeedback(kind, 'Select a detection first');
      return;
    }

    const generation = ++missionGenerationRef.current[kind];
    setFeedback(kind, `Sending ${behaviorTree}…`);
    const goalId = sendArmAutonomyGoal(targetXY, behaviorTree, {
      onFeedback: (feedback) => {
        if (missionGenerationRef.current[kind] === generation) {
          setFeedback(kind, feedback.progress || 'Running…');
        }
      },
      onResult: (result) => {
        if (missionGenerationRef.current[kind] !== generation) {
          return;
        }
        const outcome = result.result ? 'Succeeded' : 'Failed';
        setFeedback(kind, result.message ? `${outcome}: ${result.message}` : outcome);
        if (activeGoalRef.current?.kind === kind) {
          activeGoalRef.current = null;
          setActiveMission(null);
        }
        alertsRef.current?.pushAlert(
          result.message || `${behaviorTree} mission ${result.result ? 'succeeded' : 'failed'}.`,
          result.result ? 'success' : 'error'
        );
      },
      onFailed: (error) => {
        if (missionGenerationRef.current[kind] !== generation) {
          return;
        }
        setFeedback(kind, `Action error: ${error}`);
        if (activeGoalRef.current?.kind === kind) {
          activeGoalRef.current = null;
          setActiveMission(null);
        }
        alertsRef.current?.pushAlert(`Failed to run ${behaviorTree}: ${error}`, 'error');
      }
    });

    if (!goalId) {
      setFeedback(kind, 'Failed to send goal');
      return;
    }
    activeGoalRef.current = { kind, id: goalId };
    setActiveMission(kind);
    console.log(`${LOG_PREFIX} action goal sent`, { kind, behaviorTree, targetXY, goalId });
  }

  function inferArucoPositions() {
    if (calibrationActive || activeGoalRef.current) {
      return;
    }

    const generation = ++calibrationGenerationRef.current;
    setCalibrationActive(true);
    setCalibrationFeedback('Starting inference…');
    const goalId = inferPanelMarkerPositions({
      onFeedback: (feedback) => {
        if (calibrationGenerationRef.current === generation) {
          setCalibrationFeedback(feedback.progress || `${feedback.confirmations}/5 frames`);
        }
      },
      onResult: (result) => {
        if (calibrationGenerationRef.current !== generation) {
          return;
        }
        setCalibrationActive(false);
        const assignment = result.marker_names
          .map((name, index) => `${name}=${result.marker_ids[index]}`)
          .join(', ');
        setCalibrationFeedback(
          `${result.result ? 'Succeeded' : 'Failed'}: ${result.message}${assignment ? ` (${assignment})` : ''}`
        );
        alertsRef.current?.pushAlert(
          result.message || (result.result ? 'ArUco positions inferred.' : 'ArUco inference failed.'),
          result.result ? 'success' : 'error'
        );
      },
      onFailed: (error) => {
        if (calibrationGenerationRef.current !== generation) {
          return;
        }
        setCalibrationActive(false);
        setCalibrationFeedback(`Action error: ${error}`);
        alertsRef.current?.pushAlert(`Failed to infer ArUco positions: ${error}`, 'error');
      }
    });

    if (!goalId) {
      setCalibrationActive(false);
      setCalibrationFeedback('Failed to send inference goal');
    }
  }

  function abortMission(kind: MissionKind) {
    const activeGoal = activeGoalRef.current;
    if (!activeGoal || activeGoal.kind !== kind) {
      return;
    }
    try {
      if (cancelArmAutonomyGoal(activeGoal.id)) {
        setFeedback(kind, 'Cancellation requested…');
      } else {
        setFeedback(kind, 'Failed to request cancellation');
      }
    } catch (error) {
      const message = error instanceof Error ? error.message : String(error);
      setFeedback(kind, `Cancellation error: ${message}`);
      alertsRef.current?.pushAlert(`Failed to abort mission: ${message}`, 'error');
    }
  }

  const { width, height } = imageSize;

  return (
    <div className={styles['arm-autonomy']}>
      <div className={`${styles['controls']} ${styles['utility-controls']}`}>
        <div className={styles['action-control']}>
          <div className={styles['action-buttons']}>
            <Button
              className={styles['control-button']}
              disabled={activeMission !== null || calibrationActive}
              onClick={() => startMission('compact_herman')}
            >
              SEND Compact Herman
            </Button>
            <Button
              className={styles['control-button']}
              disabled={activeMission !== 'compact_herman'}
              onClick={() => abortMission('compact_herman')}
            >
              Abort Compact Herman
            </Button>
          </div>
          <input
            className={styles['feedback-field']}
            value={missionFeedback.compact_herman}
            readOnly
            aria-label="Compact Herman action feedback"
          />
        </div>

        <div className={styles['action-control']}>
          <div className={styles['action-buttons']}>
            <Button
              className={styles['control-button']}
              active={calibrationActive}
              disabled={calibrationActive || activeMission !== null || !rosConnected}
              onClick={inferArucoPositions}
            >
              Infer aruco positions
            </Button>
          </div>
          <input
            className={styles['feedback-field']}
            value={calibrationFeedback}
            readOnly
            aria-label="ArUco position inference feedback"
          />
        </div>
      </div>

      <div className={styles['viewport']}>
        <div
          className={styles['image-frame']}
          style={hasImage && width > 0 && height > 0 ? { aspectRatio: `${width} / ${height}` } : undefined}
        >
          <canvas
            ref={canvasRef}
            className={styles['canvas']}
            style={{ visibility: hasImage ? 'visible' : 'hidden' }}
          />
          {hasImage && width > 0 && height > 0 && (
            <div className={styles['overlay-layer']}>
              {detections.map((det, index) => {
                const cx = det.bbox.center.position.x;
                const cy = det.bbox.center.position.y;
                const halfW = det.bbox.size_x / 2;
                const halfH = det.bbox.size_y / 2;
                const selected = selectedIndex === index;
                return (
                  <div key={`${det.id ?? index}-${cx}-${cy}`} style={detectionColorStyle(det)}>
                    <div
                      className={`${styles['bbox']} ${selected ? styles['bbox-selected'] : ''}`}
                      style={{
                        left: `${((cx - halfW) / width) * 100}%`,
                        top: `${((cy - halfH) / height) * 100}%`,
                        width: `${(det.bbox.size_x / width) * 100}%`,
                        height: `${(det.bbox.size_y / height) * 100}%`
                      }}
                    >
                      <span className={styles['bbox-label']}>{detectionLabel(det)}</span>
                    </div>
                    <button
                      type="button"
                      className={`${styles['hit-target']} ${selected ? styles['hit-target-selected'] : ''}`}
                      style={{
                        left: `${(cx / width) * 100}%`,
                        top: `${(cy / height) * 100}%`
                      }}
                      onClick={() => selectDetection(index)}
                      aria-label={`Select ${detectionLabel(det)}`}
                    />
                  </div>
                );
              })}
            </div>
          )}
        </div>
        {!hasImage && (
          <div className={styles['placeholder']}>
            {decodeError ?? 'Waiting for /arm/panel/image_rectified…'}
            <br />
            <small>Open browser DevTools console for [arm-autonomy] logs.</small>
          </div>
        )}
      </div>

      <div className={styles['status-strip']}>
        <span className={styles['status-item']}>
          ROS:{' '}
          <strong className={rosConnected ? styles['status-ok'] : styles['status-missing']}>
            {rosConnected ? 'connected' : 'disconnected'}
          </strong>
        </span>
        <span className={styles['status-item']}>
          Transform:{' '}
          <strong className={hasHomography ? styles['status-ok'] : styles['status-missing']}>
            {hasHomography ? 'OK' : 'missing'}
          </strong>
        </span>
        <span className={styles['status-item']}>
          image XY: <strong>{formatXY(imageXY)}</strong>
        </span>
        <span className={styles['status-item']}>
          send XY: <strong>{formatXY(sendXY, 3)}</strong>
        </span>
      </div>

      <div className={styles['controls']}>
        <Button
          className={`${styles['control-button']} ${styles['live-button']}`}
          active={feedPaused}
          onClick={() => setFeedPaused((paused) => !paused)}
          tooltip={feedPaused ? 'Resume video and detections' : 'Pause video and detections'}
        >
          {feedPaused ? 'Paused' : 'Live'}
        </Button>

        <div className={styles['action-control']}>
          <div className={styles['action-buttons']}>
            <Button
              className={styles['control-button']}
              disabled={!sendXY || activeMission !== null || calibrationActive}
              onClick={() => startMission('approach')}
            >
              SEND Approach
            </Button>
            <Button
              className={styles['control-button']}
              disabled={activeMission !== 'approach'}
              onClick={() => abortMission('approach')}
            >
              Abort Approach
            </Button>
          </div>
          <input
            className={styles['feedback-field']}
            value={missionFeedback.approach}
            readOnly
            aria-label="Approach action feedback"
          />
        </div>

        <div className={styles['action-control']}>
          <div className={styles['action-buttons']}>
            <Button
              className={styles['control-button']}
              disabled={!sendXY || !selectedClassId() || activeMission !== null || calibrationActive}
              onClick={() => startMission('interact')}
            >
              SEND Interact
            </Button>
            <Button
              className={styles['control-button']}
              disabled={activeMission !== 'interact'}
              onClick={() => abortMission('interact')}
            >
              Abort Interact
            </Button>
          </div>
          <input
            className={styles['feedback-field']}
            value={missionFeedback.interact}
            readOnly
            aria-label="Interact action feedback"
          />
        </div>

      </div>
    </div>
  );
}
