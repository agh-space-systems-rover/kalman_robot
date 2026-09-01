import styles from './arm-autonomy.module.css';

import {
  addArmAutonomyListener,
  applyHomography,
  compressedImageDataUrl,
  decodeRosImage,
  detectionCenter,
  detectionLabel,
  Detection2D,
  ensureArmAutonomySubscriptions,
  getArmAutonomyDetections,
  getArmAutonomyHomography,
  getArmAutonomyImage,
  getArmAutonomyRockDetections,
  getArmAutonomyRockImage,
  getArmAutonomyRockPositions,
  ImageXY,
  PoseStamped,
  publishArmAutonomyRockTarget,
  publishArmAutonomyTarget,
  SendXY,
  stopArmAutonomyRockTarget
} from '../common/arm-autonomy';
import { alertsRef } from '../common/refs';
import { ros } from '../common/ros';
import { CSSProperties, useCallback, useEffect, useRef, useState } from 'react';

import Button from '../components/button';

const LOG_PREFIX = '[arm-autonomy panel]';

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

function validRockPose(pose: PoseStamped['pose'] | undefined): pose is PoseStamped['pose'] {
  return Boolean(
    pose &&
      Number.isFinite(pose.position.x) &&
      Number.isFinite(pose.position.y) &&
      Number.isFinite(pose.position.z) &&
      Math.hypot(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w) > 0.5
  );
}

function formatPose(pose: PoseStamped['pose'] | undefined): string {
  if (!validRockPose(pose)) {
    return 'not localized';
  }
  return `${pose.position.x.toFixed(3)}, ${pose.position.y.toFixed(3)}, ${pose.position.z.toFixed(3)}`;
}

export default function ArmAutonomyPanel() {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [imageSize, setImageSize] = useState({ width: 0, height: 0 });
  const [rockImageSize, setRockImageSize] = useState({ width: 0, height: 0 });
  const [view, setView] = useState<'panel' | 'rocks'>('panel');
  const [detections, setDetections] = useState<Detection2D[]>([]);
  const [rockDetections, setRockDetections] = useState<Detection2D[]>([]);
  const [rockPositions, setRockPositions] = useState<PoseStamped['pose'][]>([]);
  const [rockImageUrl, setRockImageUrl] = useState<string | null>(null);
  const [selectedIndex, setSelectedIndex] = useState<number | null>(null);
  const [selectedRockIndex, setSelectedRockIndex] = useState<number | null>(null);
  const [imageXY, setImageXY] = useState<ImageXY | null>(null);
  const [sendXY, setSendXY] = useState<SendXY | null>(null);
  const [hasHomography, setHasHomography] = useState(false);
  const [hasImage, setHasImage] = useState(false);
  const [rosConnected, setRosConnected] = useState(false);
  const [decodeError, setDecodeError] = useState<string | null>(null);
  const [feedPaused, setFeedPaused] = useState(false);

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
        console.warn(`${LOG_PREFIX} decode failed`, {
          encoding: image.encoding,
          width: image.width,
          height: image.height
        });
      }
    } else {
      setHasImage(false);
    }

    setDetections(getArmAutonomyDetections());
    setHasHomography(getArmAutonomyHomography() !== null);
    setRockDetections(getArmAutonomyRockDetections());
    setRockPositions(getArmAutonomyRockPositions());
    setRockImageUrl(compressedImageDataUrl(getArmAutonomyRockImage()));
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

  function selectRock(index: number) {
    if (!validRockPose(rockPositions[index])) {
      return;
    }
    setSelectedRockIndex(index);
    console.log(`${LOG_PREFIX} rock clicked`, {
      index,
      label: detectionLabel(rockDetections[index]),
      pose: rockPositions[index]
    });
  }

  function handleSend() {
    if (view === 'rocks') {
      const pose = selectedRockIndex === null ? undefined : rockPositions[selectedRockIndex];
      if (!pose || !publishArmAutonomyRockTarget(pose)) {
        alertsRef.current?.pushAlert('Failed to send rock target — select a localized rock and check ROS.', 'error');
        return;
      }
      alertsRef.current?.pushAlert(`Started approach to rock ${selectedRockIndex! + 1}`, 'success');
      return;
    }
    if (!sendXY || !imageXY) {
      console.warn(`${LOG_PREFIX} SEND ignored — missing coordinates`);
      return;
    }

    const label =
      selectedIndex !== null && detections[selectedIndex] ? detectionLabel(detections[selectedIndex]) : undefined;

    const published = publishArmAutonomyTarget({ imageXY, sendXY, label });
    console.log(`${LOG_PREFIX} SEND clicked`, { imageXY, sendXY, label, published });

    if (published) {
      alertsRef.current?.pushAlert(
        `Sent panel target (${sendXY.x.toFixed(2)}, ${sendXY.y.toFixed(2)}) on /arm/panel/target`,
        'success'
      );
    } else {
      alertsRef.current?.pushAlert('Failed to send panel target — ROS disconnected.', 'error');
    }
  }

  const { width, height } = imageSize;
  const selectedRockPose = selectedRockIndex === null ? undefined : rockPositions[selectedRockIndex];
  const canSendRock = validRockPose(selectedRockPose);

  return (
    <div className={styles['arm-autonomy']}>
      <div className={styles['viewport']}>
        <div
          className={styles['image-frame']}
          style={
            view === 'panel' && hasImage && width > 0 && height > 0
              ? { aspectRatio: `${width} / ${height}` }
              : view === 'rocks' && rockImageUrl && rockImageSize.width > 0 && rockImageSize.height > 0
                ? {
                    aspectRatio: `${rockImageSize.width} / ${rockImageSize.height}`
                  }
                : undefined
          }
        >
          <canvas
            ref={canvasRef}
            className={styles['canvas']}
            style={{
              display: view === 'panel' ? 'block' : 'none',
              visibility: hasImage ? 'visible' : 'hidden'
            }}
          />
          {view === 'rocks' && rockImageUrl && (
            <img
              className={styles['canvas']}
              src={rockImageUrl}
              alt='Arm camera YOLO detections'
              onLoad={(event) =>
                setRockImageSize({
                  width: event.currentTarget.naturalWidth,
                  height: event.currentTarget.naturalHeight
                })
              }
            />
          )}
          {view === 'panel' && hasImage && width > 0 && height > 0 && (
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
                      type='button'
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
          {view === 'rocks' && rockImageUrl && rockImageSize.width > 0 && rockImageSize.height > 0 && (
            <div className={styles['overlay-layer']}>
              {rockDetections.map((det, index) => {
                const cx = det.bbox.center.position.x;
                const cy = det.bbox.center.position.y;
                const halfW = det.bbox.size_x / 2;
                const halfH = det.bbox.size_y / 2;
                const selected = selectedRockIndex === index;
                const localized = validRockPose(rockPositions[index]);
                return (
                  <div key={`${det.id ?? index}-${cx}-${cy}`} style={detectionColorStyle(det)}>
                    <div
                      className={`${styles['bbox']} ${
                        selected ? styles['bbox-selected'] : ''
                      } ${!localized ? styles['bbox-invalid'] : ''}`}
                      style={{
                        left: `${((cx - halfW) / rockImageSize.width) * 100}%`,
                        top: `${((cy - halfH) / rockImageSize.height) * 100}%`,
                        width: `${(det.bbox.size_x / rockImageSize.width) * 100}%`,
                        height: `${(det.bbox.size_y / rockImageSize.height) * 100}%`
                      }}
                    >
                      <span className={styles['bbox-label']}>
                        {detectionLabel(det)}
                        {!localized ? ' — no depth' : ''}
                      </span>
                    </div>
                    <button
                      type='button'
                      className={`${styles['hit-target']} ${selected ? styles['hit-target-selected'] : ''}`}
                      style={{
                        left: `${(cx / rockImageSize.width) * 100}%`,
                        top: `${(cy / rockImageSize.height) * 100}%`
                      }}
                      disabled={!localized}
                      onClick={() => selectRock(index)}
                      aria-label={`Select rock ${index + 1}`}
                    />
                  </div>
                );
              })}
            </div>
          )}
        </div>
        {((view === 'panel' && !hasImage) || (view === 'rocks' && !rockImageUrl)) && (
          <div className={styles['placeholder']}>
            {view === 'panel'
              ? decodeError ?? 'Waiting for /arm/panel/image_rectified…'
              : 'Waiting for /d455_arm_wheel/yolo_annotated/compressed…'}
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
        {view === 'panel' ? (
          <>
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
          </>
        ) : (
          <>
            <span className={styles['status-item']}>
              stones: <strong>{rockDetections.length}</strong>
            </span>
            <span className={styles['status-item']}>
              selected pose: <strong>{formatPose(selectedRockPose)}</strong>
            </span>
          </>
        )}
      </div>

      <div className={styles['controls']}>
        <Button className={styles['control-button']} active={view === 'panel'} onClick={() => setView('panel')}>
          PANEL
        </Button>
        <Button className={styles['control-button']} active={view === 'rocks'} onClick={() => setView('rocks')}>
          ROCKS
        </Button>
        <Button
          className={styles['control-button']}
          active={feedPaused}
          onClick={() => setFeedPaused((paused) => !paused)}
          tooltip={feedPaused ? 'Resume video and detections' : 'Pause video and detections'}
        >
          {feedPaused ? 'Paused' : 'Live'}
        </Button>
        <Button
          className={styles['control-button']}
          disabled={view === 'rocks' ? !canSendRock : !sendXY}
          onClick={handleSend}
        >
          SEND
        </Button>
        {view === 'rocks' && (
          <Button
            className={styles['control-button']}
            onClick={() => {
              stopArmAutonomyRockTarget();
              alertsRef.current?.pushAlert('Stopped rock target stream.', 'success');
            }}
          >
            STOP
          </Button>
        )}
      </div>
    </div>
  );
}
