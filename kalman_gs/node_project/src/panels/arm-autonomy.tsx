import styles from './arm-autonomy.module.css';

import {
  addArmAutonomyListener,
  applyHomography,
  decodeRosImage,
  detectionCenter,
  detectionLabel,
  Detection2D,
  ensureArmAutonomySubscriptions,
  getArmAutonomyDetections,
  getArmAutonomyHomography,
  getArmAutonomyImage,
  ImageXY,
  SendXY
} from '../common/arm-autonomy';
import { alertsRef } from '../common/refs';
import { ros } from '../common/ros';
import { useCallback, useEffect, useRef, useState } from 'react';

import Button from '../components/button';

const LOG_PREFIX = '[arm-autonomy panel]';

function formatXY(xy: ImageXY | SendXY | null): string {
  if (!xy) {
    return '—';
  }
  return `${xy.x.toFixed(1)}, ${xy.y.toFixed(1)}`;
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
  }, [drawImageData]);

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

  function handleSend() {
    if (!sendXY) {
      console.warn(`${LOG_PREFIX} SEND ignored — no sendXY`);
      return;
    }
    console.log(`${LOG_PREFIX} SEND clicked`, sendXY);
    alertsRef.current?.pushAlert(
      `SEND stub: target panel (${sendXY.x.toFixed(2)}, ${sendXY.y.toFixed(2)}) — action client TODO`,
      'success'
    );
  }

  const { width, height } = imageSize;

  return (
    <div className={styles['arm-autonomy']}>
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
                  <div key={`${det.id ?? index}-${cx}-${cy}`}>
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
          send XY: <strong>{formatXY(sendXY)}</strong>
        </span>
      </div>

      <div className={styles['controls']}>
        <Button className={styles['send-button']} disabled={!sendXY} onClick={handleSend}>
          SEND
        </Button>
      </div>
    </div>
  );
}
