import styles from './arm-autonomy.module.css';

import {
  addArmAutonomyListener,
  applyHomography,
  decodeRosImage,
  detectionCenter,
  detectionLabel,
  Detection2D,
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

  const refreshFromRos = useCallback(() => {
    const image = getArmAutonomyImage();
    const canvas = canvasRef.current;
    if (image && canvas) {
      const imageData = decodeRosImage(image);
      if (imageData) {
        canvas.width = imageData.width;
        canvas.height = imageData.height;
        const ctx = canvas.getContext('2d');
        ctx?.putImageData(imageData, 0, 0);
        setImageSize({ width: imageData.width, height: imageData.height });
        setHasImage(true);
      }
    } else {
      setHasImage(false);
    }

    setDetections(getArmAutonomyDetections());
    setHasHomography(getArmAutonomyHomography() !== null);
  }, []);

  useEffect(() => {
    const interval = setInterval(() => setRosConnected(ros.isConnected), 500);
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
    if (homography) {
      setSendXY(applyHomography(homography, center.x, center.y));
    } else {
      setSendXY(null);
    }
  }

  function handleSend() {
    if (!sendXY) {
      return;
    }
    alertsRef.current?.pushAlert(
      `SEND stub: target panel (${sendXY.x.toFixed(2)}, ${sendXY.y.toFixed(2)}) — action client TODO`,
      'success'
    );
  }

  const { width, height } = imageSize;

  return (
    <div className={styles['arm-autonomy']}>
      <div className={styles['viewport']}>
        {hasImage ? (
          <div
            className={styles['image-frame']}
            style={width > 0 && height > 0 ? { aspectRatio: `${width} / ${height}` } : undefined}
          >
            <canvas ref={canvasRef} className={styles['canvas']} />
            {width > 0 && height > 0 && (
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
        ) : (
          <div className={styles['placeholder']}>Waiting for /arm/panel/image_rectified…</div>
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
