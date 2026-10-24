import { PanelExtensionContext } from "@foxglove/extension";
import { ReactElement, useCallback, useEffect, useLayoutEffect, useRef, useState } from "react";
import { createRoot } from "react-dom/client";

import "./camera2dof.css";

const TOPIC = "/camera_2dof/cmd";
/** ROS 2 full type name (foxglove_bridge / CDR publish). */
const SCHEMA = "kalman_interfaces/msg/Camera2Dof";
const SCHEMA_SHORT = "kalman_interfaces/Camera2Dof";
const CAMERA_ID = 0;

/** MessageDefinition for foxglove_bridge advertise (not in rosmsg-msgs-common). */
const CAMERA_2DOF_DEFINITION = {
  name: SCHEMA,
  definitions: [
    { name: "camera_id", type: "uint8" },
    { name: "yaw", type: "float32" },
    { name: "pitch", type: "float32" },
  ],
};

function advertiseOptions(): { datatypes: Map<string, typeof CAMERA_2DOF_DEFINITION> } {
  return {
    datatypes: new Map([
      [SCHEMA, CAMERA_2DOF_DEFINITION],
      [SCHEMA_SHORT, { ...CAMERA_2DOF_DEFINITION, name: SCHEMA_SHORT }],
    ]),
  };
}

type Pos = { x: number; y: number };

function toYawPitch(pos: Pos): { yaw: number; pitch: number } {
  return {
    yaw: Math.round(pos.x * 180),
    pitch: Math.round(pos.y * 180),
  };
}

function Camera2DofPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [pos, setPos] = useState<Pos>({ x: 0.5, y: 0.5 });
  const [autoSend, setAutoSend] = useState(false);
  const [rate, setRate] = useState(10);
  const [canPublish, setCanPublish] = useState(false);
  const [status, setStatus] = useState("Waiting for live connection…");
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();

  const posRef = useRef(pos);
  posRef.current = pos;

  const containerRef = useRef<HTMLDivElement>(null);
  const isDragging = useRef(false);
  const advertisedRef = useRef(false);

  const ensureAdvertised = useCallback((): boolean => {
    if (!context.advertise || !context.publish) {
      setStatus("Publish not available (need Foxglove WebSocket → :8765)");
      setCanPublish(false);
      return false;
    }
    if (advertisedRef.current) {
      return true;
    }
    try {
      // foxglove_bridge requires datatypes; rosbridge ignores them.
      context.advertise(TOPIC, SCHEMA, advertiseOptions());
      advertisedRef.current = true;
      setCanPublish(true);
      setStatus(`Ready — ${TOPIC}`);
      return true;
    } catch (err) {
      setStatus(`Advertise failed: ${String(err)}`);
      setCanPublish(false);
      return false;
    }
  }, [context]);

  const publish = useCallback(
    (next: Pos) => {
      if (!ensureAdvertised() || !context.publish) {
        return;
      }
      const { yaw, pitch } = toYawPitch(next);
      context.publish(TOPIC, { camera_id: CAMERA_ID, yaw, pitch });
      setStatus(`Published yaw=${yaw} pitch=${pitch}`);
    },
    [context, ensureAdvertised],
  );

  useLayoutEffect(() => {
    context.onRender = (_renderState, done) => {
      setRenderDone(() => done);
    };
    // Keep render loop alive so Foxglove does not treat the panel as stalled.
    context.watch("currentFrame");
    ensureAdvertised();
  }, [context, ensureAdvertised]);

  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  useEffect(() => {
    if (!autoSend) {
      return;
    }
    const id = setInterval(() => {
      publish(posRef.current);
    }, 1000 / rate);
    return () => {
      clearInterval(id);
    };
  }, [autoSend, rate, publish]);

  const yawDisplay = (pos.x * 180 - 90).toFixed(1);
  const pitchDisplay = (pos.y * 180 - 90).toFixed(1);

  function updateFromPointer(e: React.PointerEvent): void {
    const container = containerRef.current;
    if (!container) {
      return;
    }
    const rect = container.getBoundingClientRect();
    const x = Math.max(0, Math.min(1, (e.clientX - rect.left) / rect.width));
    const y = Math.max(0, Math.min(1, (e.clientY - rect.top) / rect.height));
    setPos({ x, y });
  }

  function handlePointerDown(e: React.PointerEvent): void {
    isDragging.current = true;
    containerRef.current?.setPointerCapture(e.pointerId);
    updateFromPointer(e);
  }

  function handlePointerMove(e: React.PointerEvent): void {
    if (!isDragging.current) {
      return;
    }
    updateFromPointer(e);
  }

  function handlePointerUp(): void {
    isDragging.current = false;
  }

  return (
    <div className="kalman-camera-2dof">
      <div className="kalman-camera-2dof__status" data-ok={canPublish ? "true" : "false"}>
        {status}
      </div>

      <div className="kalman-camera-2dof__joystick-wrapper">
        <div
          className="kalman-camera-2dof__joystick-area"
          ref={containerRef}
          onPointerDown={handlePointerDown}
          onPointerMove={handlePointerMove}
          onPointerUp={handlePointerUp}
        >
          <div
            className="kalman-camera-2dof__crosshair-h"
            style={{ top: `${pos.y * 100}%` }}
          />
          <div
            className="kalman-camera-2dof__crosshair-v"
            style={{ left: `${pos.x * 100}%` }}
          />
          <div
            className="kalman-camera-2dof__thumb"
            style={{ left: `${pos.x * 100}%`, top: `${pos.y * 100}%` }}
          />
        </div>
      </div>

      <div className="kalman-camera-2dof__values-row">
        <span className="kalman-camera-2dof__value-entry">
          <span className="kalman-camera-2dof__value-label">YAW</span>
          <span className="kalman-camera-2dof__value-number">{yawDisplay}°</span>
        </span>
        <span className="kalman-camera-2dof__value-entry">
          <span className="kalman-camera-2dof__value-label">PCH</span>
          <span className="kalman-camera-2dof__value-number">{pitchDisplay}°</span>
        </span>
      </div>

      <div className="kalman-camera-2dof__divider" />

      <div className="kalman-camera-2dof__controls-row">
        <label className="kalman-camera-2dof__checkbox">
          <input
            type="checkbox"
            checked={autoSend}
            onChange={(e) => {
              setAutoSend(e.target.checked);
            }}
          />
          Auto-send
        </label>
        <button
          type="button"
          className="kalman-camera-2dof__btn"
          title="Center joystick (90° / 90°)"
          onClick={() => {
            setPos({ x: 0.5, y: 0.5 });
          }}
        >
          Center
        </button>
      </div>

      {autoSend ? (
        <div className="kalman-camera-2dof__rate-row">
          <span className="kalman-camera-2dof__rate-label">HZ</span>
          <div className="kalman-camera-2dof__slider-container">
            <input
              className="kalman-camera-2dof__slider"
              type="range"
              min={1}
              max={60}
              step={1}
              value={rate}
              onChange={(e) => {
                setRate(Number(e.target.value));
              }}
            />
            <div className="kalman-camera-2dof__slider-labels">
              <span>1/s</span>
              <span className="kalman-camera-2dof__slider-value">{rate}/s</span>
              <span>60/s</span>
            </div>
          </div>
        </div>
      ) : (
        <button
          type="button"
          className="kalman-camera-2dof__send-button"
          title="Publish camera position to ROS"
          onClick={() => {
            publish(pos);
          }}
        >
          Send
        </button>
      )}
    </div>
  );
}

export function initCamera2DofPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<Camera2DofPanel context={context} />);
  return () => {
    root.unmount();
  };
}
