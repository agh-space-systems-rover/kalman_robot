import styles from './imu.module.css';

import { imuRotation } from '../common/imu';
import {
  quatFromAxisAngle,
  quatTimesQuat,
  vecFromCssColor
} from '../common/mini-math-lib';
import {
  Vector3,
  HemisphericLight,
  MeshBuilder,
  Mesh,
  Scene,
  ArcRotateCamera,
  StandardMaterial,
  Color3,
  Color4,
  PBRMaterial,
  Camera,
  DynamicTexture,
  Matrix,
  IPointerEvent,
  PickingInfo,
  PointerEventTypes,
  CubicEase,
  EasingFunction,
  Animation,
  SceneLoader,
  AbstractMesh,
  Quaternion
} from '@babylonjs/core';
import '@babylonjs/loaders/OBJ';
import { useCallback, useEffect, useRef, useState } from 'react';

import BabylonJS from '../components/babylon-js';

const ORTHO_FOV_SCALE_INV = 5;
const FOG_DENSITY = 0.001;
const ROS_IMU_LINK_YAW = 1.57;

function getClosestTargetAngle(current: number, target: number) {
  const diff = target - current;
  const diff2 = diff + Math.PI * 2;
  const diff3 = diff - Math.PI * 2;
  if (Math.abs(diff) < Math.abs(diff2) && Math.abs(diff) < Math.abs(diff3)) {
    return diff + current;
  }
  if (Math.abs(diff2) < Math.abs(diff3)) {
    return diff2 + current;
  }
  return diff3 + current;
}

function getClosestRightAngle(current: number) {
  return (Math.round((2 * current) / Math.PI) * Math.PI) / 2;
}

function parseCssColor(color: string): Color3 {
  const vec = vecFromCssColor(color);
  return new Color3(vec.x, vec.y, vec.z);
}

function grayscaleColor(color: Color3): Color3 {
  const luma = color.toLuminance();
  return new Color3(luma, luma, luma);
}

type Props = {
  props: {
    cameraPosition?: { x: number; y: number; z: number };
    cameraAlpha?: number;
    cameraBeta?: number;
    cameraRadius?: number;
    cameraMode?: 'perspective' | 'ortho';
  };
};

export default function Imu({ props }: Props) {
  // Set default values for props
  if (props.cameraPosition === undefined) {
    props.cameraPosition = { x: 0, y: 0, z: 0 };
  }
  if (props.cameraAlpha === undefined) {
    props.cameraAlpha = (-3 * Math.PI) / 4;
  }
  if (props.cameraBeta === undefined) {
    props.cameraBeta = Math.PI / 4;
  }
  if (props.cameraRadius === undefined) {
    props.cameraRadius = 2.5;
  }
  if (props.cameraMode === undefined) {
    props.cameraMode = 'perspective';
  }

  // theme handling
  const [rerenderCount, setRerenderCount] = useState(0);
  useEffect(() => {
    const changeTheme = () => {
      setRerenderCount((count) => count + 1);
    };

    window.addEventListener('theme-change', changeTheme);
    return () => {
      window.removeEventListener('theme-change', changeTheme);
    };
  }, [setRerenderCount]);
  const style = getComputedStyle(document.body);
  const BG_COLOR = parseCssColor(style.getPropertyValue('--background'));
  const RED_COLOR = new Color3(1, 0.4, 0.4);
  const GREEN_COLOR = new Color3(0.4, 0.7, 0.2);
  const BLUE_COLOR = new Color3(0.4, 0.4, 1);
  const GRID_COLOR = parseCssColor(style.getPropertyValue('--dark-active'));

  // const box = useRef<Mesh>();
  const kalman = useRef<AbstractMesh>();
  const camera = useRef<ArcRotateCamera>();
  const viewGizmoCamera = useRef<ArcRotateCamera>();
  const viewGizmoXBulb = useRef<Mesh>();
  const viewGizmoYBulb = useRef<Mesh>();
  const viewGizmoZBulb = useRef<Mesh>();
  const viewGizmoScene = useRef<Scene>();
  const viewSetAnimSrcCameraParams = useRef<{
    fov: number;
    radius: number;
    lowerRadiusLimit?: number;
    upperRadiusLimit?: number;
  } | null>(null);
  const animationEndTime = useRef(0);

  useEffect(() => {
    const onImuUpdated = () => {
      if (kalman.current !== undefined) {
        const imuToWorld = Object.assign({}, imuRotation); // imu frame -> world frame
        const roverToImu = quatFromAxisAngle(
          { x: 0, y: 0, z: 1 },
          ROS_IMU_LINK_YAW
        ); // rover frame -> imu frame
        const roverToWorld = quatTimesQuat(imuToWorld, roverToImu); // rover frame -> world frame

        kalman.current.rotationQuaternion = new Quaternion(
          -roverToWorld.v.x,
          -roverToWorld.v.z,
          -roverToWorld.v.y,
          roverToWorld.w
        );
      }
    };

    window.addEventListener('imu-update', onImuUpdated);

    return () => {
      window.removeEventListener('imu-update', onImuUpdated);
    };
  }, [kalman]);

  const babylonSmoothSet = useCallback(
    (target: any, targetProp: string, targetVal: any, duration: number) => {
      animationEndTime.current = Date.now() + duration * 1000;
      var ease = new CubicEase();
      ease.setEasingMode(EasingFunction.EASINGMODE_EASEINOUT);
      Animation.CreateAndStartAnimation(
        'at4',
        target,
        targetProp,
        100 / duration,
        100,
        target[targetProp],
        targetVal,
        0,
        ease
      );
    },
    [rerenderCount]
  );

  const onSceneReady = useCallback(
    async (scene: Scene) => {
      // background color
      scene.clearColor = new Color4(BG_COLOR.r, BG_COLOR.g, BG_COLOR.b, 1);

      // fog
      scene.fogMode = Scene.FOGMODE_EXP;
      scene.fogColor = BG_COLOR;
      scene.fogDensity = FOG_DENSITY;

      // ambient light
      const ambientLight1 = new HemisphericLight(
        'ambient-light-1',
        new Vector3(1),
        scene
      );
      ambientLight1.intensity = 0.5;
      const ambientLight2 = new HemisphericLight(
        'ambient-light-2',
        new Vector3(-1),
        scene
      );
      ambientLight2.intensity = 0.2;

      // This creates and positions a free camera (non-mesh)
      camera.current = new ArcRotateCamera(
        'camera',
        0,
        0,
        0,
        new Vector3(0, 0, -10),
        scene
      );
      camera.current.minZ = 0.1;
      camera.current.maxZ = 10000;
      camera.current.lowerRadiusLimit = 0.1;
      camera.current.upperRadiusLimit = 1000;
      camera.current.inertia = 0.8;
      camera.current.panningInertia = 0.8;
      camera.current.wheelDeltaPercentage = 0.05; // tuned to inertia
      camera.current.setTarget(Vector3.Zero());
      camera.current.attachControl(
        scene.getEngine().getRenderingCanvas(),
        true
      );
      camera.current.position = new Vector3(
        props.cameraPosition.x,
        props.cameraPosition.y,
        props.cameraPosition.z
      );
      camera.current.alpha = props.cameraAlpha;
      camera.current.beta = props.cameraBeta;
      camera.current.radius = props.cameraRadius;
      camera.current.mode =
        props.cameraMode === 'perspective'
          ? Camera.PERSPECTIVE_CAMERA
          : Camera.ORTHOGRAPHIC_CAMERA;
      if (camera.current.mode === Camera.ORTHOGRAPHIC_CAMERA) {
        const newFov = camera.current.fov / ORTHO_FOV_SCALE_INV;
        const scale = Math.tan(camera.current.fov / 2) / Math.tan(newFov / 2);
        camera.current.fov = newFov;
        camera.current.radius *= scale;
        camera.current.lowerRadiusLimit *= scale;
        camera.current.upperRadiusLimit *= scale;
      }

      const mesh = await SceneLoader.ImportMeshAsync(
        null,
        '/',
        'kalman.obj',
        scene
      );
      kalman.current = mesh.meshes[0];
      const kalmanMat = new PBRMaterial('kalman-material', scene);
      kalmanMat.albedoColor = parseCssColor(
        style.getPropertyValue('--interactive')
      );
      kalmanMat.metallic = 0;
      kalmanMat.roughness = 1;
      kalman.current.material = kalmanMat;

      // // Create world origin gizmo (axis lines and repeating flat gray grid.

      // // x-axis
      // const xAxis = MeshBuilder.CreateBox('x-axis', { size: 1 }, scene);
      // xAxis.scaling = new Vector3(100000, 0.0001, 0.0001);
      // const xAxisMaterial = new StandardMaterial('x-axis-material', scene);
      // xAxisMaterial.emissiveColor = redColor;
      // xAxisMaterial.wireframe = true;
      // xAxisMaterial.disableLighting = true;
      // xAxis.material = xAxisMaterial;

      // // y-axis
      // const yAxis = MeshBuilder.CreateBox('y-axis', { size: 1 }, scene);
      // yAxis.scaling = new Vector3(0.0001, 0.0001, 100000);
      // const yAxisMaterial = new StandardMaterial('y-axis-material', scene);
      // yAxisMaterial.emissiveColor = greenColor;
      // yAxisMaterial.wireframe = true;
      // yAxisMaterial.disableLighting = true;
      // yAxis.material = yAxisMaterial;

      // center grid
      const gridLineMaterial = new StandardMaterial(
        'grid-line-material',
        scene
      );
      gridLineMaterial.emissiveColor = GRID_COLOR;
      gridLineMaterial.wireframe = true;
      gridLineMaterial.disableLighting = true;
      const gridSize = 10;
      for (let i = -gridSize; i <= gridSize; i += 1) {
        // // Do not overdraw center axes.
        // if (i === 0) {
        //   continue;
        // }

        // y-axes
        const gridLine = MeshBuilder.CreateBox('grid-line', { size: 1 }, scene);
        gridLine.scaling = new Vector3(0.0001, 0.0001, 2 * gridSize);
        gridLine.position.x = i;
        gridLine.material = gridLineMaterial;

        // x-axes
        const gridLine2 = MeshBuilder.CreateBox(
          'grid-line',
          { size: 1 },
          scene
        );
        gridLine2.scaling = new Vector3(2 * gridSize, 0.0001, 0.0001);
        gridLine2.position.z = i;
        gridLine2.material = gridLineMaterial;
      }

      // handle mouse events
      const onPointerDown = (
        evt: IPointerEvent,
        pickInfo: PickingInfo,
        type: PointerEventTypes
      ) => {
        if (viewGizmoScene.current === undefined) {
          return;
        }
        const gizmoCanvasClientPos = viewGizmoScene.current
          .getEngine()
          .getRenderingCanvasClientRect();
        const ray = viewGizmoScene.current.createPickingRay(
          evt.clientX - gizmoCanvasClientPos!.x,
          evt.clientY - gizmoCanvasClientPos!.y,
          Matrix.Identity(),
          viewGizmoCamera.current as any
        );
        const pickInfo2 = viewGizmoScene.current.pickWithRay(ray);

        if (pickInfo2!.hit) {
          const name = pickInfo2!.pickedMesh!.name;

          if (camera.current === undefined) {
            return;
          }

          if (viewSetAnimSrcCameraParams.current === null) {
            let switchToOrtho = true;
            switch (name) {
              case 'x-text':
              case 'x-bulb':
                babylonSmoothSet(
                  camera.current,
                  'alpha',
                  getClosestTargetAngle(camera.current.alpha, 0),
                  0.5
                );
                babylonSmoothSet(
                  camera.current,
                  'beta',
                  getClosestTargetAngle(camera.current.beta, Math.PI / 2),
                  0.5
                );
                break;
              case 'x-negative-bulb':
                babylonSmoothSet(
                  camera.current,
                  'alpha',
                  getClosestTargetAngle(camera.current.alpha, Math.PI),
                  0.5
                );
                babylonSmoothSet(
                  camera.current,
                  'beta',
                  getClosestTargetAngle(camera.current.beta, Math.PI / 2),
                  0.5
                );
                break;
              case 'y-text':
              case 'y-bulb':
                babylonSmoothSet(
                  camera.current,
                  'alpha',
                  getClosestTargetAngle(camera.current.alpha, Math.PI / 2),
                  0.5
                );
                babylonSmoothSet(
                  camera.current,
                  'beta',
                  getClosestTargetAngle(camera.current.beta, Math.PI / 2),
                  0.5
                );
                break;
              case 'y-negative-bulb':
                babylonSmoothSet(
                  camera.current,
                  'alpha',
                  getClosestTargetAngle(camera.current.alpha, -Math.PI / 2),
                  0.5
                );
                babylonSmoothSet(
                  camera.current,
                  'beta',
                  getClosestTargetAngle(camera.current.beta, Math.PI / 2),
                  0.5
                );
                break;
              case 'z-text':
              case 'z-bulb':
                babylonSmoothSet(
                  camera.current,
                  'alpha',
                  getClosestTargetAngle(
                    camera.current.alpha,
                    getClosestRightAngle(camera.current.alpha)
                  ),
                  0.5
                );
                babylonSmoothSet(
                  camera.current,
                  'beta',
                  getClosestTargetAngle(camera.current.beta, 0),
                  0.5
                );
                break;
              case 'z-negative-bulb':
                babylonSmoothSet(
                  camera.current,
                  'alpha',
                  getClosestTargetAngle(
                    camera.current.alpha,
                    getClosestRightAngle(camera.current.alpha)
                  ),
                  0.5
                );
                babylonSmoothSet(
                  camera.current,
                  'beta',
                  getClosestTargetAngle(camera.current.beta, Math.PI),
                  0.5
                );
                break;
              default:
                switchToOrtho = false;
                break;
            }

            if (
              switchToOrtho &&
              camera.current!.mode === Camera.PERSPECTIVE_CAMERA
            ) {
              // Start animating fov to current fov * 0.2, modulate radius simultaneously
              viewSetAnimSrcCameraParams.current = {
                fov: camera.current!.fov,
                radius: camera.current!.radius,
                lowerRadiusLimit: camera.current!.lowerRadiusLimit,
                upperRadiusLimit: camera.current!.upperRadiusLimit
              };
              babylonSmoothSet(
                camera.current!,
                'fov',
                camera.current!.fov / ORTHO_FOV_SCALE_INV,
                0.5
              );
              camera.current!.lowerRadiusLimit = 0;
              camera.current!.upperRadiusLimit = 1000000;

              // At the end of the animation switch to ortho and stop modulating radius
              setTimeout(() => {
                camera.current!.mode = Camera.ORTHOGRAPHIC_CAMERA;
                const radiusScale =
                  Math.tan(viewSetAnimSrcCameraParams.current.fov / 2) /
                  Math.tan(camera.current.fov / 2);
                camera.current!.lowerRadiusLimit =
                  viewSetAnimSrcCameraParams.current.lowerRadiusLimit *
                  radiusScale;
                camera.current!.upperRadiusLimit =
                  viewSetAnimSrcCameraParams.current.upperRadiusLimit *
                  radiusScale;
                viewSetAnimSrcCameraParams.current = null;
              }, 500);
            }
          }
        } else if (evt.button !== 2) {
          // On manual camera rotation, switch to perspective
          if (camera.current!.mode === Camera.ORTHOGRAPHIC_CAMERA) {
            // Switch to perspective and set radius
            camera.current!.mode = Camera.PERSPECTIVE_CAMERA;

            // Start animating fov to current fov * 5, modulate radius simultaneously
            viewSetAnimSrcCameraParams.current = {
              fov: camera.current!.fov,
              radius: camera.current!.radius,
              lowerRadiusLimit: camera.current!.lowerRadiusLimit,
              upperRadiusLimit: camera.current!.upperRadiusLimit
            };
            babylonSmoothSet(
              camera.current!,
              'fov',
              camera.current!.fov * ORTHO_FOV_SCALE_INV,
              0.5
            );
            camera.current!.lowerRadiusLimit = 0;
            camera.current!.upperRadiusLimit = 1000000;

            // At the end of the animation stop modulating radius
            setTimeout(() => {
              const radiusScale =
                Math.tan(viewSetAnimSrcCameraParams.current.fov / 2) /
                Math.tan(camera.current.fov / 2);
              camera.current!.lowerRadiusLimit =
                viewSetAnimSrcCameraParams.current.lowerRadiusLimit *
                radiusScale;
              camera.current!.upperRadiusLimit =
                viewSetAnimSrcCameraParams.current.upperRadiusLimit *
                radiusScale;
              viewSetAnimSrcCameraParams.current = null;
            }, 500);
          }
        }
      };
      scene.onPointerDown = onPointerDown;
    },
    [
      kalman,
      babylonSmoothSet,
      props.cameraPosition,
      props.cameraAlpha,
      props.cameraBeta,
      props.cameraRadius,
      props.cameraMode,
      rerenderCount
    ]
  );

  const onRender = useCallback(
    (scene: Scene) => {
      // // Rotate the box once it's loaded.
      // if (box.current !== undefined) {
      //   const deltaTimeInMillis = scene.getEngine().getDeltaTime();

      //   const rpm = 10;
      //   box.current.rotation.y +=
      //     (rpm / 60) * Math.PI * 2 * (deltaTimeInMillis / 1000);
      // }

      // Update camera parameters
      if (camera.current !== undefined) {
        if (viewSetAnimSrcCameraParams.current !== null) {
          // Set radius relative to current (animated) fov
          const scale =
            Math.tan(viewSetAnimSrcCameraParams.current.fov / 2) /
            Math.tan(camera.current.fov / 2);
          camera.current.radius =
            viewSetAnimSrcCameraParams.current.radius * scale;
        }

        //
        if (camera.current.mode === Camera.ORTHOGRAPHIC_CAMERA) {
          // Set ortho parameters
          // Use radius, fov, aspect ratio to calculate ortho camera size
          const size =
            camera.current!.radius * 2 * Math.tan(camera.current.fov / 2);
          const size2 =
            size * camera.current!.getEngine().getAspectRatio(camera.current);
          camera.current.orthoLeft = -size2 / 2;
          camera.current.orthoRight = size2 / 2;
          camera.current.orthoBottom = -size / 2;
          camera.current.orthoTop = size / 2;

          // // Set pan sensitivity relative to camera radius
          // const panelHeight = camera
          //   .current!.getEngine()
          //   .getRenderingCanvasClientRect().height;
          // camera.current.panningSensibility =
          //   (panelHeight * 31) / camera.current.radius; // tuned to panningInertia
          camera.current.panningSensibility = 0;
        } else {
          // // Set pan sensitivity relative to camera radius
          // const panelHeight = camera
          //   .current!.getEngine()
          //   .getRenderingCanvasClientRect().height;
          // camera.current.panningSensibility =
          //   (panelHeight * 5.85) / camera.current.radius; // tuned to panningInertia
          camera.current.panningSensibility = 0;
        }

        // If the camera is not being currently animated, normalize angles.
        if (Date.now() > animationEndTime.current) {
          // Normalize angles
          if (camera.current.alpha > Math.PI) {
            camera.current.alpha -= Math.PI * 2;
          } else if (camera.current.alpha < -Math.PI) {
            camera.current.alpha += Math.PI * 2;
          }
          if (camera.current.beta > Math.PI) {
            camera.current.beta -= Math.PI * 2;
          } else if (camera.current.beta < -Math.PI) {
            camera.current.beta += Math.PI * 2;
          }
        }

        // Update props.
        props.cameraPosition.x = camera.current.position.x;
        props.cameraPosition.y = camera.current.position.y;
        props.cameraPosition.z = camera.current.position.z;
        props.cameraAlpha = camera.current.alpha;
        props.cameraBeta = camera.current.beta;
        if (camera.current.mode === Camera.PERSPECTIVE_CAMERA) {
          props.cameraRadius = camera.current.radius;
          props.cameraMode = 'perspective';
        } else {
          const newFov = camera.current.fov * ORTHO_FOV_SCALE_INV;
          const scale = Math.tan(camera.current.fov / 2) / Math.tan(newFov / 2);
          props.cameraRadius = camera.current.radius * scale;
          props.cameraMode = 'ortho';
        }
      }
    },
    [rerenderCount]
  );

  const onViewGizmoReady = useCallback(
    (scene: Scene) => {
      scene.clearColor = new Color4(0, 0, 0, 0);

      viewGizmoCamera.current = new ArcRotateCamera(
        'camera',
        0,
        0,
        0,
        Vector3.Zero(),
        scene
      );
      viewGizmoCamera.current.setPosition(new Vector3(0, 0, -10));
      viewGizmoCamera.current.mode = Camera.ORTHOGRAPHIC_CAMERA;
      viewGizmoCamera.current.orthoBottom = -2;
      viewGizmoCamera.current.orthoTop = 2;
      viewGizmoCamera.current.orthoLeft = -2;
      viewGizmoCamera.current.orthoRight = 2;
      viewGizmoCamera.current.inertia = 0;
      viewGizmoCamera.current.panningInertia = 0;
      viewGizmoCamera.current.lowerBetaLimit = -Infinity;
      viewGizmoCamera.current.upperBetaLimit = Infinity;

      // x-axis
      const xAxis = MeshBuilder.CreateCylinder(
        'x-axis',
        { diameter: 0.1, height: 1 },
        scene
      );
      xAxis.rotation.z = Math.PI / 2;
      xAxis.position.x = 0.5;
      const xAxisMaterial = new StandardMaterial('x-axis-material', scene);
      xAxisMaterial.emissiveColor = RED_COLOR;
      xAxisMaterial.disableLighting = true;
      xAxis.material = xAxisMaterial;

      // y-axis
      const yAxis = MeshBuilder.CreateCylinder(
        'y-axis',
        { diameter: 0.1, height: 1 },
        scene
      );
      yAxis.rotation.x = Math.PI / 2;
      yAxis.position.z = 0.5;
      const yAxisMaterial = new StandardMaterial('y-axis-material', scene);
      yAxisMaterial.emissiveColor = GREEN_COLOR;
      yAxisMaterial.disableLighting = true;
      yAxis.material = yAxisMaterial;

      // z-axis
      const zAxis = MeshBuilder.CreateCylinder(
        'z-axis',
        { diameter: 0.1, height: 1 },
        scene
      );
      zAxis.position.y = 0.5;
      const zAxisMaterial = new StandardMaterial('z-axis-material', scene);
      zAxisMaterial.emissiveColor = BLUE_COLOR;
      zAxisMaterial.disableLighting = true;
      zAxis.material = zAxisMaterial;

      // x bulb
      viewGizmoXBulb.current = MeshBuilder.CreateSphere(
        'x-bulb',
        { diameter: 0.5 },
        scene
      );
      viewGizmoXBulb.current.position.x = 1;
      const xBulbMaterial = new StandardMaterial('x-bulb-material', scene);
      xBulbMaterial.emissiveColor = RED_COLOR;
      xBulbMaterial.disableLighting = true;
      viewGizmoXBulb.current.material = xBulbMaterial;

      // x text on the bulb
      const xText = MeshBuilder.CreatePlane('x-text', { size: 0.5 }, scene);
      xText.position.z = -0.9;
      xText.parent = viewGizmoXBulb.current;
      const xTextDynamicTexture = new DynamicTexture(
        'x-text-dynamic-texture',
        { width: 64, height: 64 },
        scene
      );
      xTextDynamicTexture.hasAlpha = true;
      xTextDynamicTexture.drawText(
        'E',
        16,
        51,
        'bold 50px sans-serif',
        '#622',
        'transparent',
        true
      );
      const xTextMaterial = new StandardMaterial('x-text-material', scene);
      xTextMaterial.diffuseTexture = xTextDynamicTexture;
      xTextMaterial.disableLighting = true;
      xTextMaterial.emissiveColor = Color3.White();
      xText.material = xTextMaterial;

      // x negative bulb
      const xNegativeBulbMaterial = new StandardMaterial(
        'x-negative-bulb-material',
        scene
      );
      xNegativeBulbMaterial.emissiveColor = RED_COLOR;
      xNegativeBulbMaterial.disableLighting = true;
      xNegativeBulbMaterial.alpha = 0.2;
      const xNegativeBulb = MeshBuilder.CreateSphere(
        'x-negative-bulb',
        { diameter: 0.6 },
        scene
      );
      xNegativeBulb.position.x = -1;
      xNegativeBulb.material = xNegativeBulbMaterial;
      const xNegativeBulbInside = MeshBuilder.CreateSphere(
        'x-negative-bulb-inside',
        { diameter: 0.4 },
        scene
      );
      xNegativeBulbInside.position.x = -1;
      xNegativeBulbInside.material = xNegativeBulbMaterial;

      // y bulb
      viewGizmoYBulb.current = MeshBuilder.CreateSphere(
        'y-bulb',
        { diameter: 0.5 },
        scene
      );
      viewGizmoYBulb.current.position.z = 1;
      const yBulbMaterial = new StandardMaterial('y-bulb-material', scene);
      yBulbMaterial.emissiveColor = GREEN_COLOR;
      yBulbMaterial.disableLighting = true;
      viewGizmoYBulb.current.material = yBulbMaterial;

      // y text on the bulb
      const yText = MeshBuilder.CreatePlane('y-text', { size: 0.5 }, scene);
      yText.position.z = -0.9;
      yText.parent = viewGizmoYBulb.current;
      const yTextDynamicTexture = new DynamicTexture(
        'y-text-dynamic-texture',
        { width: 64, height: 64 },
        scene
      );
      yTextDynamicTexture.hasAlpha = true;
      yTextDynamicTexture.drawText(
        'N',
        12,
        51,
        'bold 50px sans-serif',
        '#262',
        'transparent',
        true
      );
      const yTextMaterial = new StandardMaterial('y-text-material', scene);
      yTextMaterial.diffuseTexture = yTextDynamicTexture;
      yTextMaterial.disableLighting = true;
      yTextMaterial.emissiveColor = Color3.White();
      yText.material = yTextMaterial;

      // y negative bulb
      const yNegativeBulbMaterial = new StandardMaterial(
        'y-negative-bulb-material',
        scene
      );
      yNegativeBulbMaterial.emissiveColor = GREEN_COLOR;
      yNegativeBulbMaterial.disableLighting = true;
      yNegativeBulbMaterial.alpha = 0.2;
      const yNegativeBulb = MeshBuilder.CreateSphere(
        'y-negative-bulb',
        { diameter: 0.6 },
        scene
      );
      yNegativeBulb.position.z = -1;
      yNegativeBulb.material = yNegativeBulbMaterial;
      const yNegativeBulbInside = MeshBuilder.CreateSphere(
        'y-negative-bulb-inside',
        { diameter: 0.4 },
        scene
      );
      yNegativeBulbInside.position.z = -1;
      yNegativeBulbInside.material = yNegativeBulbMaterial;

      // z bulb
      viewGizmoZBulb.current = MeshBuilder.CreateSphere(
        'z-bulb',
        { diameter: 0.5 },
        scene
      );
      viewGizmoZBulb.current.position.y = 1;
      const zBulbMaterial = new StandardMaterial('z-bulb-material', scene);
      zBulbMaterial.emissiveColor = BLUE_COLOR;
      zBulbMaterial.disableLighting = true;
      viewGizmoZBulb.current.material = zBulbMaterial;

      // z text on the bulb
      const zText = MeshBuilder.CreatePlane('z-text', { size: 0.5 }, scene);
      zText.position.z = -0.9;
      zText.parent = viewGizmoZBulb.current;
      const zTextDynamicTexture = new DynamicTexture(
        'z-text-dynamic-texture',
        { width: 64, height: 64 },
        scene
      );
      zTextDynamicTexture.hasAlpha = true;
      zTextDynamicTexture.drawText(
        'U',
        14,
        51,
        'bold 50px sans-serif',
        '#226',
        'transparent',
        true
      );
      const zTextMaterial = new StandardMaterial('y-text-material', scene);
      zTextMaterial.diffuseTexture = zTextDynamicTexture;
      zTextMaterial.disableLighting = true;
      zTextMaterial.emissiveColor = Color3.White();
      zText.material = zTextMaterial;

      // z negative bulb
      const zNegativeBulbMaterial = new StandardMaterial(
        'z-negative-bulb-material',
        scene
      );
      zNegativeBulbMaterial.emissiveColor = BLUE_COLOR;
      zNegativeBulbMaterial.disableLighting = true;
      zNegativeBulbMaterial.alpha = 0.2;
      const zNegativeBulb = MeshBuilder.CreateSphere(
        'z-negative-bulb',
        { diameter: 0.6 },
        scene
      );
      zNegativeBulb.position.y = -1;
      zNegativeBulb.material = zNegativeBulbMaterial;
      const zNegativeBulbInside = MeshBuilder.CreateSphere(
        'z-negative-bulb-inside',
        { diameter: 0.4 },
        scene
      );
      zNegativeBulbInside.position.y = -1;
      zNegativeBulbInside.material = zNegativeBulbMaterial;

      viewGizmoScene.current = scene;
    },
    [rerenderCount]
  );

  const onViewGizmoRender = useCallback(
    (scene: Scene) => {
      if (
        camera.current === undefined ||
        viewGizmoCamera.current === undefined ||
        viewGizmoXBulb.current === undefined ||
        viewGizmoYBulb.current === undefined ||
        viewGizmoZBulb.current === undefined
      ) {
        return;
      }

      viewGizmoCamera.current.alpha = camera.current.alpha;
      viewGizmoCamera.current.beta = camera.current.beta;

      const bulbRot = new Vector3(
        Math.PI / 2 - camera.current.beta,
        -camera.current.alpha - Math.PI / 2,
        0
      );
      viewGizmoXBulb.current.rotation = bulbRot;
      viewGizmoYBulb.current.rotation = bulbRot;
      viewGizmoZBulb.current.rotation = bulbRot;
    },
    [rerenderCount]
  );

  return (
    <div key={rerenderCount} className={styles['canvas-container']}>
      <BabylonJS
        className={styles['canvas']}
        id='canvas'
        antialias
        onSceneReady={onSceneReady}
        onRender={onRender}
      />
      <BabylonJS
        className={styles['view-gizmo']}
        id='view-gizmo'
        antialias
        onSceneReady={onViewGizmoReady}
        onRender={onViewGizmoRender}
      />
    </div>
  );
}
