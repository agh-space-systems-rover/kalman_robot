import { useEffect, useState } from 'react'
import styled from 'styled-components'
import * as THREE from 'three'
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls'

import type { AutonomyState } from '../../store/Autonomy/autonomyTypes'
import { useAppSelector } from '../../store/storeHooks'

interface Imu {
  alfa: number | null
  beta: number | null
  gamma: number | null
}

const Wrapper = styled.div`
  grid-area: imuvis;
  overflow: hidden;
`

export const IMUVis: () => JSX.Element = () => {
  const [imuOffset, setImuOffset] = useState<Imu>({ alfa: 0, beta: 0, gamma: 0 })
  const autonomy = useAppSelector((state) => state.autonomy)

  useEffect(() => {
    init()
  }, [])

  useEffect(() => {
    setRoverRotation(autonomy, imuOffset)
    animate()
  }, [imuOffset, autonomy])

  return (
    <Wrapper style={{ flex: 1 }}>
      <h2>IMU Visualization</h2>
      <div id='dupa'></div>
      <button onClick={(): void => setImuOffset(autonomy.imuRaw)}>Reset IMU visualization offset</button>
    </Wrapper>
  )
}

// === THREE.JS CODE START ===

let camera: THREE.PerspectiveCamera, scene: THREE.Scene, renderer: THREE.WebGLRenderer, rover: THREE.Group

// === THREE.JS EXAMPLE CODE END ===
function init(): void {
  scene = new THREE.Scene()

  renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true })
  renderer.setPixelRatio(window.devicePixelRatio)
  renderer.setSize(250, 250)
  renderer.shadowMap.enabled = true
  const dupa = document.getElementById('dupa')
  if (dupa) {
    if (dupa.lastChild) {
      dupa.removeChild(dupa.lastChild)
    }
    dupa.appendChild(renderer.domElement)
  }

  camera = new THREE.PerspectiveCamera(40, 1.0, 1, 1000)
  camera.position.set(-3, 2, 3)

  scene.add(camera)

  const controls = new OrbitControls(camera, renderer.domElement)
  controls.minDistance = 1
  controls.maxDistance = 10
  controls.maxPolarAngle = Math.PI / 2

  scene.add(new THREE.AmbientLight(0x333333))
  const light = new THREE.DirectionalLight(0xeeeeee, 0.8)
  light.position.x = 0
  light.position.y = 2
  light.position.z = 2
  light.castShadow = true
  scene.add(light)
  scene.add(new THREE.AxesHelper(2.5))

  scene.add(createPlane())
  window.addEventListener('resize', onWindowResize)

  rover = createRover()
  rover.position.y = 0.25
  scene.add(rover)
}

function createPlane(): THREE.Mesh {
  const geometry = new THREE.PlaneGeometry(5, 5)
  const material = new THREE.MeshStandardMaterial({
    color: 0xff7700,
    side: THREE.DoubleSide,
    opacity: 0.5,
    transparent: true,
  })
  const plane = new THREE.Mesh(geometry, material)
  plane.receiveShadow = true
  plane.rotation.x = Math.PI / 2
  return plane
}

function createRover(): THREE.Group {
  const rover = new THREE.Group()
  const material = new THREE.MeshStandardMaterial({ color: 0x777777 })

  const body = new THREE.Mesh(new THREE.BoxGeometry(1, 0.5, 1), material)
  body.castShadow = true
  body.receiveShadow = true
  rover.add(body)

  const arm = new THREE.Mesh(new THREE.BoxGeometry(0.2, 0.5, 0.2), material)
  arm.position.y = 0.25
  arm.position.x = 0.5
  arm.rotation.z = -Math.PI / 4
  arm.castShadow = true
  arm.receiveShadow = true
  rover.add(arm)

  const arm2 = new THREE.Mesh(new THREE.BoxGeometry(0.5, 0.2, 0.2), material)
  arm2.position.y = 0.4
  arm2.position.x = 0.8
  arm2.castShadow = true
  arm2.receiveShadow = true
  rover.add(arm2)

  const arm3 = new THREE.Mesh(new THREE.BoxGeometry(0.1, 1, 0.1), material)
  arm3.position.x = -0.3
  arm3.position.y = 0.3
  arm3.position.z = -0.3
  arm3.castShadow = true
  arm3.receiveShadow = true
  rover.add(arm3)

  rover.add(createWheel(0.5, -0.25, 0.5, material))
  rover.add(createWheel(-0.5, -0.25, 0.5, material))
  rover.add(createWheel(0.5, -0.25, -0.5, material))
  rover.add(createWheel(-0.5, -0.25, -0.5, material))

  rover.add(new THREE.AxesHelper(2.0))
  rover.rotation.x = Math.PI / 8
  rover.rotation.y = Math.PI / 8

  return rover
}

function createWheel(x: number, y: number, z: number, material: THREE.Material): THREE.Mesh {
  const cylinder = new THREE.Mesh(new THREE.CylinderGeometry(0.2, 0.2, 0.3, 10), material)
  cylinder.position.x = x
  cylinder.position.y = y
  cylinder.position.z = z
  cylinder.rotation.x = Math.PI / 2
  cylinder.castShadow = true
  cylinder.receiveShadow = true
  return cylinder
}

function onWindowResize(): void {
  camera.aspect = window.innerWidth / window.innerHeight
  camera.updateProjectionMatrix()
  renderer.setSize(window.innerWidth, window.innerHeight)
}

function setRoverRotation(autonomy: AutonomyState, imuOffset: Imu): void {
  rover.rotation.z = autonomy.imu.alfa !== null && imuOffset.alfa !== null ? autonomy.imu.alfa - imuOffset.alfa : 0
  rover.rotation.x = autonomy.imu.beta !== null && imuOffset.beta !== null ? autonomy.imu.beta - imuOffset.beta : 0
  rover.rotation.y = autonomy.imu.gamma !== null && imuOffset.gamma !== null ? autonomy.imu.gamma - imuOffset.gamma : 0
}

function animate(): void {
  // requestAnimationFrame(animate);
  render()
}

function render(): void {
  renderer.render(scene, camera)
}
