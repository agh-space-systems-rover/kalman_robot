export type Vector3 = {
  x: number;
  y: number;
  z: number;
};

export type Quaternion = {
  w: number;
  v: Vector3;
};

export const zeroVec: Vector3 = { x: 0, y: 0, z: 0 };
export const zeroQuat: Quaternion = { w: 0, v: zeroVec };

export function cross(lhs: Vector3, rhs: Vector3): Vector3 {
  return {
    x: lhs.y * rhs.z - lhs.z * rhs.y,
    y: lhs.z * rhs.x - lhs.x * rhs.z,
    z: lhs.x * rhs.y - lhs.y * rhs.x
  };
}

export function addVec(...vecs: Vector3[]): Vector3 {
  return vecs.reduce(
    (acc, vec) => {
      return {
        x: acc.x + vec.x,
        y: acc.y + vec.y,
        z: acc.z + vec.z
      };
    },
    { x: 0, y: 0, z: 0 }
  );
}

export function scaleVec(vec: Vector3, scale: number): Vector3 {
  return {
    x: vec.x * scale,
    y: vec.y * scale,
    z: vec.z * scale
  };
}

export function dotVec(lhs: Vector3, rhs: Vector3): number {
  return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

export function vecLength(vec: Vector3): number {
  return Math.sqrt(dotVec(vec, vec));
}

export function normalizeVec(vec: Vector3): Vector3 {
  return scaleVec(vec, 1 / vecLength(vec));
}

export function quatTimesVec(lhs: Quaternion, rhs: Vector3): Vector3 {
  const c = cross(lhs.v, rhs);
  return addVec(rhs, scaleVec(c, 2 * lhs.w), scaleVec(cross(lhs.v, c), 2));
}

export function quatTimesQuat(lhs: Quaternion, rhs: Quaternion): Quaternion {
  return {
    w:
      lhs.w * rhs.w - lhs.v.x * rhs.v.x - lhs.v.y * rhs.v.y - lhs.v.z * rhs.v.z,
    v: {
      x:
        lhs.w * rhs.v.x +
        lhs.v.x * rhs.w +
        lhs.v.y * rhs.v.z -
        lhs.v.z * rhs.v.y,
      y:
        lhs.w * rhs.v.y -
        lhs.v.x * rhs.v.z +
        lhs.v.y * rhs.w +
        lhs.v.z * rhs.v.x,
      z:
        lhs.w * rhs.v.z +
        lhs.v.x * rhs.v.y -
        lhs.v.y * rhs.v.x +
        lhs.v.z * rhs.w
    }
  };
}

export function quatFromAxisAngle(axis: Vector3, angle: number): Quaternion {
  const halfAngle = angle / 2;
  return {
    w: Math.cos(halfAngle),
    v: scaleVec(normalizeVec(axis), Math.sin(halfAngle))
  };
}

export function quatConj(q: Quaternion): Quaternion {
  return { w: q.w, v: scaleVec(q.v, -1) };
}

export function vecFromCssColor(color: string): Vector3 {
  // Check whether it is hex or rgb
  if (color[0] === '#') {
    // 3-char hex
    if (color.length === 4) {
      return {
        x: parseInt(color[1] + color[1], 16) / 255,
        y: parseInt(color[2] + color[2], 16) / 255,
        z: parseInt(color[3] + color[3], 16) / 255
      };
    }
    // 6-char hex
    return {
      x: parseInt(color.slice(1, 3), 16) / 255,
      y: parseInt(color.slice(3, 5), 16) / 255,
      z: parseInt(color.slice(5, 7), 16) / 255
    };
  }
  // rgb
  const rgb = color
    .slice(4, -1)
    .split(',')
    .map((x) => parseInt(x) / 255);
  return { x: rgb[0], y: rgb[1], z: rgb[2] };
}

export function clamp(value: number, min: number, max: number): number {
  return Math.min(Math.max(value, min), max);
}

export function lerp(a: number, b: number, t: number): number {
  return a + (b - a) * t;
}
