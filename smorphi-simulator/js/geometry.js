/**
 * Shared Robot Geometry Helpers
 * Single source of truth for footprint calculations used by map generation and physics.
 * Single-Block Smorphi Base Unit with Front Cargo Mesh Scoop.
 */

const RobotGeometry = {
  /**
   * Get the bounding radius of the robot including front scoop (from center to farthest corner)
   * Front scoop tip is at x = +0.225m, y = ±0.095m -> radius ~ 0.244m
   */
  getBoundingRadius(shape) {
    const scoopLen = CONFIG.SCOOP ? (CONFIG.SCOOP.MOUNT_X + CONFIG.SCOOP.LENGTH) : 0.225;
    const scoopHalfW = CONFIG.SCOOP ? (CONFIG.SCOOP.OUTER_WIDTH / 2) : 0.095;
    return Math.hypot(scoopLen, scoopHalfW);
  },

  /**
   * Compute the axis-aligned bounding box of the robot footprint in local frame.
   */
  getFootprintAABB(shape) {
    const halfW = CONFIG.ROBOT.WIDTH / 2; // 0.085
    const halfL = CONFIG.ROBOT.LENGTH / 2; // 0.085
    const scoopFront = CONFIG.SCOOP ? (CONFIG.SCOOP.MOUNT_X + CONFIG.SCOOP.LENGTH) : 0.225;
    const scoopHalfW = CONFIG.SCOOP ? (CONFIG.SCOOP.OUTER_WIDTH / 2) : 0.095;

    const minX = -halfL;
    const maxX = scoopFront;
    const minY = -Math.max(halfW, scoopHalfW);
    const maxY = Math.max(halfW, scoopHalfW);

    return { minX, minY, maxX, maxY, width: maxX - minX, height: maxY - minY };
  },

  /**
   * Get navigation footprint for a given profile.
   */
  getNavigationFootprint(profile) {
    const nav = CONFIG.MAP_NAVIGATION;
    const shape = nav.shape || "SINGLE_BLOCK";
    const aabb = this.getFootprintAABB(shape);
    const boundingRadius = this.getBoundingRadius(shape);
    const translationalClearance = boundingRadius + (nav.linearSafetyMargin || 0.04);
    const turningClearance = nav.minTurningClearance || 0.52;
    return { shape, boundingRadius, aabb, translationalClearance, turningClearance };
  },

  /**
   * Get the required obstacle inflation radius for configuration-space expansion.
   */
  getObstacleInflation(profile) {
    const fp = this.getNavigationFootprint(profile);
    return fp.translationalClearance;
  },

  /**
   * Helper to build an arbitrary oriented bounding box (OBB)
   */
  buildOBB(cx, cy, halfW, halfH, theta, centerOffsetX = 0, centerOffsetY = 0) {
    const cosT = Math.cos(theta);
    const sinT = Math.sin(theta);

    // Global center of this OBB
    const mx = cx + centerOffsetX * cosT - centerOffsetY * sinT;
    const my = cy + centerOffsetX * sinT + centerOffsetY * cosT;

    const corners = [
      { x: mx + (-halfW) * cosT - (-halfH) * sinT, y: my + (-halfW) * sinT + (-halfH) * cosT },
      { x: mx + ( halfW) * cosT - (-halfH) * sinT, y: my + ( halfW) * sinT + (-halfH) * cosT },
      { x: mx + ( halfW) * cosT - ( halfH) * sinT, y: my + ( halfW) * sinT + ( halfH) * cosT },
      { x: mx + (-halfW) * cosT - ( halfH) * sinT, y: my + (-halfW) * sinT + ( halfH) * cosT },
    ];

    const axes = [
      { x: cosT, y: sinT },
      { x: -sinT, y: cosT },
    ];

    return { cx: mx, cy: my, halfW, halfH, corners, axes };
  },

  /**
   * Build all OBBs for the Single-Block robot:
   * 1. Main Base Unit Chassis (170x170mm)
   * 2. Scoop Left Prong
   * 3. Scoop Right Prong
   * 4. Scoop Back/Base
   */
  buildRobotOBBs(cx, cy, theta) {
    const halfL = CONFIG.ROBOT.LENGTH / 2; // 0.085m
    const halfW = CONFIG.ROBOT.WIDTH / 2;  // 0.085m

    const scoopMountX = CONFIG.SCOOP ? CONFIG.SCOOP.MOUNT_X : 0.085;
    const scoopLen = CONFIG.SCOOP ? CONFIG.SCOOP.LENGTH : 0.140;
    const scoopOuterHalfW = CONFIG.SCOOP ? (CONFIG.SCOOP.OUTER_WIDTH / 2) : 0.095;
    const wallThick = CONFIG.SCOOP ? CONFIG.SCOOP.WALL_THICKNESS : 0.012;

    const boxes = [];

    // 1. Main Chassis Box
    boxes.push(this.buildOBB(cx, cy, halfL, halfW, theta, 0, 0));

    // 2. Scoop Left Wall Prong
    const prongLen = scoopLen;
    const prongCenterX = scoopMountX + prongLen / 2;
    const prongCenterY = scoopOuterHalfW - wallThick / 2;
    boxes.push(this.buildOBB(cx, cy, prongLen / 2, wallThick / 2, theta, prongCenterX, prongCenterY));

    // 3. Scoop Right Wall Prong
    boxes.push(this.buildOBB(cx, cy, prongLen / 2, wallThick / 2, theta, prongCenterX, -prongCenterY));

    // 4. Scoop Back/Lip Wall
    boxes.push(this.buildOBB(cx, cy, wallThick / 2, scoopOuterHalfW, theta, scoopMountX + wallThick / 2, 0));

    return boxes;
  },

  /**
   * Compatibility alias for map generator
   */
  buildShapeOBBs(shape, cx, cy, theta) {
    return this.buildRobotOBBs(cx, cy, theta);
  },

  /**
   * SAT collision test between two OBBs.
   * @param {object} boxA - {corners, axes}
   * @param {object} boxB - {corners, axes, cx, cy}
   * @returns {{intersects: boolean, depth?: number, normal?: {x, y}}}
   */
  testOBBCollision(boxA, boxB) {
    const axes = [...boxA.axes, ...boxB.axes];
    let minOverlap = Infinity;
    let bestAxis = { x: 0, y: 0 };

    for (const axis of axes) {
      const len = Math.hypot(axis.x, axis.y);
      if (len === 0) continue;
      const nx = axis.x / len;
      const ny = axis.y / len;

      let minA = Infinity, maxA = -Infinity;
      for (const p of boxA.corners) {
        const proj = p.x * nx + p.y * ny;
        if (proj < minA) minA = proj;
        if (proj > maxA) maxA = proj;
      }

      let minB = Infinity, maxB = -Infinity;
      for (const p of boxB.corners) {
        const proj = p.x * nx + p.y * ny;
        if (proj < minB) minB = proj;
        if (proj > maxB) maxB = proj;
      }

      const overlap = Math.min(maxA, maxB) - Math.max(minA, minB);
      if (overlap <= 0) return { intersects: false };

      if (overlap < minOverlap) {
        minOverlap = overlap;
        bestAxis = { x: nx, y: ny };
        const dirX = boxA.cx - boxB.cx;
        const dirY = boxA.cy - boxB.cy;
        if (dirX * bestAxis.x + dirY * bestAxis.y < 0) {
          bestAxis.x = -bestAxis.x;
          bestAxis.y = -bestAxis.y;
        }
      }
    }

    return { intersects: true, depth: minOverlap, normal: bestAxis };
  },

  /**
   * Test if a robot placed at (cx, cy, theta) collides with any obstacle or boundary.
   */
  testRobotCollision(shape, cx, cy, theta, obstacles, arenaW, arenaH) {
    const robotOBBs = this.buildRobotOBBs(cx, cy, theta);

    // Test boundary
    for (const box of robotOBBs) {
      for (const corner of box.corners) {
        if (corner.x < 0.001 || corner.x > arenaW - 0.001 ||
            corner.y < 0.001 || corner.y > arenaH - 0.001) {
          return true;
        }
      }
    }

    // Test obstacles
    for (const obs of obstacles) {
      const obsBox = {
        cx: obs.x + obs.w / 2,
        cy: obs.y + obs.h / 2,
        halfW: obs.w / 2,
        halfH: obs.h / 2,
        corners: [
          { x: obs.x, y: obs.y },
          { x: obs.x + obs.w, y: obs.y },
          { x: obs.x + obs.w, y: obs.y + obs.h },
          { x: obs.x, y: obs.y + obs.h },
        ],
        axes: [
          { x: 1, y: 0 },
          { x: 0, y: 1 },
        ],
      };

      for (const box of robotOBBs) {
        const result = this.testOBBCollision(box, obsBox);
        if (result.intersects) return true;
      }
    }

    return false;
  }
};

if (typeof module !== "undefined" && module.exports) {
  module.exports = RobotGeometry;
}
