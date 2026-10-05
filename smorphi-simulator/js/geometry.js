/**
 * Shared Robot Geometry Helpers
 * Single source of truth for footprint calculations used by map generation and physics.
 */

const RobotGeometry = {
  /**
   * Get module offsets for a given shape from CONFIG.SHAPES
   * @param {string} shape - Shape key (I, O, L, T, Z, S, J)
   * @returns {Array<{id: number, x: number, y: number}>}
   */
  getShapeOffsets(shape) {
    return CONFIG.SHAPES[shape] || CONFIG.SHAPES["O"];
  },

  /**
   * Compute the axis-aligned bounding box of a shape's footprint.
   * Each module is MODULE_SIZE x MODULE_SIZE centered at its offset.
   * @param {string} shape
   * @returns {{minX: number, minY: number, maxX: number, maxY: number, width: number, height: number}}
   */
  getFootprintAABB(shape) {
    const offsets = this.getShapeOffsets(shape);
    const half = CONFIG.ROBOT.MODULE_SIZE / 2;
    let minX = Infinity, minY = Infinity, maxX = -Infinity, maxY = -Infinity;
    for (const m of offsets) {
      minX = Math.min(minX, m.x - half);
      minY = Math.min(minY, m.y - half);
      maxX = Math.max(maxX, m.x + half);
      maxY = Math.max(maxY, m.y + half);
    }
    return { minX, minY, maxX, maxY, width: maxX - minX, height: maxY - minY };
  },

  /**
   * Compute the bounding radius of a shape (max distance from center to any module corner).
   * This is the orientation-independent envelope radius.
   * @param {string} shape
   * @returns {number}
   */
  getBoundingRadius(shape) {
    const offsets = this.getShapeOffsets(shape);
    const half = CONFIG.ROBOT.MODULE_SIZE / 2;
    const halfDiag = Math.hypot(half, half);
    let maxR = 0;
    for (const m of offsets) {
      const r = Math.hypot(m.x, m.y) + halfDiag;
      if (r > maxR) maxR = r;
    }
    return maxR;
  },

  /**
   * Get navigation footprint for a given profile.
   * For FIXED_O: returns orientation-independent circular envelope.
   * @param {string} profile - Navigation profile name
   * @returns {{shape: string, boundingRadius: number, aabb: object, translationalClearance: number, turningClearance: number}}
   */
  getNavigationFootprint(profile) {
    const nav = CONFIG.MAP_NAVIGATION;
    const shape = nav.shape;
    const aabb = this.getFootprintAABB(shape);
    const boundingRadius = this.getBoundingRadius(shape);
    // For a square-like shape, the translational clearance = bounding radius + safety margin
    // For FIXED_O specifically, the robot is 0.32x0.32, bounding radius ~ 0.2263
    const translationalClearance = boundingRadius + nav.linearSafetyMargin;
    const turningClearance = nav.minTurningClearance;
    return { shape, boundingRadius, aabb, translationalClearance, turningClearance };
  },

  /**
   * Get the required obstacle inflation radius for configuration-space expansion.
   * This is the distance by which obstacles must be expanded so the pathfinding
   * grid represents valid robot CENTER positions.
   * @param {string} profile
   * @returns {number}
   */
  getObstacleInflation(profile) {
    const fp = this.getNavigationFootprint(profile);
    return fp.translationalClearance;
  },

  /**
   * Build OBB corners and axes for a robot module at a given global position and heading.
   * Reuses the same logic as SmorphiRobot.getModuleBoxes but standalone.
   * @param {number} cx - Global center X
   * @param {number} cy - Global center Y
   * @param {number} theta - Robot heading in radians
   * @param {{x: number, y: number}} moduleOffset - Local offset of this module
   * @returns {{cx, cy, halfW, halfH, corners: Array, axes: Array}}
   */
  buildModuleOBB(cx, cy, theta, moduleOffset) {
    const half = CONFIG.ROBOT.MODULE_SIZE / 2;
    const cosT = Math.cos(theta);
    const sinT = Math.sin(theta);
    // Global center of this module
    const mx = cx + moduleOffset.x * cosT - moduleOffset.y * sinT;
    const my = cy + moduleOffset.x * sinT + moduleOffset.y * cosT;

    const corners = [
      { x: mx + (-half) * cosT - (-half) * sinT, y: my + (-half) * sinT + (-half) * cosT },
      { x: mx + ( half) * cosT - (-half) * sinT, y: my + ( half) * sinT + (-half) * cosT },
      { x: mx + ( half) * cosT - ( half) * sinT, y: my + ( half) * sinT + ( half) * cosT },
      { x: mx + (-half) * cosT - ( half) * sinT, y: my + (-half) * sinT + ( half) * cosT },
    ];

    const axes = [
      { x: cosT, y: sinT },
      { x: -sinT, y: cosT },
    ];

    return { cx: mx, cy: my, halfW: half, halfH: half, corners, axes };
  },

  /**
   * Build all 4 module OBBs for a given shape at a position and heading.
   * @param {string} shape
   * @param {number} cx
   * @param {number} cy
   * @param {number} theta
   * @returns {Array<{cx, cy, halfW, halfH, corners, axes}>}
   */
  buildShapeOBBs(shape, cx, cy, theta) {
    const offsets = this.getShapeOffsets(shape);
    return offsets.map(m => this.buildModuleOBB(cx, cy, theta, m));
  },

  /**
   * SAT collision test between two OBBs. Same algorithm as PhysicsEngine.testOBBCollision.
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
   * Test if a robot placed at (cx, cy, theta) in a given shape collides with any obstacle or boundary.
   * @param {string} shape
   * @param {number} cx
   * @param {number} cy
   * @param {number} theta
   * @param {Array} obstacles - Array of {x, y, w, h}
   * @param {number} arenaW
   * @param {number} arenaH
   * @returns {boolean} true if collision detected
   */
  testRobotCollision(shape, cx, cy, theta, obstacles, arenaW, arenaH) {
    const moduleOBBs = this.buildShapeOBBs(shape, cx, cy, theta);

    // Test boundary
    for (const mod of moduleOBBs) {
      for (const corner of mod.corners) {
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

      for (const mod of moduleOBBs) {
        const result = this.testOBBCollision(mod, obsBox);
        if (result.intersects) return true;
      }
    }

    return false;
  }
};

if (typeof module !== "undefined" && module.exports) {
  module.exports = RobotGeometry;
}
