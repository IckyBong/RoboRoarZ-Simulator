/**
 * Physics Engine & Multi-Body Collision Detection
 * Implements Separating Axis Theorem (SAT) for the 4 oriented modular blocks of Smorphi,
 * continuous contact resolution, and smooth wall sliding kinematics.
 */

class PhysicsEngine {
  constructor(arenaMap) {
    this.map = arenaMap;
  }

  /**
   * Run physics simulation step with sub-stepping for tunneling prevention
   * @param {SmorphiRobot} robot
   * @param {number} dt - Full frame delta time
   */
  step(robot, dt) {
    const numSubsteps = CONFIG.SIM.MAX_SUBSTEPS;
    const subDt = dt / numSubsteps;

    robot.inCollision = false;

    for (let step = 0; step < numSubsteps; step++) {
      // 1. Integrate robot kinematics
      robot.update(subDt);

      // 2. Collision detection and response
      this.resolveCollisions(robot);
    }
  }

  /**
   * Detect and resolve collisions between Smorphi modules and arena obstacles/perimeter
   * @param {SmorphiRobot} robot
   */
  resolveCollisions(robot) {
    const moduleBoxes = robot.getModuleBoxes();
    const halfMod = CONFIG.ROBOT.MODULE_SIZE / 2;

    // 1. Boundary Wall Collisions (Enforce arena perimeter)
    const margin = halfMod;
    const minX = margin;
    const maxX = this.map.width - margin;
    const minY = margin;
    const maxY = this.map.height - margin;

    for (const mod of moduleBoxes) {
      // Check each module's 4 corners against perimeter walls
      for (const corner of mod.corners) {
        if (corner.x < 0.02) {
          robot.x += (0.02 - corner.x);
          robot.vx = Math.max(0, robot.vx);
          robot.inCollision = true;
        } else if (corner.x > this.map.width - 0.02) {
          robot.x -= (corner.x - (this.map.width - 0.02));
          robot.vx = Math.min(0, robot.vx);
          robot.inCollision = true;
        }

        if (corner.y < 0.02) {
          robot.y += (0.02 - corner.y);
          robot.vy = Math.max(0, robot.vy);
          robot.inCollision = true;
        } else if (corner.y > this.map.height - 0.02) {
          robot.y -= (corner.y - (this.map.height - 0.02));
          robot.vy = Math.min(0, robot.vy);
          robot.inCollision = true;
        }
      }
    }

    // 2. Obstacles Collisions using Separating Axis Theorem (SAT)
    for (const obs of this.map.obstacles) {
      // Broad-phase AABB rejection test
      const rBound = robot.getBoundingRadius();
      const obsCenterX = obs.x + obs.w / 2;
      const obsCenterY = obs.y + obs.h / 2;
      const obsRadius = Math.hypot(obs.w, obs.h) / 2;

      if (Math.hypot(robot.x - obsCenterX, robot.y - obsCenterY) > (rBound + obsRadius)) {
        continue; // No collision possible
      }

      // Narrow-phase SAT test for each of the 4 Smorphi modules
      const obsBox = {
        cx: obsCenterX,
        cy: obsCenterY,
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

      for (const mod of moduleBoxes) {
        const collision = this.testOBBCollision(mod, obsBox);
        if (collision.intersects) {
          robot.inCollision = true;
          robot.collisionCount++;

          // Apply Minimum Translation Vector (MTV) separation
          const pushDistance = collision.depth * 1.05;
          robot.x += collision.normal.x * pushDistance;
          robot.y += collision.normal.y * pushDistance;

          // Inelastic impulse / wall-sliding response
          // Cancel normal velocity component while maintaining tangential slide
          const dot = robot.globalVx * collision.normal.x + robot.globalVy * collision.normal.y;
          if (dot < 0) {
            robot.globalVx -= dot * collision.normal.x * 1.1;
            robot.globalVy -= dot * collision.normal.y * 1.1;

            // Re-project into robot body frame
            const cosT = Math.cos(robot.theta);
            const sinT = Math.sin(robot.theta);
            robot.vx = robot.globalVx * cosT + robot.globalVy * sinT;
            robot.vy = -robot.globalVx * sinT + robot.globalVy * cosT;
          }
        }
      }
    }
  }

  /**
   * Separating Axis Theorem (SAT) between module OBB and obstacle box.
   * Delegates to shared RobotGeometry when available, with inline fallback.
   */
  testOBBCollision(boxA, boxB) {
    if (typeof RobotGeometry !== "undefined") {
      return RobotGeometry.testOBBCollision(boxA, boxB);
    }

    // Inline fallback (same algorithm)
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
  }
}

if (typeof module !== "undefined" && module.exports) {
  module.exports = PhysicsEngine;
}
