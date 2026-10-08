/**
 * Cargo Cubes Management & Rigid-Body Dynamics
 * Handles 3 physical Cargo Cubes: dimensions (90x90x90 mm, 350g),
 * floor friction drag (static/kinetic), scoop mesh containment,
 * Center of Mass (CoM) coupling, and inertial shifting under centrifugal force.
 */

class CargoCube {
  constructor(id, name, hex, css, label, x, y) {
    this.id = id;
    this.name = name;
    this.hex = hex;
    this.css = css;
    this.label = label;

    // Position and orientation
    this.x = x;
    this.y = y;
    this.theta = 0;

    // Spawn origin (checkpoint pad location stays here after the box is dragged)
    this.spawnX = x;
    this.spawnY = y;

    // Velocities
    this.vx = 0.0;
    this.vy = 0.0;
    this.omega = 0.0;

    // Physical properties
    this.size = CONFIG.CARGO.SIZE; // 0.090 m
    this.halfSize = this.size / 2; // 0.045 m
    this.mass = CONFIG.CARGO.MASS; // 0.35 kg

    // Interaction State: "UNTOUCHED" | "PUSHING_ON_GROUND" | "CAPTURED_INSIDE_MESH"
    this.state = "UNTOUCHED";
    this.captureSlot = -1; // 0, 1, 2
    this.slotOffsetX = 0.145; // default center of scoop along robot heading
    this.slotOffsetY = 0.0;

    // Inertial shifting inside mesh cavity due to centrifugal / tangential force
    this.shakeOffsetX = 0.0;
    this.shakeOffsetY = 0.0;
    this.shakeVelY = 0.0;
  }

  /**
   * Reset cube to specified coordinates
   */
  reset(x, y) {
    this.x = x;
    this.y = y;
    this.spawnX = x;
    this.spawnY = y;
    this.theta = 0;
    this.vx = 0;
    this.vy = 0;
    this.omega = 0;
    this.state = "UNTOUCHED";
    this.captureSlot = -1;
    this.shakeOffsetX = 0;
    this.shakeOffsetY = 0;
    this.shakeVelY = 0;
  }

  /**
   * Get Oriented Bounding Box (OBB) for SAT collision detection
   */
  getOBB() {
    const half = this.halfSize;
    const cosT = Math.cos(this.theta);
    const sinT = Math.sin(this.theta);

    const corners = [
      { x: this.x + (-half) * cosT - (-half) * sinT, y: this.y + (-half) * sinT + (-half) * cosT },
      { x: this.x + ( half) * cosT - (-half) * sinT, y: this.y + ( half) * sinT + (-half) * cosT },
      { x: this.x + ( half) * cosT - ( half) * sinT, y: this.y + ( half) * sinT + ( half) * cosT },
      { x: this.x + (-half) * cosT - ( half) * sinT, y: this.y + (-half) * sinT + ( half) * cosT },
    ];

    const axes = [
      { x: cosT, y: sinT },
      { x: -sinT, y: cosT },
    ];

    return {
      cx: this.x,
      cy: this.y,
      halfW: half,
      halfH: half,
      corners,
      axes,
    };
  }

  /**
   * Update cube state, floor friction drag, and scoop coupling
   * @param {number} dt - Delta time
   * @param {SmorphiRobot} robot
   */
  update(dt, robot) {
    if (this.state === "CAPTURED_INSIDE_MESH") {
      // Robot-coupled movement: the cube is held within the front scoop
      const cosT = Math.cos(robot.theta);
      const sinT = Math.sin(robot.theta);

      // Inertial shifting under centrifugal force and angular acceleration:
      // Centrifugal force outward along +X: Fc = m * omega^2 * r
      // Tangential force lateral along Y: Ft = m * alpha * r
      const omega = robot.omega;
      const ax = robot.ax;
      const ay = robot.ay;

      // Effective lateral spring-mass system inside scoop cavity (cavity inner width 150mm vs cube 90mm -> ±30mm play)
      const maxPlayY = 0.025; // 25 mm play
      const targetShakeY = Math.max(-maxPlayY, Math.min(maxPlayY, -ay * 0.015 - omega * 0.008));
      
      // Damped harmonic restoration
      this.shakeVelY += (targetShakeY - this.shakeOffsetY) * 120 * dt - this.shakeVelY * 18 * dt;
      this.shakeOffsetY += this.shakeVelY * dt;
      this.shakeOffsetY = Math.max(-maxPlayY, Math.min(maxPlayY, this.shakeOffsetY));

      // Inertial compression back against mesh backplate when braking
      const maxPlayX = 0.015;
      this.shakeOffsetX = Math.max(-maxPlayX, Math.min(0.005, -ax * 0.006));

      // Compute global position from robot pose + slot offset + shake
      const localX = this.slotOffsetX + this.shakeOffsetX;
      const localY = this.slotOffsetY + this.shakeOffsetY;

      this.x = robot.x + localX * cosT - localY * sinT;
      this.y = robot.y + localX * sinT + localY * cosT;
      this.theta = robot.theta;

      this.vx = robot.globalVx;
      this.vy = robot.globalVy;
      this.omega = robot.omega;
      return;
    }

    // UNTOUCHED or PUSHING_ON_GROUND: Simulating floor friction
    const speed = Math.hypot(this.vx, this.vy);
    if (speed > 1e-4) {
      const g = 9.81;
      const muK = CONFIG.CARGO.FLOOR_FRICTION_KINETIC || 0.28;
      const frictionAccel = muK * g;
      const speedDelta = frictionAccel * dt;

      if (speed <= speedDelta) {
        this.vx = 0;
        this.vy = 0;
      } else {
        const factor = (speed - speedDelta) / speed;
        this.vx *= factor;
        this.vy *= factor;
      }

      this.x += this.vx * dt;
      this.y += this.vy * dt;
    }

    // Rotational damping on floor
    if (Math.abs(this.omega) > 1e-4) {
      const rotDamping = 8.0 * dt;
      if (Math.abs(this.omega) <= rotDamping) {
        this.omega = 0;
      } else {
        this.omega -= Math.sign(this.omega) * rotDamping;
      }
      this.theta += this.omega * dt;
    }
  }
}

class CargoManager {
  constructor() {
    this.cubes = [];
    this.initCubes();
  }

  /**
   * Instantiate 3 Cargo Cubes with configured theme colors and labels
   */
  initCubes() {
    const cargoConfigs = CONFIG.CARGO.COLORS;
    this.cubes = [
      new CargoCube(1, cargoConfigs[0].name, cargoConfigs[0].hex, cargoConfigs[0].css, cargoConfigs[0].label, 1.8, 1.2),
      new CargoCube(2, cargoConfigs[1].name, cargoConfigs[1].hex, cargoConfigs[1].css, cargoConfigs[1].label, 2.5, 3.2),
      new CargoCube(3, cargoConfigs[2].name, cargoConfigs[2].hex, cargoConfigs[2].css, cargoConfigs[2].label, 3.6, 2.2),
    ];
  }

  /**
   * Spawns cubes with dispersed random sampling across arena bounds
   * guaranteeing reachability and spacing without locking to the direct path.
   * @param {ArenaMap} map
   */
  spawnCubes(map) {
    const arenaW = map ? map.width : CONFIG.ARENA.WIDTH;
    const arenaH = map ? map.height : CONFIG.ARENA.HEIGHT;
    const spawnX = (map && map.spawn) ? map.spawn.x : CONFIG.ARENA.DEFAULT_SPAWN.x;
    const spawnY = (map && map.spawn) ? map.spawn.y : CONFIG.ARENA.DEFAULT_SPAWN.y;
    const goalX = (map && map.goal) ? map.goal.x : CONFIG.ARENA.DEFAULT_GOAL.x;
    const goalY = (map && map.goal) ? map.goal.y : CONFIG.ARENA.DEFAULT_GOAL.y;
    const obstacles = (map && map.obstacles) ? map.obstacles : [];

    const placed = [];
    const minSpawnDistSq = 0.9 * 0.9;
    const minGoalDistSq = 0.8 * 0.8;
    const minInterCubeDistSq = 1.0 * 1.0;
    const minObsClearance = 0.22;
    const margin = 0.6;

    const distToBox = (px, py, rx, ry, rw, rh) => {
      const dx = Math.max(rx - px, 0, px - (rx + rw));
      const dy = Math.max(ry - py, 0, py - (ry + rh));
      return Math.hypot(dx, dy);
    };

    for (let i = 0; i < this.cubes.length; i++) {
      let chosenX = null;
      let chosenY = null;

      // ponytail: fixed 200 attempts per cube; add adaptive relaxation if density > 30%
      for (let attempt = 0; attempt < 200; attempt++) {
        const cx = margin + Math.random() * (arenaW - 2 * margin);
        const cy = margin + Math.random() * (arenaH - 2 * margin);

        if ((cx - spawnX) ** 2 + (cy - spawnY) ** 2 < minSpawnDistSq) continue;
        if ((cx - goalX) ** 2 + (cy - goalY) ** 2 < minGoalDistSq) continue;

        let hitObs = false;
        for (const obs of obstacles) {
          if (distToBox(cx, cy, obs.x, obs.y, obs.w, obs.h) <= minObsClearance) {
            hitObs = true;
            break;
          }
        }
        if (hitObs) continue;

        let tooClose = false;
        for (const p of placed) {
          if ((cx - p.x) ** 2 + (cy - p.y) ** 2 < minInterCubeDistSq) {
            tooClose = true;
            break;
          }
        }
        if (tooClose) continue;

        if (map && typeof map.isPointReachable === "function") {
          if (!map.isPointReachable(cx, cy)) continue;
        }

        chosenX = cx;
        chosenY = cy;
        break;
      }

      if (chosenX === null) {
        if (map && map.reachablePath && map.reachablePath.length > 5) {
          const frac = (i + 1) / (this.cubes.length + 1);
          const p = map.reachablePath[Math.floor(map.reachablePath.length * frac)];
          chosenX = p.x;
          chosenY = p.y;
        } else {
          const fallbacks = [
            { x: 1.8, y: 1.4 },
            { x: 2.5, y: 3.2 },
            { x: 3.8, y: 2.2 },
          ];
          chosenX = fallbacks[i % fallbacks.length].x;
          chosenY = fallbacks[i % fallbacks.length].y;
        }
      }

      this.cubes[i].reset(chosenX, chosenY);
      placed.push({ x: chosenX, y: chosenY });
    }
  }

  /**
   * Update physics of all cubes and resolve scoop interactions
   */
  update(dt, robot, map) {
    let capturedCount = 0;
    let hasScoopContact = false;
    let minProximity = 5.0;

    const cosT = Math.cos(robot.theta);
    const sinT = Math.sin(robot.theta);

    for (const cube of this.cubes) {
      // Relative vector from robot center to cube center
      const dx = cube.x - robot.x;
      const dy = cube.y - robot.y;

      // Project into robot body frame (lx = forward, ly = left)
      const lx = dx * cosT + dy * sinT;
      const ly = -dx * sinT + dy * cosT;
      const dist = Math.hypot(dx, dy);

      if (cube.state === "CAPTURED_INSIDE_MESH") {
        capturedCount++;
        hasScoopContact = true;
      } else {
        // Track proximity for sensors
        if (lx > 0 && Math.abs(ly) < 0.25 && dist < minProximity) {
          minProximity = dist;
        }

        // Scoop catchment zone:
        // Scoop extends from mount x=0.085 to x=0.225. Inner half-width is 0.075m.
        const inScoopCavity = (lx >= 0.080 && lx <= 0.215 && Math.abs(ly) <= 0.070);

        if (inScoopCavity) {
          // Capture the cube!
          cube.state = "CAPTURED_INSIDE_MESH";
          cube.captureSlot = capturedCount;
          
          // Slot positioning inside scoop (front-to-back nesting)
          const baseOffset = (CONFIG.SCOOP ? CONFIG.SCOOP.MOUNT_X : 0.085) + 0.055;
          cube.slotOffsetX = baseOffset + (capturedCount * 0.025);
          cube.slotOffsetY = (capturedCount % 2 === 0 ? 0.015 : -0.015);

          capturedCount++;
          hasScoopContact = true;
        } else if (lx >= 0.18 && lx <= 0.26 && Math.abs(ly) <= 0.11) {
          // Contact with outer front scoop prongs: Pushing on ground
          cube.state = "PUSHING_ON_GROUND";
          hasScoopContact = true;

          // Transfer forward velocity with floor drag penalty
          const pushForce = Math.max(0, robot.vx);
          cube.vx = robot.globalVx * 0.9;
          cube.vy = robot.globalVy * 0.9;
        } else {
          cube.state = "UNTOUCHED";
        }
      }

      cube.update(dt, robot);
    }

    // Update robot loaded mass, CoM offset, and traction slip
    robot.updateCargoState(capturedCount, hasScoopContact, minProximity);
  }

  /**
   * Get formatted telemetry summary for CodeEngine and TelemetryDashboard
   */
  getTelemetrySummary(robot) {
    const cosT = Math.cos(robot.theta);
    const sinT = Math.sin(robot.theta);

    const cubeDetails = this.cubes.map(cube => {
      const dx = cube.x - robot.x;
      const dy = cube.y - robot.y;
      const dist = Math.hypot(dx, dy);
      const targetAngle = Math.atan2(dy, dx);
      let relAngle = targetAngle - robot.theta;
      while (relAngle > Math.PI) relAngle -= 2 * Math.PI;
      while (relAngle < -Math.PI) relAngle += 2 * Math.PI;

      return {
        id: cube.id,
        name: cube.name,
        color: cube.name,
        hex: cube.hex,
        css: cube.css,
        label: cube.label,
        state: cube.state,
        dist: dist,
        relAngle: relAngle,
        relAngleDeg: (relAngle * 180) / Math.PI,
        x: cube.x,
        y: cube.y,
      };
    });

    const capturedCount = this.cubes.filter(c => c.state === "CAPTURED_INSIDE_MESH").length;

    return {
      count: capturedCount,
      totalMassKg: capturedCount * CONFIG.CARGO.MASS,
      loadPercent: (capturedCount / CONFIG.CARGO.COUNT) * 100,
      isFull: capturedCount >= CONFIG.CARGO.COUNT,
      hasContact: robot.hasScoopContact,
      frontClearance: robot.scoopProximity,
      comOffsetX: robot.comOffsetX,
      cubes: cubeDetails,
    };
  }
}

if (typeof module !== "undefined" && module.exports) {
  module.exports = { CargoCube, CargoManager };
}
