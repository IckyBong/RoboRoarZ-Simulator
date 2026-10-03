/**
 * Smorphi Robot Model & Kinematics Engine
 * Handles morphology transformations (7 Tetromino shapes),
 * 16-wheel Mecanum holonomic kinematics, and physical state integration.
 */

class SmorphiRobot {
  constructor(x = CONFIG.ARENA.DEFAULT_SPAWN.x, y = CONFIG.ARENA.DEFAULT_SPAWN.y, theta = CONFIG.ARENA.DEFAULT_SPAWN.theta) {
    // Global Pose (in Arena Metric Coordinates: meters and radians)
    this.x = x;
    this.y = y;
    this.theta = theta; // Yaw heading in radians

    // Body velocities (in local robot frame: vx = forward, vy = lateral crab, omega = rotation)
    this.vx = 0.0;
    this.vy = 0.0;
    this.omega = 0.0;

    // Target commands from autonomous code or manual input
    this.targetVx = 0.0;
    this.targetVy = 0.0;
    this.targetOmega = 0.0;

    // Actual Global Velocities
    this.globalVx = 0.0;
    this.globalVy = 0.0;

    // Linear Accelerations (for IMU)
    this.ax = 0.0;
    this.ay = 0.0;

    // Morphology State
    this.currentShape = "O"; // Default stable compact configuration
    this.targetShape = "O";
    this.isMorphing = false;
    this.morphProgress = 1.0;
    this.morphDuration = CONFIG.ROBOT.MORPH_DURATION;

    // Active relative offsets of the 4 modules: [{id, x, y}]
    this.sourceOffsets = JSON.parse(JSON.stringify(CONFIG.SHAPES["O"]));
    this.targetOffsets = JSON.parse(JSON.stringify(CONFIG.SHAPES["O"]));
    this.currentOffsets = JSON.parse(JSON.stringify(CONFIG.SHAPES["O"]));

    // Wheel speeds for all 16 Mecanum wheels [4 modules x 4 wheels]
    this.wheelSpeeds = new Float32Array(CONFIG.ROBOT.TOTAL_WHEELS);
    this.wheelAngles = new Float32Array(CONFIG.ROBOT.TOTAL_WHEELS);

    // Collision & Status flags
    this.inCollision = false;
    this.collisionCount = 0;
    this.totalDistanceTraveled = 0.0;
    this.trajectory = []; // Breadcrumbs for trail visualization
    this.lastTrailTime = 0;

    // Hinge angles between modules (3 hinges in LLR topology)
    this.hingeAngles = [0, 0, 0];
  }

  /**
   * Request shape transformation to one of the 7 Tetromino shapes
   * @param {string} shape - "I" | "O" | "L" | "T" | "Z" | "S" | "J"
   */
  setShape(shape) {
    if (!shape) return false;
    const cleanShape = shape.toString().toUpperCase().trim();
    if (!CONFIG.SHAPES[cleanShape]) {
      console.warn(`[Smorphi] Invalid shape requested: "${shape}". Valid options: I, O, L, T, Z, S, J`);
      return false;
    }

    if (this.currentShape === cleanShape && !this.isMorphing) {
      return true; // Already in target shape
    }

    // Begin morphing interpolation
    this.sourceOffsets = JSON.parse(JSON.stringify(this.currentOffsets));
    this.targetOffsets = JSON.parse(JSON.stringify(CONFIG.SHAPES[cleanShape]));
    this.targetShape = cleanShape;
    this.isMorphing = true;
    this.morphProgress = 0.0;

    return true;
  }

  /**
   * Set holonomic velocities (local robot reference frame)
   * @param {number} vx - Forward (+) / Backward (-) speed in m/s
   * @param {number} vy - Lateral Crab Left (+) / Right (-) speed in m/s
   * @param {number} omega - Counter-Clockwise (+) / Clockwise (-) angular speed in rad/s
   */
  setVelocity(vx = 0, vy = 0, omega = 0) {
    const maxLin = CONFIG.ROBOT.MAX_LINEAR_SPEED;
    const maxAng = CONFIG.ROBOT.MAX_ANGULAR_SPEED;

    // Magnitude clamping for translation vector
    const linSpeed = Math.hypot(vx, vy);
    if (linSpeed > maxLin) {
      const scale = maxLin / linSpeed;
      vx *= scale;
      vy *= scale;
    }

    // Clamp rotation
    omega = Math.max(-maxAng, Math.min(maxAng, omega));

    this.targetVx = Number.isFinite(vx) ? vx : 0;
    this.targetVy = Number.isFinite(vy) ? vy : 0;
    this.targetOmega = Number.isFinite(omega) ? omega : 0;
  }

  /**
   * Immediate stop
   */
  stop() {
    this.targetVx = 0;
    this.targetVy = 0;
    this.targetOmega = 0;
  }

  /**
   * Reset position to specific coordinates
   */
  resetPose(x = CONFIG.ARENA.DEFAULT_SPAWN.x, y = CONFIG.ARENA.DEFAULT_SPAWN.y, theta = 0) {
    this.x = x;
    this.y = y;
    this.theta = theta;
    this.vx = 0;
    this.vy = 0;
    this.omega = 0;
    this.targetVx = 0;
    this.targetVy = 0;
    this.targetOmega = 0;
    this.globalVx = 0;
    this.globalVy = 0;
    this.ax = 0;
    this.ay = 0;
    this.trajectory = [];
    this.inCollision = false;
  }

  /**
   * Simulation physics step
   * @param {number} dt - Delta time in seconds
   */
  update(dt) {
    // 1. Update Morphology Transformation
    if (this.isMorphing) {
      this.morphProgress += dt / this.morphDuration;
      if (this.morphProgress >= 1.0) {
        this.morphProgress = 1.0;
        this.isMorphing = false;
        this.currentShape = this.targetShape;
        this.currentOffsets = JSON.parse(JSON.stringify(this.targetOffsets));
      } else {
        // Smooth S-curve easing (smootherstep)
        const t = this.morphProgress;
        const ease = t * t * t * (t * (t * 6 - 15) + 10);

        for (let i = 0; i < CONFIG.ROBOT.NUM_MODULES; i++) {
          this.currentOffsets[i].x = this.sourceOffsets[i].x + (this.targetOffsets[i].x - this.sourceOffsets[i].x) * ease;
          this.currentOffsets[i].y = this.sourceOffsets[i].y + (this.targetOffsets[i].y - this.sourceOffsets[i].y) * ease;
        }
      }
    }

    // 2. Accelerate velocities toward targets (Ramping dynamics)
    const maxLinDelta = CONFIG.ROBOT.LINEAR_ACCEL * dt;
    const maxAngDelta = CONFIG.ROBOT.ANGULAR_ACCEL * dt;

    const prevVx = this.vx;
    const prevVy = this.vy;

    this.vx += Math.max(-maxLinDelta, Math.min(maxLinDelta, this.targetVx - this.vx));
    this.vy += Math.max(-maxLinDelta, Math.min(maxLinDelta, this.targetVy - this.vy));
    this.omega += Math.max(-maxAngDelta, Math.min(maxAngDelta, this.targetOmega - this.omega));

    // Approximate body accelerations for IMU
    this.ax = (this.vx - prevVx) / dt;
    this.ay = (this.vy - prevVy) / dt;

    // 3. Coordinate Transformation: Body velocities -> Global velocities
    // Heading: 0 rad points along +X axis, PI/2 points along +Y axis
    const cosT = Math.cos(this.theta);
    const sinT = Math.sin(this.theta);

    this.globalVx = this.vx * cosT - this.vy * sinT;
    this.globalVy = this.vx * sinT + this.vy * cosT;

    // 4. Integrate Global Pose
    const prevX = this.x;
    const prevY = this.y;

    this.x += this.globalVx * dt;
    this.y += this.globalVy * dt;
    this.theta += this.omega * dt;

    // Keep theta normalized to [-PI, PI]
    while (this.theta > Math.PI) this.theta -= 2 * Math.PI;
    while (this.theta < -Math.PI) this.theta += 2 * Math.PI;

    // Odometry distance integration
    const stepDist = Math.hypot(this.x - prevX, this.y - prevY);
    this.totalDistanceTraveled += stepDist;

    // 5. Mecanum Kinematics (Compute 16 wheel rotational speeds)
    this.updateWheelKinematics(dt);

    // 6. Compute Hinge Articulation Angles
    this.updateHingeAngles();

    // 7. Update Trajectory Breadcrumbs
    const now = performance.now();
    if (now - this.lastTrailTime > 80 && (stepDist > 0.005 || Math.abs(this.omega) > 0.05)) {
      this.trajectory.push({ x: this.x, y: this.y });
      if (this.trajectory.length > 250) {
        this.trajectory.shift();
      }
      this.lastTrailTime = now;
    }
  }

  /**
   * Inverse kinematics for 16 Mecanum Wheels across the 4 modular blocks
   */
  updateWheelKinematics(dt) {
    const R = CONFIG.ROBOT.WHEEL_RADIUS;
    const s = CONFIG.ROBOT.MODULE_SIZE / 2; // half module size (0.08m)

    // Local wheel positions relative to each module center:
    // [Front-Left, Front-Right, Rear-Left, Rear-Right]
    const localWheelOffsets = [
      { lx:  s * 0.7, ly:  s * 0.9, rollerSign:  1 }, // FL (+45)
      { lx:  s * 0.7, ly: -s * 0.9, rollerSign: -1 }, // FR (-45)
      { lx: -s * 0.7, ly:  s * 0.9, rollerSign: -1 }, // RL (-45)
      { lx: -s * 0.7, ly: -s * 0.9, rollerSign:  1 }, // RR (+45)
    ];

    let wheelIdx = 0;
    for (let m = 0; m < CONFIG.ROBOT.NUM_MODULES; m++) {
      const mod = this.currentOffsets[m];

      for (let w = 0; w < 4; w++) {
        const offset = localWheelOffsets[w];
        // Total lever arm from robot centroid to this specific wheel:
        const rx = mod.x + offset.lx;
        const ry = mod.y + offset.ly;

        // Mecanum inverse kinematics equation:
        // V_wheel = (Vx - Vy * rollerSign + (rx * rollerSign - ry) * omega) / R
        const vWheel = (this.vx - offset.rollerSign * this.vy + (rx * offset.rollerSign - ry) * this.omega) / R;

        this.wheelSpeeds[wheelIdx] = vWheel;
        this.wheelAngles[wheelIdx] += vWheel * dt;
        wheelIdx++;
      }
    }
  }

  /**
   * Compute relative hinge angles between adjacent connected modules
   */
  updateHingeAngles() {
    for (let i = 0; i < 3; i++) {
      const mA = this.currentOffsets[i];
      const mB = this.currentOffsets[i + 1];
      const dx = mB.x - mA.x;
      const dy = mB.y - mA.y;
      this.hingeAngles[i] = Math.atan2(dy, dx);
    }
  }

  /**
   * Get 4 Oriented Bounding Boxes (OBBs) for the active Smorphi configuration
   * Returns array of module collision structures in global coordinates
   */
  getModuleBoxes() {
    const halfSize = CONFIG.ROBOT.MODULE_SIZE / 2;
    const cosT = Math.cos(this.theta);
    const sinT = Math.sin(this.theta);

    return this.currentOffsets.map((mod, idx) => {
      // Global center of this module
      const cx = this.x + mod.x * cosT - mod.y * sinT;
      const cy = this.y + mod.x * sinT + mod.y * cosT;

      // 4 local corners relative to module center
      const corners = [
        { lx: -halfSize, ly: -halfSize },
        { lx:  halfSize, ly: -halfSize },
        { lx:  halfSize, ly:  halfSize },
        { lx: -halfSize, ly:  halfSize },
      ].map(pt => ({
        x: cx + pt.lx * cosT - pt.ly * sinT,
        y: cy + pt.lx * sinT + pt.ly * cosT,
      }));

      // Normals/Axes for SAT collision
      const axes = [
        { x: cosT, y: sinT },
        { x: -sinT, y: cosT },
      ];

      return {
        moduleId: idx,
        cx,
        cy,
        halfSize,
        corners,
        axes,
        rotation: this.theta,
      };
    });
  }

  /**
   * Get overall approximate circular bounding radius for fast broad-phase collision
   */
  getBoundingRadius() {
    let maxDistSq = 0;
    for (const mod of this.currentOffsets) {
      const dSq = mod.x * mod.x + mod.y * mod.y;
      if (dSq > maxDistSq) maxDistSq = dSq;
    }
    // Module half-diagonal is sqrt(0.08^2 + 0.08^2) ~= 0.113m
    return Math.sqrt(maxDistSq) + 0.115;
  }
}

if (typeof module !== "undefined" && module.exports) {
  module.exports = SmorphiRobot;
}
