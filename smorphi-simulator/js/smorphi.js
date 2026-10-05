/**
 * Smorphi Robot Model & Kinematics Engine
 * Platform: Single-Block Smorphi Base Unit with Front Cargo Mesh Scoop
 * 4-Wheel Mecanum Holonomic Kinematics (3-DOF: Vx, Vy, Omega)
 * Dynamic Mass Distribution, Center of Mass (CoM) Shift, and Wheel Slip Emulation.
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

    // Physical dimensions & mass
    this.width = CONFIG.ROBOT.WIDTH;   // 0.170 m
    this.length = CONFIG.ROBOT.LENGTH; // 0.170 m
    this.height = CONFIG.ROBOT.HEIGHT; // 0.315 m
    this.baseMass = CONFIG.ROBOT.MASS; // 2.20 kg

    // Dynamic Cargo Payload & CoM State
    this.cargoCount = 0;
    this.cargoMass = 0.0;
    this.totalMass = this.baseMass;
    this.comOffsetX = 0.0; // Forward displacement of Center of Mass (m)
    this.rotationalInertia = (1 / 12) * this.baseMass * (this.width ** 2 + this.length ** 2);
    this.tractionMultiplier = 1.0;
    this.hasScoopContact = false;
    this.scoopProximity = 0.25;

    // Compatibility fields
    this.currentShape = "SINGLE_BLOCK";
    this.isMorphing = false;

    // 4 Mecanum wheels [0: FL, 1: FR, 2: RL, 3: RR]
    this.totalWheels = CONFIG.ROBOT.TOTAL_WHEELS || 4;
    this.wheelSpeeds = new Float32Array(this.totalWheels);
    this.wheelAngles = new Float32Array(this.totalWheels);

    // Collision & Status flags
    this.inCollision = false;
    this.collisionCount = 0;
    this.totalDistanceTraveled = 0.0;
    this.trajectory = [];
    this.lastTrailTime = 0;
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
   * Compatibility method for older scripts (morphing is now obsolete)
   */
  setShape(shape) {
    // Single block architecture does not morph
    return true;
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
    this.collisionCount = 0;
    this.totalDistanceTraveled = 0;
    this.wheelAngles = [0, 0, 0, 0];
    this.wheelSpeeds = [0, 0, 0, 0];
  }

  /**
   * Update internal cargo load, Center of Mass (CoM), and rotational inertia
   * @param {number} count - Number of captured cubes (0, 1, 2, 3)
   * @param {boolean} contact - Tactile sensor flag in scoop
   * @param {number} proximity - Distance to nearest cube in front
   */
  updateCargoState(count, contact = false, proximity = 0.25) {
    this.cargoCount = count;
    this.cargoMass = count * CONFIG.CARGO.MASS;
    this.totalMass = this.baseMass + this.cargoMass;
    this.hasScoopContact = contact;
    this.scoopProximity = proximity;

    // Forward displacement of CoM: scoop center is ~0.145m forward
    const scoopCenterDist = (CONFIG.SCOOP.MOUNT_X + CONFIG.SCOOP.LENGTH / 2) || 0.155;
    this.comOffsetX = (this.cargoMass * scoopCenterDist) / this.totalMass;

    // Parallel Axis Theorem for Rotational Inertia (I_zz)
    const baseI = (1 / 12) * this.baseMass * (this.width ** 2 + this.length ** 2);
    const cargoI = count * (
      (1 / 6) * CONFIG.CARGO.MASS * (CONFIG.CARGO.SIZE ** 2) +
      CONFIG.CARGO.MASS * ((scoopCenterDist - this.comOffsetX) ** 2)
    );
    this.rotationalInertia = baseI + this.baseMass * (this.comOffsetX ** 2) + cargoI;

    // Wheel traction multiplier under extra load (proportional slip factor)
    const slipFactor = CONFIG.CARGO.SLIP_FACTOR || 0.32;
    this.tractionMultiplier = Math.max(0.45, 1.0 - slipFactor * (this.cargoMass / this.baseMass));
  }

  /**
   * Get loaded total mass in kg
   */
  getLoadedMass() {
    return this.totalMass;
  }

  /**
   * Simulation physics step
   * @param {number} dt - Delta time in seconds
   */
  update(dt) {
    // 1. Dynamic acceleration limits based on current mass and inertia
    const massRatio = this.baseMass / this.totalMass; // Slows down when loaded
    const inertiaRatio = ((1 / 12) * this.baseMass * (this.width ** 2 + this.length ** 2)) / this.rotationalInertia;

    const maxLinDelta = CONFIG.ROBOT.LINEAR_ACCEL * massRatio * dt;
    const maxAngDelta = CONFIG.ROBOT.ANGULAR_ACCEL * inertiaRatio * dt;

    const prevVx = this.vx;
    const prevVy = this.vy;

    // Apply wheel slip to lateral strafe command (Vy) when loaded
    const effectiveTargetVy = this.targetVy * this.tractionMultiplier;

    this.vx += Math.max(-maxLinDelta, Math.min(maxLinDelta, this.targetVx - this.vx));
    this.vy += Math.max(-maxLinDelta, Math.min(maxLinDelta, effectiveTargetVy - this.vy));
    this.omega += Math.max(-maxAngDelta, Math.min(maxAngDelta, this.targetOmega - this.omega));

    // Approximate body accelerations for IMU
    this.ax = (this.vx - prevVx) / dt;
    this.ay = (this.vy - prevVy) / dt;

    // 2. Coordinate Transformation: Body velocities -> Global velocities
    // Heading: 0 rad points along +X axis, PI/2 points along +Y axis
    const cosT = Math.cos(this.theta);
    const sinT = Math.sin(this.theta);

    this.globalVx = this.vx * cosT - this.vy * sinT;
    this.globalVy = this.vx * sinT + this.vy * cosT;

    // 3. Integrate Global Pose
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

    // 4. Mecanum Kinematics (Compute 4 wheel rotational speeds)
    this.updateWheelKinematics(dt);

    // 5. Update Trajectory Breadcrumbs
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
   * Inverse kinematics for 4 Mecanum Wheels at chassis corners
   */
  updateWheelKinematics(dt) {
    const R = CONFIG.ROBOT.WHEEL_RADIUS; // 0.030m
    const wheelOffsets = CONFIG.ROBOT.WHEEL_OFFSETS;

    for (let w = 0; w < this.totalWheels; w++) {
      const off = wheelOffsets[w];
      const rx = off.lx;
      const ry = off.ly;

      // Mecanum inverse kinematics equation:
      // V_wheel = (Vx - Vy * rollerSign + (rx * rollerSign - ry) * omega) / R
      const vWheel = (this.vx - off.rollerSign * this.vy + (rx * off.rollerSign - ry) * this.omega) / R;

      this.wheelSpeeds[w] = vWheel;
      this.wheelAngles[w] += vWheel * dt;
    }
  }

  /**
   * Get Oriented Bounding Boxes (OBBs) for the active Single-Block Smorphi
   * Delegates to RobotGeometry.buildRobotOBBs
   */
  getModuleBoxes() {
    return RobotGeometry.buildRobotOBBs(this.x, this.y, this.theta);
  }

  /**
   * Get approximate circular bounding radius for fast broad-phase collision
   */
  getBoundingRadius() {
    return RobotGeometry.getBoundingRadius();
  }
}

if (typeof module !== "undefined" && module.exports) {
  module.exports = SmorphiRobot;
}
