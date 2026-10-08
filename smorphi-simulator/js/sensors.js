/**
 * Sensor Emulation Suite: 360° 2D LiDAR Raycaster, 6-DOF IMU, Odometry, and Cargo Sensors
 * Platform: Single-Block Smorphi Base Unit with Front Cargo Mesh Scoop
 * Provides realistic readouts and API helpers for autonomous navigation scripts.
 */

class SensorSuite {
  constructor(arenaMap) {
    this.map = arenaMap;

    // Preallocated arrays for 360-degree LiDAR
    this.lidarRanges = new Float32Array(CONFIG.LIDAR.NUM_BEAMS);
    this.lidarPoints = []; // 360 points {x, y, dist, angle} for rendering

    // IMU State
    this.imu = {
      heading: 0, // deg (0 - 360)
      headingRad: 0, // rad (-PI to PI)
      yaw_rate: 0, // deg/s
      yaw_rate_rad: 0, // rad/s
      ax: 0, // m/s^2
      ay: 0, // m/s^2
      roll: 0,
      pitch: 0,
    };

    // Odometry State
    this.pose = {
      x: 0,
      y: 0,
      theta: 0,
      vx: 0,
      vy: 0,
      omega: 0,
      totalDistance: 0,
      comOffsetX: 0,
      totalMass: CONFIG.ROBOT.MASS,
      cargoCount: 0,
    };

    // Target/Goal Tracker (Passive)
    this.target = {
      x: 0,
      y: 0,
      distance: 0,
      angle: 0,
      angleDeg: 0,
      reached: false,
    };

    // Cargo & Scoop Sensor State
    this.cargo = {
      count: 0,
      totalMassKg: 0,
      loadPercent: 0,
      isFull: false,
      hasContact: false,
      frontClearance: 0.25,
      comOffsetX: 0,
      cubes: [],
    };

    // Mission & Finish State
    this.mission = {
      state: "RUNNING", // "RUNNING" | "FINISHED"
      completed: false,
      time: 0,
      round: 1,
    };
  }

  /**
   * Update all sensors given current robot state and cargo state
   * @param {SmorphiRobot} robot
   * @param {CargoManager} [cargoManager]
   * @param {object} [missionInfo]
   */
  update(robot, cargoManager = null, missionInfo = null) {
    // 1. Update 360-Degree LiDAR Raycasting
    this.updateLidar(robot);

    // 2. Update 6-DOF IMU
    this.updateIMU(robot);

    // 3. Update Wheel Odometry
    this.updateOdometry(robot);

    // 4. Update Target Objective Tracker (Passive)
    this.updateTarget(robot);

    // 5. Update Cargo Sensor Readings
    if (cargoManager) {
      this.cargo = cargoManager.getTelemetrySummary(robot);
    }

    // 6. Update Mission & Finish Tracker
    if (missionInfo) {
      this.mission.state = missionInfo.state || "RUNNING";
      this.mission.completed = missionInfo.state === "FINISHED" || this.target.reached;
      this.mission.time = missionInfo.time || 0;
      this.mission.round = missionInfo.round || 1;
    } else {
      this.mission.completed = this.target.reached;
    }
  }

  /**
   * Raycast 360 beams against arena obstacles and perimeter walls
   */
  updateLidar(robot) {
    const rx = robot.x;
    const ry = robot.y;
    const rTheta = robot.theta;
    const maxR = CONFIG.LIDAR.MAX_RANGE;
    const minR = CONFIG.LIDAR.MIN_RANGE;
    const segments = this.map.segments;
    const numBeams = CONFIG.LIDAR.NUM_BEAMS;

    this.lidarPoints = [];

    for (let i = 0; i < numBeams; i++) {
      // Beam angle relative to robot heading: 0 deg = straight ahead, 90 deg = left, 270 deg = right
      const relAngleRad = (i * Math.PI) / 180;
      const globalAngle = rTheta + relAngleRad;

      const dirX = Math.cos(globalAngle);
      const dirY = Math.sin(globalAngle);

      let closestDist = maxR;
      let hitX = rx + dirX * maxR;
      let hitY = ry + dirY * maxR;

      // Raycast against all line segments in the arena
      for (let s = 0; s < segments.length; s++) {
        const seg = segments[s];
        const hit = this.raySegmentIntersection(rx, ry, dirX, dirY, seg.p1, seg.p2, closestDist);
        if (hit !== null && hit.dist < closestDist && hit.dist >= minR) {
          closestDist = hit.dist;
          hitX = hit.x;
          hitY = hit.y;
        }
      }

      // Add emulated sensor noise
      const noise = (Math.random() - 0.5) * 2 * CONFIG.LIDAR.NOISE_SIGMA;
      const finalDist = Math.max(minR, Math.min(maxR, closestDist + noise));

      this.lidarRanges[i] = finalDist;
      this.lidarPoints.push({
        angleDeg: i,
        angleRad: relAngleRad,
        distance: finalDist,
        x: hitX,
        y: hitY,
      });
    }
  }

  /**
   * Fast 2D Ray-Segment intersection test
   */
  raySegmentIntersection(rx, ry, dx, dy, p1, p2, maxDist) {
    const x1 = p1.x - rx;
    const y1 = p1.y - ry;
    const x2 = p2.x - rx;
    const y2 = p2.y - ry;

    const sx = x2 - x1;
    const sy = y2 - y1;

    const cross = dx * sy - dy * sx;
    if (Math.abs(cross) < 1e-7) return null; // Parallel

    const t = (x1 * sy - y1 * sx) / cross;
    const u = (x1 * dy - y1 * dx) / cross;

    if (t >= 0 && t <= maxDist && u >= 0 && u <= 1) {
      return {
        dist: t,
        x: rx + dx * t,
        y: ry + dy * t,
      };
    }
    return null;
  }

  /**
   * Update IMU sensor readings
   */
  updateIMU(robot) {
    let headingDeg = (robot.theta * 180) / Math.PI;
    while (headingDeg < 0) headingDeg += 360;
    while (headingDeg >= 360) headingDeg -= 360;

    this.imu.heading = headingDeg;
    this.imu.headingRad = robot.theta;
    this.imu.yaw_rate = (robot.omega * 180) / Math.PI;
    this.imu.yaw_rate_rad = robot.omega;
    this.imu.ax = robot.ax;
    this.imu.ay = robot.ay;
    this.imu.roll = 0;
    this.imu.pitch = 0;
  }

  /**
   * Update wheel odometry readings
   */
  updateOdometry(robot) {
    this.pose.x = robot.x;
    this.pose.y = robot.y;
    this.pose.theta = robot.theta;
    this.pose.vx = robot.vx;
    this.pose.vy = robot.vy;
    this.pose.omega = robot.omega;
    this.pose.totalDistance = robot.totalDistanceTraveled;
    this.pose.comOffsetX = robot.comOffsetX;
    this.pose.totalMass = robot.totalMass;
    this.pose.cargoCount = robot.cargoCount;
  }

  /**
   * Update relative target goal coordinates (Passive beacon)
   */
  updateTarget(robot) {
    const gx = this.map.goal.x;
    const gy = this.map.goal.y;
    const dx = gx - robot.x;
    const dy = gy - robot.y;

    const dist = Math.hypot(dx, dy);
    const targetHeading = Math.atan2(dy, dx);
    let relAngle = targetHeading - robot.theta;

    while (relAngle > Math.PI) relAngle -= 2 * Math.PI;
    while (relAngle < -Math.PI) relAngle += 2 * Math.PI;

    this.target.x = gx;
    this.target.y = gy;
    this.target.distance = dist;
    this.target.angle = relAngle;
    this.target.angleDeg = (relAngle * 180) / Math.PI;
    this.target.reached = dist < 0.35;
  }

  /**
   * Create the user-facing sensors input object for the Code Injection Script
   */
  getScriptInput(robot, checkpointManager = null) {
    const ranges = this.lidarRanges;

    // Helper functions on LiDAR object for clean user scripting
    const lidarHelper = {
      ranges: ranges,

      // Minimum distance in front sector [-rangeDeg, +rangeDeg]
      getFront: (rangeDeg = 30) => {
        let minDist = CONFIG.LIDAR.MAX_RANGE;
        const half = Math.min(179, Math.abs(rangeDeg));
        for (let a = 0; a <= half; a++) {
          if (ranges[a] < minDist) minDist = ranges[a];
        }
        for (let a = 360 - half; a < 360; a++) {
          if (ranges[a] < minDist) minDist = ranges[a];
        }
        return minDist;
      },

      // Minimum distance in left sector (around 90 deg)
      getLeft: (rangeDeg = 45) => {
        let minDist = CONFIG.LIDAR.MAX_RANGE;
        const minA = Math.max(0, 90 - Math.floor(rangeDeg / 2));
        const maxA = Math.min(180, 90 + Math.floor(rangeDeg / 2));
        for (let a = minA; a <= maxA; a++) {
          if (ranges[a] < minDist) minDist = ranges[a];
        }
        return minDist;
      },

      // Minimum distance in right sector (around 270 deg)
      getRight: (rangeDeg = 45) => {
        let minDist = CONFIG.LIDAR.MAX_RANGE;
        const minA = Math.max(180, 270 - Math.floor(rangeDeg / 2));
        const maxA = Math.min(359, 270 + Math.floor(rangeDeg / 2));
        for (let a = minA; a <= maxA; a++) {
          if (ranges[a] < minDist) minDist = ranges[a];
        }
        return minDist;
      },

      // Minimum distance in rear sector (around 180 deg)
      getBack: (rangeDeg = 30) => {
        let minDist = CONFIG.LIDAR.MAX_RANGE;
        const minA = Math.max(90, 180 - Math.floor(rangeDeg / 2));
        const maxA = Math.min(270, 180 + Math.floor(rangeDeg / 2));
        for (let a = minA; a <= maxA; a++) {
          if (ranges[a] < minDist) minDist = ranges[a];
        }
        return minDist;
      },

      // Arbitrary degree range minimum
      getMinInRange: (startDeg, endDeg) => {
        let minDist = CONFIG.LIDAR.MAX_RANGE;
        const s = ((Math.floor(startDeg) % 360) + 360) % 360;
        const e = ((Math.floor(endDeg) % 360) + 360) % 360;
        if (s <= e) {
          for (let a = s; a <= e; a++) {
            if (ranges[a] < minDist) minDist = ranges[a];
          }
        } else {
          for (let a = s; a < 360; a++) {
            if (ranges[a] < minDist) minDist = ranges[a];
          }
          for (let a = 0; a <= e; a++) {
            if (ranges[a] < minDist) minDist = ranges[a];
          }
        }
        return minDist;
      },
    };

    const lidarProxy = new Proxy(lidarHelper, {
      get(target, prop) {
        if (prop in target) return target[prop];
        const num = Number(prop);
        if (Number.isInteger(num)) {
          const idx = ((num % 360) + 360) % 360;
          return ranges[idx];
        }
        return undefined;
      },
    });

    const cpData = checkpointManager
      ? checkpointManager.getScriptData()
      : { list: [], navTarget: null, discoveredCount: 0, total: 0 };

    return {
      lidar: lidarProxy,
      imu: { ...this.imu },
      pose: { ...this.pose },
      cargo: { ...this.cargo },
      mission: { ...this.mission },
      shape: "SINGLE_BLOCK",
      isMorphing: false,
      target: { ...this.target },
      collision: robot.inCollision,
      checkpoints: cpData.list,
      navTarget: cpData.navTarget,
      discoveredCheckpoints: cpData.discoveredCount,
    };
  }
}

if (typeof module !== "undefined" && module.exports) {
  module.exports = SensorSuite;
}
