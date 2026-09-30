/**
 * Interactive Code Injection & Script Execution Engine
 * Compiles and executes user JavaScript navigation scripts in real-time,
 * provides sandboxed error isolation, persistent memory across ticks, and preset algorithms.
 */

class CodeEngine {
  constructor() {
    this.compiledFunction = null;
    this.memory = {}; // Preserved state between ticks
    this.isRunning = false;
    this.hasError = false;
    this.lastErrorMessage = null;
    this.logCallbacks = [];
    this.lastLogTime = 0;

    // Default Preset Scripts
    this.presets = {
      // 1. User-Requested Default Autonomous Obstacle Avoidance Template
      default_avoidance: `/**
 * ROBO-ROARZ AUTONOMOUS OBSTACLE AVOIDANCE & MORPHING
 * ----------------------------------------------------
 * Inputs:
 *   - sensors.lidar: 360-deg laser array (ranges, .getFront(), .getLeft(), .getRight())
 *   - sensors.imu:   { heading, yaw_rate }
 *   - sensors.pose:  { x, y, theta }
 *   - sensors.shape: Active shape ("O", "I", "L", "T", "Z", "S")
 *   - sensors.target:{ distance, angle, reached }
 *
 * Outputs:
 *   - robot.setVelocity(vx, vy, omega): Set holonomic speed (m/s, rad/s)
 *   - robot.setShape(shape): Morph into "I" | "O" | "L" | "T" | "Z" | "S"
 *   - robot.log(message): Output text to simulator console
 */

// Initialize state machine
if (!memory.initialized) {
  memory.state = "CRUISE";
  memory.dodgeDirection = 1; // 1 = Left, -1 = Right
  memory.stuckTimer = 0;
  memory.initialized = true;
  robot.setShape("O"); // Start with standard stable 2x2 shape
  robot.log("RoboRoarZ Autonomous Script Initialized.");
}

// 1. Read LiDAR Distance Sectors
const frontDist = sensors.lidar.getFront(30);  // Min dist in [-30°, +30°]
const leftDist  = sensors.lidar.getLeft(40);   // Min dist on left flank
const rightDist = sensors.lidar.getRight(40);  // Min dist on right flank
const backDist  = sensors.lidar.getBack(30);

// 2. Narrow Corridor Detection Logic
// If both left and right walls are close (< 0.38m), we are entering a narrow passage!
const isNarrowCorridor = (leftDist < 0.38 && rightDist < 0.38);

if (isNarrowCorridor) {
  if (sensors.shape !== "I") {
    robot.log(">>> Narrow corridor detected! Morphing to streamlined 'I' shape...");
    robot.setShape("I");
  }
  // Drive forward slowly and smoothly through the corridor
  const lateralCorrection = (leftDist - rightDist) * 0.4;
  robot.setVelocity(0.20, lateralCorrection, 0.0);
  return;
}

// 3. Front Obstacle Avoidance Logic (< 0.6 m threshold)
if (frontDist < 0.60) {
  memory.stuckTimer += dt;

  // Decide bypass direction based on open space
  if (leftDist > rightDist) {
    memory.dodgeDirection = 1; // Crab/turn left
  } else {
    memory.dodgeDirection = -1; // Crab/turn right
  }

  // Use Mecanum Holonomic capability:
  // Combine lateral strafing (crabbing) with slight reverse & rotation
  const strafeSpeed = 0.25 * memory.dodgeDirection;
  const turnSpeed = 1.2 * memory.dodgeDirection;
  const reverseSpeed = frontDist < 0.30 ? -0.10 : 0.0;

  robot.setVelocity(reverseSpeed, strafeSpeed, turnSpeed);

  if (Math.random() < 0.02) {
    robot.log(\`Obstacle at \${frontDist.toFixed(2)}m -> Crabbing \${memory.dodgeDirection > 0 ? 'LEFT' : 'RIGHT'}\`);
  }
} else {
  // Clear path ahead: Cruise forward at nominal speed
  memory.stuckTimer = 0;

  // If in open space with shape "I", return to "O" for optimal turning stability
  if (sensors.shape === "I" && leftDist > 0.65 && rightDist > 0.65) {
    robot.log("Open area reached. Restoring 'O' shape.");
    robot.setShape("O");
  }

  robot.setVelocity(0.35, 0.0, 0.0);
}
`,

      // 2. Goal Seeking with Artificial Potential Field & Dynamic Morphing
      goal_seeker: `/**
 * ROBO-ROARZ GOAL-SEEKING & RECONFIGURATION NAVIGATOR
 * Combines attractive goal vector with LiDAR repulsive obstacle forces.
 */

if (!memory.init) {
  memory.init = true;
  robot.setShape("O");
  robot.log("Target Seeking Navigator Started!");
}

const target = sensors.target;
const frontDist = sensors.lidar.getFront(35);
const leftDist = sensors.lidar.getLeft(45);
const rightDist = sensors.lidar.getRight(45);

// Check if Goal Reached!
if (target.reached) {
  robot.setVelocity(0, 0, 0);
  robot.log("MISSION ACCOMPLISHED: Target Objective Reached!");
  return;
}

// Check for tight choke points on the way to goal
if (leftDist < 0.35 && rightDist < 0.35) {
  robot.setShape("I"); // Morph to squeeze through
} else if (sensors.shape === "I" && leftDist > 0.6 && rightDist > 0.6) {
  robot.setShape("O");
}

// 1. Attractive force towards target
let targetAngle = target.angle; // radians relative to heading
let attractiveVx = Math.cos(targetAngle) * 0.32;
let attractiveVy = Math.sin(targetAngle) * 0.32;

// 2. Repulsive force from obstacles
let repulseVx = 0;
let repulseVy = 0;

if (frontDist < 0.65) {
  const urgency = (0.65 - frontDist) / 0.65;
  repulseVx -= urgency * 0.45;
  // Push toward clearer side
  if (leftDist > rightDist) {
    repulseVy += urgency * 0.35;
  } else {
    repulseVy -= urgency * 0.35;
  }
}

// Combine forces for Mecanum holonomic locomotion
let vx = attractiveVx + repulseVx;
let vy = attractiveVy + repulseVy;
let omega = targetAngle * 1.5; // Rotate to face target

// Keep rotation smooth
omega = Math.max(-2.0, Math.min(2.0, omega));

robot.setVelocity(vx, vy, omega);
`,

      // 3. Mecanum Holonomic Omnidirectional Strafe Demo
      holonomic_drift: `/**
 * MECANUM HOLONOMIC DRIFT & ORBIT DEMO
 * Demonstrates 3-DOF crabbing (lateral motion) without turning heading!
 */

if (!memory.t) {
  memory.t = 0;
  robot.setShape("O");
  robot.log("Holonomic Mecanum Strafe Demo Initialized.");
}

memory.t += dt;

// Circular drift trajectory:
// Moves sideways and forward while keeping heading fixed at 0 rad!
const speed = 0.28;
const vx = Math.cos(memory.t * 0.8) * speed;
const vy = Math.sin(memory.t * 0.8) * speed;

// Check front LiDAR
if (sensors.lidar.getFront(25) < 0.4) {
  robot.setVelocity(-0.15, vy, 0);
} else {
  robot.setVelocity(vx, vy, 0.0); // Zero rotation! Pure holonomic translation!
}
`,

      // 4. Wall Follower (PID)
      wall_follower: `/**
 * PID RIGHT-WALL FOLLOWER
 * Maintains constant 0.38m distance to right wall using LiDAR
 */

if (!memory.pid) {
  memory.targetDist = 0.38;
  memory.prevError = 0;
  memory.integral = 0;
  memory.pid = true;
  robot.setShape("O");
  robot.log("Right Wall Follower Initialized.");
}

const frontDist = sensors.lidar.getFront(35);
const rightDist = sensors.lidar.getRight(40);

if (frontDist < 0.5) {
  // Obstacle ahead: Turn left immediately
  robot.setVelocity(0.05, 0.0, 1.5);
  return;
}

// PD Controller on wall distance
const error = rightDist - memory.targetDist;
const derivative = (error - memory.prevError) / dt;
memory.prevError = error;

const Kp = 1.8;
const Kd = 0.4;
const steer = Kp * error + Kd * derivative;

robot.setVelocity(0.28, 0.0, -steer);
`,
    };
  }

  /**
   * Compile user script text into an executable function
   * @param {string} codeText
   */
  compileScript(codeText) {
    this.hasError = false;
    this.lastErrorMessage = null;

    try {
      // Sandboxed function wrapping
      // Arguments: sensors, robot, memory, dt
      this.compiledFunction = new Function("sensors", "robot", "memory", "dt", codeText);
      this.memory = {}; // Reset persistent memory on fresh compile
      this.log("Code compiled successfully.");
      return { success: true };
    } catch (err) {
      this.hasError = true;
      this.lastErrorMessage = `Syntax Error: ${err.message}`;
      this.log(`[SYNTAX ERROR] ${err.message}`, "error");
      return { success: false, error: err.message };
    }
  }

  /**
   * Execute one simulation tick of user code
   * @param {Object} sensorsInput - Read-only sensor snapshot
   * @param {SmorphiRobot} robot - Robot model instance
   * @param {number} dt - Delta time
   */
  executeTick(sensorsInput, robot, dt) {
    if (!this.isRunning || !this.compiledFunction || this.hasError) {
      return;
    }

    // Safe robot controller proxy
    const robotAPI = {
      setVelocity: (vx, vy, omega) => {
        robot.setVelocity(vx, vy, omega);
      },
      setShape: (shape) => {
        return robot.setShape(shape);
      },
      stop: () => {
        robot.stop();
      },
      log: (msg) => {
        this.log(String(msg), "user");
      },
    };

    try {
      this.compiledFunction(sensorsInput, robotAPI, this.memory, dt);
    } catch (err) {
      this.hasError = true;
      this.lastErrorMessage = `Runtime Error: ${err.message}`;
      this.log(`[RUNTIME ERROR] ${err.message}`, "error");
      robot.stop(); // Fail-safe stop
    }
  }

  /**
   * Logging facility for simulator console
   */
  log(message, type = "info") {
    const now = performance.now();
    // Throttle duplicate rapid logs
    if (type === "user" && now - this.lastLogTime < 50) return;
    this.lastLogTime = now;

    const entry = {
      time: new Date().toLocaleTimeString(),
      type,
      text: message,
    };

    for (const cb of this.logCallbacks) {
      cb(entry);
    }
  }

  onLog(callback) {
    this.logCallbacks.push(callback);
  }

  start() {
    this.isRunning = true;
    this.hasError = false;
  }

  stop() {
    this.isRunning = false;
  }

  resetMemory() {
    this.memory = {};
  }
}

if (typeof module !== "undefined" && module.exports) {
  module.exports = CodeEngine;
}
