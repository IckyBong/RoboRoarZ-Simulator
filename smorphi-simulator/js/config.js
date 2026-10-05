/**
 * Smorphi Robotics Simulator - Global Configuration & Physical Constants
 * Platform: Smorphi Modular Reconfigurable Robot (RoboRoarZ Competition Specification)
 */

const CONFIG = {
  // Arena Specifications
  ARENA: {
    WIDTH: 5.0, // meters (X axis: 0 to 5.0m)
    HEIGHT: 5.0, // meters (Y axis: 0 to 5.0m)
    WALL_THICKNESS: 0.1, // meters
    WALL_HEIGHT: 0.4, // meters in 3D
    DEFAULT_SPAWN: { x: 0.8, y: 0.8, theta: 0 }, // Initial robot position
    DEFAULT_GOAL: { x: 4.2, y: 4.2 }, // Default target objective
  },

  // Smorphi Physical Robot Specifications (Single-Block Base Unit)
  ROBOT: {
    WIDTH: 0.170, // 170 mm width (0.17m)
    LENGTH: 0.170, // 170 mm length (0.17m)
    HEIGHT: 0.315, // 315 mm height in 3D (0.315m)
    MASS: 2.20, // 2.2 kg base unit empty mass
    
    // Kinematics & Speed Limits
    MAX_LINEAR_SPEED: 0.6, // m/s (Max holonomic translation)
    MAX_ANGULAR_SPEED: 3.0, // rad/s (Max rotation ~170 deg/s)
    LINEAR_ACCEL: 2.5, // m/s^2
    ANGULAR_ACCEL: 8.0, // rad/s^2
    
    // 4-Wheel Mecanum System (1 Wheel on each chassis corner)
    WHEEL_RADIUS: 0.030, // 30 mm radius (60 mm diameter)
    WHEEL_WIDTH: 0.025, // 25 mm width
    TOTAL_WHEELS: 4,
    WHEEL_OFFSETS: [
      { id: "FL", lx:  0.065, ly:  0.082, rollerSign:  1 }, // Front-Left (+45°)
      { id: "FR", lx:  0.065, ly: -0.082, rollerSign: -1 }, // Front-Right (-45°)
      { id: "RL", lx: -0.065, ly:  0.082, rollerSign: -1 }, // Rear-Left (-45°)
      { id: "RR", lx: -0.065, ly: -0.082, rollerSign:  1 }, // Rear-Right (+45°)
    ],
  },

  // Front Cargo Mesh Scoop Specification
  SCOOP: {
    MOUNT_X: 0.085, // Mounted at front face of robot (+0.085m)
    LENGTH: 0.140, // Extends 140 mm forward (from x=0.085m to x=0.225m)
    OUTER_WIDTH: 0.190, // 190 mm outer width
    INNER_WIDTH: 0.150, // 150 mm inner containment cavity width
    HEIGHT: 0.085, // 85 mm mesh wall height
    LIP_THICKNESS: 0.003, // 3 mm bottom retaining lip
    GROUND_CLEARANCE: 0.004, // 4 mm clearance above floor
    WALL_THICKNESS: 0.012, // 12 mm mesh perimeter thickness
  },

  // Cargo Cubes Physical Specifications
  CARGO: {
    COUNT: 3,
    SIZE: 0.090, // 90 mm x 90 mm x 90 mm (0.09m)
    MASS: 0.35, // 350 grams (0.35 kg) per cube
    TOTAL_MASS: 1.05, // 1.05 kg for all 3 cubes
    COLORS: [
      { id: 1, name: "Amber", hex: 0xf59e0b, css: "#f59e0b", label: "01" },
      { id: 2, name: "Cyan", hex: 0x06b6d4, css: "#06b6d4", label: "02" },
      { id: 3, name: "Emerald", hex: 0x10b981, css: "#10b981", label: "03" },
    ],
    // Physics interaction parameters
    FLOOR_FRICTION_STATIC: 0.38,
    FLOOR_FRICTION_KINETIC: 0.28,
    MESH_SPRING_K: 6500, // N/m
    MESH_DAMPING_C: 140, // Ns/m
    MESH_FRICTION: 0.22,
    SLIP_FACTOR: 0.32, // Traction degradation multiplier
  },

  // 2D LiDAR 360-Degree Emulation (Top tier: y = 0.285m)
  LIDAR: {
    NUM_BEAMS: 360, // 360 beams (1 degree angular resolution)
    MIN_RANGE: 0.05, // 5 cm minimum detection range
    MAX_RANGE: 5.0, // 5.0 meters maximum range
    SCAN_RATE: 10, // Hz emulated scan frequency
    NOISE_SIGMA: 0.004, // 4mm Gaussian noise
    ELEVATION_Y: 0.285, // meters in 3D
  },

  // 6-DOF IMU Emulation
  IMU: {
    UPDATE_RATE: 50, // Hz
    NOISE_YAW: 0.002, // rad
    NOISE_ACCEL: 0.01, // m/s^2
  },

  // Simulation Engine Timing
  SIM: {
    PHYSICS_FPS: 60,
    DT: 1 / 60,
    MAX_SUBSTEPS: 4,
  },

  // Obstacle Density Presets
  DENSITY_PRESETS: {
    LOW: { label: "7.5% - Training Ground", density: 0.075, minObstacles: 5, maxObstacles: 8 },
    MEDIUM: { label: "15.0% - RoboRoarZ Standard", density: 0.15, minObstacles: 12, maxObstacles: 18, forceCorridors: true },
    HIGH: { label: "28.0% - Dense Maze & Tight Chokepoints", density: 0.28, minObstacles: 22, maxObstacles: 30, forceCorridors: true },
  },

  // Map Generation Navigation Profile
  MAP_NAVIGATION: {
    profile: "SINGLE_BLOCK",
    shape: "SINGLE_BLOCK",

    gridResolution: 100,       // 100×100 over 5×5 m = 5 cm cells
    linearSafetyMargin: 0.04,  // 4 cm extra clearance on each side

    minPassageWidth: 0.40,     // Clear passage width for robot
    minTurningClearance: 0.52, // Safe turning envelope target

    pathSampleSpacing: 0.025,  // 2.5 cm exact SAT collision certification
    minStartGoalClearance: 0.55,

    maxGenerationAttempts: 1000,
    enableSeededGeneration: true,
  },

  // Visual Theme Colors
  COLORS: {
    CHASSIS: 0x1e293b,
    CABIN: 0x0f172a,
    ACCENT: 0x06b6d4,
    SCOOP: 0x334155,
    SCOOP_MESH: 0x475569,
    WHEEL: 0x1e293b,
    ROLLER: 0x94a3b8,
    LASER_SAFE: 0x10b981, // Emerald Green
    LASER_WARN: 0xf59e0b, // Amber
    LASER_DANGER: 0xef4444, // Red
    OBSTACLE: 0x475569,
    ARENA_FLOOR: 0x0f172a,
    GOAL: 0xfacc15, // Golden yellow
  }
};

if (typeof module !== "undefined" && module.exports) {
  module.exports = CONFIG;
}
