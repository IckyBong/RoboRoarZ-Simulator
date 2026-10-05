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

  // Smorphi Physical Robot Specifications
  ROBOT: {
    NUM_MODULES: 4,
    MODULE_SIZE: 0.16, // 16 cm (0.16m x 0.16m x 0.16m per modular cube)
    MODULE_HEIGHT: 0.16, // 16 cm height in 3D
    MODULE_MASS: 0.5, // 500 grams (0.5 kg) per module
    TOTAL_MASS: 2.0, // Total mass: 2.0 kg
    
    // Kinematics & Speed Limits
    MAX_LINEAR_SPEED: 0.6, // m/s (Max holonomic translation)
    MAX_ANGULAR_SPEED: 3.0, // rad/s (Max rotation ~170 deg/s)
    LINEAR_ACCEL: 2.5, // m/s^2
    ANGULAR_ACCEL: 8.0, // rad/s^2
    
    // Mecanum Wheel System (16 Wheels: 4 per module)
    WHEEL_RADIUS: 0.03, // 30 mm radius (60 mm diameter)
    WHEEL_WIDTH: 0.025, // 25 mm width
    WHEELS_PER_MODULE: 4,
    TOTAL_WHEELS: 16,

    // Reconfiguration / Morphing
    MORPH_DURATION: 0.6, // seconds to complete hinge transformation
    HINGE_RADIUS: 0.015, // hinge cylinder radius
  },

  // 7 Tetromino Morphological Shapes (Coordinates in meters relative to centroid)
  // [dx, dy] where dx is along robot heading (Forward), dy is Lateral (Left)
  SHAPES: {
    // Monomino / Straight Line (1x4) - Streamlined profile (0.16m width) for narrow corridor passage
    "I": [
      { id: 0, x: -0.24, y: 0.0 },
      { id: 1, x: -0.08, y: 0.0 },
      { id: 2, x:  0.08, y: 0.0 },
      { id: 3, x:  0.24, y: 0.0 },
    ],

    // Square (2x2) - Compact, balanced footprint (0.32m x 0.32m)
    "O": [
      { id: 0, x: -0.08, y: -0.08 },
      { id: 1, x:  0.08, y: -0.08 },
      { id: 2, x: -0.08, y:  0.08 },
      { id: 3, x:  0.08, y:  0.08 },
    ],

    // L-Shape
    "L": [
      { id: 0, x: -0.16, y: -0.08 },
      { id: 1, x:  0.00, y: -0.08 },
      { id: 2, x:  0.16, y: -0.08 },
      { id: 3, x: -0.16, y:  0.08 },
    ],

    // T-Shape
    "T": [
      { id: 0, x: -0.08, y: -0.16 },
      { id: 1, x: -0.08, y:  0.00 },
      { id: 2, x: -0.08, y:  0.16 },
      { id: 3, x:  0.08, y:  0.00 },
    ],

    // Z-Shape
    "Z": [
      { id: 0, x:  0.08, y: -0.16 },
      { id: 1, x:  0.08, y:  0.00 },
      { id: 2, x: -0.08, y:  0.00 },
      { id: 3, x: -0.08, y:  0.16 },
    ],

    // S-Shape (Mirrored Z)
    "S": [
      { id: 0, x: -0.08, y: -0.16 },
      { id: 1, x: -0.08, y:  0.00 },
      { id: 2, x:  0.08, y:  0.00 },
      { id: 3, x:  0.08, y:  0.16 },
    ],

    // J-Shape (Mirrored L / Line Variation)
    "J": [
      { id: 0, x: -0.16, y:  0.08 },
      { id: 1, x:  0.00, y:  0.08 },
      { id: 2, x:  0.16, y:  0.08 },
      { id: 3, x: -0.16, y: -0.08 },
    ],
  },

  // 2D LiDAR 360-Degree Emulation
  LIDAR: {
    NUM_BEAMS: 360, // 360 beams (1 degree angular resolution)
    MIN_RANGE: 0.05, // 5 cm minimum detection range
    MAX_RANGE: 5.0, // 5.0 meters maximum range
    SCAN_RATE: 10, // Hz emulated scan frequency
    NOISE_SIGMA: 0.004, // 4mm Gaussian noise
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
  // Conservative defaults for FIXED_O — easy to recalibrate after physical measurement
  MAP_NAVIGATION: {
    profile: "FIXED_O",        // Active navigation profile
    shape: "O",                // Robot shape assumed during map validation

    gridResolution: 100,       // 100×100 over 5×5 m = 5 cm cells
    linearSafetyMargin: 0.04,  // 4 cm extra clearance on each side

    minPassageWidth: 0.40,     // Fixed O footprint + safety margin
    minTurningClearance: 0.52, // Safe turning envelope target

    pathSampleSpacing: 0.025,  // 2.5 cm exact SAT collision certification
    minStartGoalClearance: 0.55,

    maxGenerationAttempts: 1000,
    enableSeededGeneration: true,
  },

  // Visual Theme Colors
  COLORS: {
    MODULES: [
      0x06b6d4, // M1: Cyan (Master Controller)
      0x3b82f6, // M2: Blue
      0x8b5cf6, // M3: Violet
      0xec4899, // M4: Pink
    ],
    CHASSIS: 0x1e293b,
    WHEEL: 0x334155,
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
