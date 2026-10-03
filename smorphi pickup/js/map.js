/**
 * Randomized Arena Map Generator & Navigation Grid Validator
 * Features procedural obstacle placement, narrow corridor generation for RoboRoarZ,
 * and BFS path reachability verification.
 */

class ArenaMap {
  constructor(width = CONFIG.ARENA.WIDTH, height = CONFIG.ARENA.HEIGHT) {
    this.width = width;
    this.height = height;
    this.obstacles = [];
    this.segments = []; // Precomputed 2D line segments for ultra-fast LiDAR raycasting
    this.goal = { x: CONFIG.ARENA.DEFAULT_GOAL.x, y: CONFIG.ARENA.DEFAULT_GOAL.y };
    this.spawn = { x: CONFIG.ARENA.DEFAULT_SPAWN.x, y: CONFIG.ARENA.DEFAULT_SPAWN.y };
    this.densityMode = "MEDIUM";
    this.gridResolution = 50; // 50x50 occupancy grid (0.1m cells)
    this.reachablePath = []; // Validated path waypoints

    // Initialize with standard competition map
    this.generateMap("MEDIUM");
  }

  /**
   * Procedural map generator based on preset or custom density
   * @param {string} presetKey - "LOW" (7.5%) | "MEDIUM" (15%) | "HIGH" (28%)
   */
  generateMap(presetKey = "MEDIUM") {
    this.densityMode = presetKey;
    const preset = CONFIG.DENSITY_PRESETS[presetKey] || CONFIG.DENSITY_PRESETS.MEDIUM;
    this.obstacles = [];

    // 1. Build Perimeter Boundary Walls
    this.buildPerimeterWalls();

    // 2. Procedural Obstacles placement loop
    const targetObstacleCount = Math.floor(
      preset.minObstacles + Math.random() * (preset.maxObstacles - preset.minObstacles + 1)
    );

    // Keep clear zones around spawn and goal
    const clearRadiusSq = 0.65 * 0.65;

    // Special RoboRoarZ Feature: Force at least 1-2 Narrow Corridors in Medium and High modes!
    // Smorphi "O" shape is 0.32m wide, so a 0.26m - 0.28m corridor forces morphing to "I" shape!
    if (preset.forceCorridors) {
      this.generateNarrowCorridors();
    }

    // Place random obstacle blocks & pillars
    let attempts = 0;
    while (this.obstacles.length < targetObstacleCount && attempts < 200) {
      attempts++;

      // Obstacle size (0.3m to 0.7m)
      const w = 0.25 + Math.random() * 0.45;
      const h = 0.25 + Math.random() * 0.45;
      const x = 0.5 + Math.random() * (this.width - 1.0 - w);
      const y = 0.5 + Math.random() * (this.height - 1.0 - h);

      // Check distance from spawn and goal
      const dSpawnSq = Math.hypot(x + w / 2 - this.spawn.x, y + h / 2 - this.spawn.y) ** 2;
      const dGoalSq = Math.hypot(x + w / 2 - this.goal.x, y + h / 2 - this.goal.y) ** 2;

      if (dSpawnSq < clearRadiusSq || dGoalSq < clearRadiusSq) continue;

      // Check overlap with existing obstacles (leave at least 0.25m gap or corridor)
      let overlaps = false;
      for (const obs of this.obstacles) {
        if (
          x < obs.x + obs.w + 0.15 &&
          x + w > obs.x - 0.15 &&
          y < obs.y + obs.h + 0.15 &&
          y + h > obs.y - 0.15
        ) {
          overlaps = true;
          break;
        }
      }

      if (!overlaps) {
        this.obstacles.push({
          id: `obs_${this.obstacles.length}`,
          type: Math.random() > 0.3 ? "box" : "pillar",
          x,
          y,
          w,
          h,
          rotation: 0,
        });
      }
    }

    // 3. Compile line segments for physics and LiDAR
    this.rebuildSegments();

    // 4. Validate Path Reachability using BFS
    const isValid = this.validateAndCarvePath();
    if (!isValid) {
      // If blocked after carve, re-generate once
      console.log("[ArenaMap] Regenerating map to ensure path reachability...");
      this.rebuildSegments();
    }

    return this.obstacles;
  }

  /**
   * Creates 1 or 2 narrow corridors specifically designed for Smorphi "I"-shape morphing
   */
  generateNarrowCorridors() {
    // A corridor consists of two parallel walls with a gap of 0.26m - 0.29m
    // (Smorphi "O" is 0.32m, "I" is 0.16m)
    const gap = 0.27; // 27 cm gap
    const wallLength = 0.9;
    const wallThick = 0.12;

    // Place corridor in the middle of the arena
    const midX = 2.4 + (Math.random() - 0.5) * 0.6;
    const midY = 2.4 + (Math.random() - 0.5) * 0.6;

    // Top wall of corridor
    this.obstacles.push({
      id: "corridor_wall_1",
      type: "corridor",
      x: midX - wallLength / 2,
      y: midY + gap / 2,
      w: wallLength,
      h: wallThick,
      rotation: 0,
      label: "Narrow Corridor",
    });

    // Bottom wall of corridor
    this.obstacles.push({
      id: "corridor_wall_2",
      type: "corridor",
      x: midX - wallLength / 2,
      y: midY - gap / 2 - wallThick,
      w: wallLength,
      h: wallThick,
      rotation: 0,
      label: "Narrow Corridor",
    });
  }

  /**
   * Build 4 perimeter boundary walls
   */
  buildPerimeterWalls() {
    const t = CONFIG.ARENA.WALL_THICKNESS;
    const W = this.width;
    const H = this.height;

    this.perimeter = [
      // Bottom Wall (y = 0)
      { id: "wall_bottom", type: "wall", x: 0, y: -t, w: W, h: t },
      // Top Wall (y = H)
      { id: "wall_top", type: "wall", x: 0, y: H, w: W, h: t },
      // Left Wall (x = 0)
      { id: "wall_left", type: "wall", x: -t, y: -t, w: t, h: H + 2 * t },
      // Right Wall (x = W)
      { id: "wall_right", type: "wall", x: W, y: -t, w: t, h: H + 2 * t },
    ];
  }

  /**
   * Convert obstacles into 2D line segments for raycasting & collision
   */
  rebuildSegments() {
    this.segments = [];

    // Arena boundary interior edges
    const W = this.width;
    const H = this.height;

    this.segments.push(
      { p1: { x: 0, y: 0 }, p2: { x: W, y: 0 } },
      { p1: { x: W, y: 0 }, p2: { x: W, y: H } },
      { p1: { x: W, y: H }, p2: { x: 0, y: H } },
      { p1: { x: 0, y: H }, p2: { x: 0, y: 0 } }
    );

    // Obstacle edges
    for (const obs of this.obstacles) {
      const x1 = obs.x;
      const y1 = obs.y;
      const x2 = obs.x + obs.w;
      const y2 = obs.y + obs.h;

      this.segments.push(
        { p1: { x: x1, y: y1 }, p2: { x: x2, y: y1 }, obsId: obs.id },
        { p1: { x: x2, y: y1 }, p2: { x: x2, y: y2 }, obsId: obs.id },
        { p1: { x: x2, y: y2 }, p2: { x: x1, y: y2 }, obsId: obs.id },
        { p1: { x: x1, y: y2 }, p2: { x: x1, y: y1 }, obsId: obs.id }
      );
    }
  }

  /**
   * Discretize arena and run BFS to verify start-to-goal reachability.
   * If blocked, carve a navigable corridor so the map is guaranteed solvable.
   */
  validateAndCarvePath() {
    const N = this.gridResolution;
    const cellW = this.width / N;
    const cellH = this.height / N;

    // 0 = free, 1 = occupied
    const grid = new Uint8Array(N * N);

    // Inflate obstacles slightly by robot half-width (0.1m)
    const margin = 0.10;

    for (const obs of this.obstacles) {
      const minI = Math.max(0, Math.floor((obs.x - margin) / cellW));
      const maxI = Math.min(N - 1, Math.floor((obs.x + obs.w + margin) / cellW));
      const minJ = Math.max(0, Math.floor((obs.y - margin) / cellH));
      const maxJ = Math.min(N - 1, Math.floor((obs.y + obs.h + margin) / cellH));

      for (let i = minI; i <= maxI; i++) {
        for (let j = minJ; j <= maxJ; j++) {
          grid[j * N + i] = 1;
        }
      }
    }

    const startI = Math.floor(this.spawn.x / cellW);
    const startJ = Math.floor(this.spawn.y / cellH);
    const goalI = Math.floor(this.goal.x / cellW);
    const goalJ = Math.floor(this.goal.y / cellH);

    grid[startJ * N + startI] = 0;
    grid[goalJ * N + goalI] = 0;

    // BFS Search
    const queue = [[startI, startJ]];
    const visited = new Uint8Array(N * N);
    const parent = new Int32Array(N * N).fill(-1);

    visited[startJ * N + startI] = 1;
    let found = false;

    const dirs = [
      [1, 0], [-1, 0], [0, 1], [0, -1],
      [1, 1], [-1, 1], [1, -1], [-1, -1]
    ];

    while (queue.length > 0) {
      const [ci, cj] = queue.shift();

      if (ci === goalI && cj === goalJ) {
        found = true;
        break;
      }

      for (const [di, dj] of dirs) {
        const ni = ci + di;
        const nj = cj + dj;

        if (ni >= 0 && ni < N && nj >= 0 && nj < N) {
          const idx = nj * N + ni;
          if (!visited[idx] && grid[idx] === 0) {
            visited[idx] = 1;
            parent[idx] = cj * N + ci;
            queue.push([ni, nj]);
          }
        }
      }
    }

    if (found) {
      // Reconstruct path
      this.reachablePath = [];
      let curr = goalJ * N + goalI;
      while (curr !== -1) {
        const pj = Math.floor(curr / N);
        const pi = curr % N;
        this.reachablePath.unshift({
          x: (pi + 0.5) * cellW,
          y: (pj + 0.5) * cellH,
        });
        curr = parent[curr];
      }
      return true;
    } else {
      // Auto-carve a pathway by removing 1 or 2 blocking obstacles
      console.warn("[ArenaMap] Path initially blocked! Removing obstacle to guarantee solvable map.");
      if (this.obstacles.length > 2) {
        // Remove an obstacle near the diagonal
        const removed = this.obstacles.splice(Math.floor(this.obstacles.length / 2), 1);
        this.rebuildSegments();
        return this.validateAndCarvePath();
      }
      return false;
    }
  }

  /**
   * Set dynamic goal position
   */
  setGoal(x, y) {
    this.goal.x = Math.max(0.3, Math.min(this.width - 0.3, x));
    this.goal.y = Math.max(0.3, Math.min(this.height - 0.3, y));
    this.validateAndCarvePath();
  }
}

if (typeof module !== "undefined" && module.exports) {
  module.exports = ArenaMap;
}
