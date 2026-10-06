/**
 * Physically-Aware Arena Map Generator & Navigation Validator
 * Uses configuration-space inflation, A* pathfinding, minimum-clearance validation,
 * and exact SAT path certification to guarantee generated maps are traversable
 * by the robot in its configured navigation profile.
 */

class ArenaMap {
  constructor(width = CONFIG.ARENA.WIDTH, height = CONFIG.ARENA.HEIGHT) {
    this.width = width;
    this.height = height;
    this.obstacles = [];
    this.segments = [];
    this.goal = { x: CONFIG.ARENA.DEFAULT_GOAL.x, y: CONFIG.ARENA.DEFAULT_GOAL.y };
    this.spawn = { x: CONFIG.ARENA.DEFAULT_SPAWN.x, y: CONFIG.ARENA.DEFAULT_SPAWN.y };
    this.densityMode = "MEDIUM";
    this.reachablePath = [];
    this.reachableGrid = null;
    this.lastGenerationResult = null;

    // Seeded PRNG state
    this._seed = 1;
    this._rng = Math.random; // fallback

    this.generateMap("MEDIUM");
  }

  // --- Seeded PRNG (xoshiro128**-inspired, lightweight) ---

  _seedRng(seed) {
    this._seed = seed;
    // Simple mulberry32
    let s = seed | 0;
    this._rng = () => {
      s = (s + 0x6D2B79F5) | 0;
      let t = Math.imul(s ^ (s >>> 15), 1 | s);
      t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t;
      return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
    };
  }

  _random() {
    return this._rng();
  }

  // --- Main Generation Entry Point ---

  /**
   * @param {string} presetKey - "LOW" | "MEDIUM" | "HIGH"
   * @param {number} [seed] - Optional deterministic seed
   * @returns {Array} obstacles
   */
  generateMap(presetKey = "MEDIUM", seed) {
    this.densityMode = presetKey;
    const preset = CONFIG.DENSITY_PRESETS[presetKey] || CONFIG.DENSITY_PRESETS.MEDIUM;
    const nav = CONFIG.MAP_NAVIGATION;

    if (seed !== undefined) {
      this._seedRng(seed);
    } else if (nav.enableSeededGeneration) {
      this._seedRng((Date.now() ^ (Math.random() * 0xFFFFFFFF)) >>> 0);
    } else {
      this._rng = Math.random;
    }

    this.obstacles = [];
    this.buildPerimeterWalls();

    const targetCount = Math.floor(
      preset.minObstacles + this._random() * (preset.maxObstacles - preset.minObstacles + 1)
    );

    const inflation = RobotGeometry.getObstacleInflation(nav.profile);
    const N = nav.gridResolution;
    const cellSize = this.width / N;
    const clearRadiusSq = nav.minStartGoalClearance * nav.minStartGoalClearance;

    // 4x4 stratified grid sectors covering inner arena bounds
    const margin = 0.35;
    const gridDim = 4;
    const colWidth = (this.width - 2 * margin) / gridDim;
    const rowHeight = (this.height - 2 * margin) / gridDim;
    const sectors = [];
    for (let r = 0; r < gridDim; r++) {
      for (let c = 0; c < gridDim; c++) {
        sectors.push({ c, r, count: 0 });
      }
    }

    const minCenterDistSq = 0.45 * 0.45;
    let attempts = 0;
    let bestResult = null;

    while (this.obstacles.length < targetCount && attempts < nav.maxGenerationAttempts) {
      attempts++;

      // Pick from sectors with fewest obstacles to enforce spatial dispersion
      const sortedSectors = [...sectors].sort((a, b) => (a.count - b.count) || (this._random() - 0.5));
      const sec = sortedSectors[Math.floor(this._random() * Math.min(4, sortedSectors.length))];

      const secCx = margin + (sec.c + 0.5) * colWidth;
      const secCy = margin + (sec.r + 0.5) * rowHeight;

      // Obstacle size (mixed compact shapes: pillars, slender barriers, compact boxes)
      const roll = this._random();
      let w, h;
      if (roll < 0.40) {
        // Small pillar / column
        w = 0.18 + this._random() * 0.12;
        h = 0.18 + this._random() * 0.12;
      } else if (roll < 0.70) {
        // Slender wall / barrier
        if (this._random() < 0.5) {
          w = 0.30 + this._random() * 0.18;
          h = 0.16 + this._random() * 0.08;
        } else {
          w = 0.16 + this._random() * 0.08;
          h = 0.30 + this._random() * 0.18;
        }
      } else {
        // Compact box
        w = 0.22 + this._random() * 0.16;
        h = 0.22 + this._random() * 0.16;
      }

      // Sample center around sector center with jitter across sector
      const cxRaw = secCx + (this._random() - 0.5) * colWidth * 1.4;
      const cyRaw = secCy + (this._random() - 0.5) * rowHeight * 1.4;

      const x = Math.max(margin, Math.min(this.width - margin - w, cxRaw - w / 2));
      const y = Math.max(margin, Math.min(this.height - margin - h, cyRaw - h / 2));

      const cx = x + w / 2;
      const cy = y + h / 2;

      // Reject if too close to spawn or goal
      if (Math.hypot(cx - this.spawn.x, cy - this.spawn.y) ** 2 < clearRadiusSq) continue;
      if (Math.hypot(cx - this.goal.x, cy - this.goal.y) ** 2 < clearRadiusSq) continue;

      // Reject if too close to center of existing obstacles (inter-obstacle distance >= 0.45m)
      let tooClose = false;
      for (const obs of this.obstacles) {
        const ocx = obs.x + obs.w / 2;
        const ocy = obs.y + obs.h / 2;
        if ((cx - ocx) ** 2 + (cy - ocy) ** 2 < minCenterDistSq) {
          tooClose = true;
          break;
        }
      }
      if (tooClose) continue;

      // Reject if overlaps existing obstacles (with minimum gap)
      const minGap = nav.minPassageWidth;
      let overlaps = false;
      for (const obs of this.obstacles) {
        if (x < obs.x + obs.w + minGap &&
            x + w > obs.x - minGap &&
            y < obs.y + obs.h + minGap &&
            y + h > obs.y - minGap) {
          overlaps = true;
          break;
        }
      }
      if (overlaps) continue;

      // Tentatively add
      const candidate = {
        id: `obs_${this.obstacles.length}`,
        type: roll < 0.40 ? "pillar" : "box",
        x, y, w, h, rotation: 0,
      };
      this.obstacles.push(candidate);

      // Validate: build grid, run A*, check clearance, certify with SAT
      const validation = this._validateMap(N, cellSize, inflation);

      if (!validation.valid) {
        // Reject candidate — remove it
        this.obstacles.pop();
        continue;
      }

      const sc = Math.min(gridDim - 1, Math.max(0, Math.floor((cx - margin) / colWidth)));
      const sr = Math.min(gridDim - 1, Math.max(0, Math.floor((cy - margin) / rowHeight)));
      sectors[sr * gridDim + sc].count++;
      bestResult = validation;
    }

    // If we have obstacles but no bestResult yet, validate current state
    if (!bestResult) {
      bestResult = this._validateMap(N, cellSize, inflation);
    }

    this.rebuildSegments();

    if (bestResult && bestResult.valid) {
      this.reachablePath = bestResult.path.points;
      const startI = Math.min(N - 1, Math.max(0, Math.floor(this.spawn.x / cellSize)));
      const startJ = Math.min(N - 1, Math.max(0, Math.floor(this.spawn.y / cellSize)));
      const finalGrid = this._buildCSpaceGrid(N, cellSize, inflation);
      this.reachableGrid = this._computeReachableGrid(finalGrid, N, startI, startJ);
    } else {
      this.reachablePath = [];
      this.reachableGrid = null;
    }

    this.lastGenerationResult = {
      valid: bestResult ? bestResult.valid : false,
      obstacleCount: this.obstacles.length,
      requestedObstacleCount: targetCount,
      pathLength: bestResult ? bestResult.path.length : 0,
      minimumClearance: bestResult ? bestResult.path.minimumClearance : 0,
      attempts: attempts,
      profile: nav.profile,
      seed: this._seed,
    };

    console.log(`[ArenaMap] Generated: ${this.obstacles.length}/${targetCount} obstacles, ` +
      `path=${(this.lastGenerationResult.pathLength).toFixed(2)}m, ` +
      `minClearance=${(this.lastGenerationResult.minimumClearance).toFixed(3)}m, ` +
      `attempts=${attempts}, profile=${nav.profile}`);

    this._updateDebugUI();
    return this.obstacles;
  }

  // --- Validation Pipeline ---

  _validateMap(N, cellSize, inflation) {
    const nav = CONFIG.MAP_NAVIGATION;

    // Step 1: Check spawn and goal physically safe
    if (RobotGeometry.testRobotCollision(
      nav.shape, this.spawn.x, this.spawn.y, 0,
      this.obstacles, this.width, this.height
    )) {
      return { valid: false, reason: "spawn_collision" };
    }

    if (RobotGeometry.testRobotCollision(
      nav.shape, this.goal.x, this.goal.y, 0,
      this.obstacles, this.width, this.height
    )) {
      return { valid: false, reason: "goal_collision" };
    }

    // Step 2: Build configuration-space occupancy grid
    const grid = this._buildCSpaceGrid(N, cellSize, inflation);

    // Step 3: Verify spawn and goal cells are free (without forcing them)
    const startI = Math.floor(this.spawn.x / cellSize);
    const startJ = Math.floor(this.spawn.y / cellSize);
    const goalI = Math.floor(this.goal.x / cellSize);
    const goalJ = Math.floor(this.goal.y / cellSize);

    const si = Math.min(N - 1, Math.max(0, startI));
    const sj = Math.min(N - 1, Math.max(0, startJ));
    const gi = Math.min(N - 1, Math.max(0, goalI));
    const gj = Math.min(N - 1, Math.max(0, goalJ));

    if (grid[sj * N + si] === 1) return { valid: false, reason: "spawn_cell_blocked" };
    if (grid[gj * N + gi] === 1) return { valid: false, reason: "goal_cell_blocked" };

    // Step 4: A* pathfinding
    const astarResult = this._astar(grid, N, si, sj, gi, gj, cellSize);
    if (!astarResult.found) return { valid: false, reason: "no_path" };

    // Step 5: Minimum clearance validation
    const clearance = this._computePathClearance(astarResult.points, N, cellSize, grid);
    if (clearance.minimum < nav.minPassageWidth) {
      return { valid: false, reason: "insufficient_clearance", clearance: clearance.minimum };
    }

    // Step 6: Turning clearance at significant direction changes
    if (!this._validateTurningClearance(astarResult.points, clearance, nav)) {
      return { valid: false, reason: "insufficient_turning_clearance" };
    }

    // Step 7: Exact SAT path certification
    if (!this._certifyPathSAT(astarResult.points, nav)) {
      return { valid: false, reason: "sat_certification_failed" };
    }

    return {
      valid: true,
      path: {
        points: astarResult.points,
        length: astarResult.length,
        minimumClearance: clearance.minimum,
      },
    };
  }

  // --- C-Space Grid Construction ---

  _buildCSpaceGrid(N, cellSize, inflation) {
    const grid = new Uint8Array(N * N);

    // Mark boundary inflation (robot center cannot be within inflation of boundary)
    const boundaryMargin = Math.ceil(inflation / cellSize);
    for (let i = 0; i < N; i++) {
      for (let j = 0; j < N; j++) {
        if (i < boundaryMargin || i >= N - boundaryMargin ||
            j < boundaryMargin || j >= N - boundaryMargin) {
          grid[j * N + i] = 1;
        }
      }
    }

    // Inflate each obstacle
    for (const obs of this.obstacles) {
      const minI = Math.max(0, Math.floor((obs.x - inflation) / cellSize));
      const maxI = Math.min(N - 1, Math.floor((obs.x + obs.w + inflation) / cellSize));
      const minJ = Math.max(0, Math.floor((obs.y - inflation) / cellSize));
      const maxJ = Math.min(N - 1, Math.floor((obs.y + obs.h + inflation) / cellSize));
      for (let i = minI; i <= maxI; i++) {
        for (let j = minJ; j <= maxJ; j++) {
          grid[j * N + i] = 1;
        }
      }
    }

    return grid;
  }

  // --- Reachability Analysis (BFS flood-fill on C-Space) ---

  _computeReachableGrid(grid, N, si, sj) {
    const reachable = new Uint8Array(N * N);
    const startIdx = sj * N + si;
    if (grid[startIdx] === 1) return reachable;

    const queue = new Int32Array(N * N);
    let head = 0;
    let tail = 0;

    reachable[startIdx] = 1;
    queue[tail++] = startIdx;

    const dirs = [
      [1, 0, false], [-1, 0, false], [0, 1, false], [0, -1, false],
      [1, 1, true], [-1, 1, true], [1, -1, true], [-1, -1, true],
    ];

    while (head < tail) {
      const currIdx = queue[head++];
      const ci = currIdx % N;
      const cj = (currIdx - ci) / N;

      for (let d = 0; d < 8; d++) {
        const di = dirs[d][0];
        const dj = dirs[d][1];
        const isDiag = dirs[d][2];
        const ni = ci + di;
        const nj = cj + dj;
        if (ni < 0 || ni >= N || nj < 0 || nj >= N) continue;
        const nIdx = nj * N + ni;
        if (reachable[nIdx] || grid[nIdx] === 1) continue;

        if (isDiag) {
          if (grid[cj * N + ni] === 1 || grid[nj * N + ci] === 1) continue;
        }

        reachable[nIdx] = 1;
        queue[tail++] = nIdx;
      }
    }
    return reachable;
  }

  /**
   * Check if a world point (x, y) is reachable by robot from spawn
   * @param {number} x
   * @param {number} y
   * @returns {boolean}
   */
  isPointReachable(x, y) {
    const nav = CONFIG.MAP_NAVIGATION;
    const N = nav.gridResolution;
    const cellSize = this.width / N;

    if (!this.reachableGrid) {
      const inflation = RobotGeometry.getObstacleInflation(nav.profile);
      const grid = this._buildCSpaceGrid(N, cellSize, inflation);
      const si = Math.min(N - 1, Math.max(0, Math.floor(this.spawn.x / cellSize)));
      const sj = Math.min(N - 1, Math.max(0, Math.floor(this.spawn.y / cellSize)));
      this.reachableGrid = this._computeReachableGrid(grid, N, si, sj);
    }

    const i = Math.floor(x / cellSize);
    const j = Math.floor(y / cellSize);
    if (i < 0 || i >= N || j < 0 || j >= N) return false;
    return this.reachableGrid[j * N + i] === 1;
  }

  // --- A* Pathfinding (8-connected, no diagonal corner cutting, binary heap) ---

  _astar(grid, N, si, sj, gi, gj, cellSize) {
    const gScore = new Float32Array(N * N).fill(Infinity);
    const parent = new Int32Array(N * N).fill(-1);
    const closed = new Uint8Array(N * N);

    const startIdx = sj * N + si;
    const goalIdx = gj * N + gi;
    gScore[startIdx] = 0;

    // Binary min-heap: [fScore, nodeIdx]
    const heap = new MinHeap();
    const heuristic = (i, j) => {
      // Octile distance
      const dx = Math.abs(i - gi);
      const dy = Math.abs(j - gj);
      return (dx + dy - 0.5858 * Math.min(dx, dy)) * cellSize;
    };
    heap.push(heuristic(si, sj), startIdx);

    const SQRT2 = Math.SQRT2 * cellSize;
    const CARD = cellSize;

    // 8 directions: [di, dj, cost, isDiagonal]
    const dirs = [
      [1, 0, CARD, false], [-1, 0, CARD, false],
      [0, 1, CARD, false], [0, -1, CARD, false],
      [1, 1, SQRT2, true], [-1, 1, SQRT2, true],
      [1, -1, SQRT2, true], [-1, -1, SQRT2, true],
    ];

    while (heap.size > 0) {
      const [, currIdx] = heap.pop();
      if (currIdx === goalIdx) break;
      if (closed[currIdx]) continue;
      closed[currIdx] = 1;

      const ci = currIdx % N;
      const cj = (currIdx - ci) / N;

      for (const [di, dj, cost, isDiag] of dirs) {
        const ni = ci + di;
        const nj = cj + dj;
        if (ni < 0 || ni >= N || nj < 0 || nj >= N) continue;
        const nIdx = nj * N + ni;
        if (closed[nIdx] || grid[nIdx] === 1) continue;

        // Diagonal corner-cutting prevention
        if (isDiag) {
          if (grid[cj * N + ni] === 1 || grid[nj * N + ci] === 1) continue;
        }

        const tentG = gScore[currIdx] + cost;
        if (tentG < gScore[nIdx]) {
          gScore[nIdx] = tentG;
          parent[nIdx] = currIdx;
          heap.push(tentG + heuristic(ni, nj), nIdx);
        }
      }
    }

    if (gScore[goalIdx] === Infinity) {
      return { found: false, points: [], length: 0 };
    }

    // Reconstruct path
    const points = [];
    let idx = goalIdx;
    while (idx !== -1) {
      const pi = idx % N;
      const pj = (idx - pi) / N;
      points.push({ x: (pi + 0.5) * cellSize, y: (pj + 0.5) * cellSize });
      idx = parent[idx];
    }
    points.reverse();

    // Compute path length
    let length = 0;
    for (let k = 1; k < points.length; k++) {
      length += Math.hypot(points[k].x - points[k - 1].x, points[k].y - points[k - 1].y);
    }

    return { found: true, points, length };
  }

  // --- Minimum Clearance Computation ---

  _computePathClearance(points, N, cellSize, grid) {
    let minimum = Infinity;
    let bottleneckIdx = 0;

    for (let k = 0; k < points.length; k++) {
      const px = points[k].x;
      const py = points[k].y;

      // Distance to nearest obstacle edge
      let minDist = Infinity;

      // Check arena boundary
      minDist = Math.min(minDist, px, py, this.width - px, this.height - py);

      // Check all obstacles
      for (const obs of this.obstacles) {
        // Distance from point to AABB
        const dx = Math.max(obs.x - px, 0, px - (obs.x + obs.w));
        const dy = Math.max(obs.y - py, 0, py - (obs.y + obs.h));
        const d = Math.hypot(dx, dy);
        // If inside obstacle, distance is 0
        if (px >= obs.x && px <= obs.x + obs.w && py >= obs.y && py <= obs.y + obs.h) {
          minDist = 0;
        } else {
          minDist = Math.min(minDist, d);
        }
      }

      if (minDist < minimum) {
        minimum = minDist;
        bottleneckIdx = k;
      }
    }

    return { minimum, bottleneckIdx, average: minimum };
  }

  // --- Turning Clearance Validation ---

  _validateTurningClearance(points, clearance, nav) {
    if (points.length < 3) return true;

    for (let k = 1; k < points.length - 1; k++) {
      const prev = points[k - 1];
      const curr = points[k];
      const next = points[k + 1];

      // Compute heading change
      const a1 = Math.atan2(curr.y - prev.y, curr.x - prev.x);
      const a2 = Math.atan2(next.y - curr.y, next.x - curr.x);
      let dAngle = Math.abs(a2 - a1);
      if (dAngle > Math.PI) dAngle = 2 * Math.PI - dAngle;

      // Significant heading change (> 30 degrees)
      if (dAngle > 0.52) {
        // Check local clearance at this point
        let localClearance = Infinity;
        localClearance = Math.min(localClearance, curr.x, curr.y,
          this.width - curr.x, this.height - curr.y);

        for (const obs of this.obstacles) {
          const dx = Math.max(obs.x - curr.x, 0, curr.x - (obs.x + obs.w));
          const dy = Math.max(obs.y - curr.y, 0, curr.y - (obs.y + obs.h));
          localClearance = Math.min(localClearance, Math.hypot(dx, dy));
        }

        if (localClearance < nav.minTurningClearance / 2) {
          return false;
        }
      }
    }
    return true;
  }

  // --- Exact SAT Path Certification ---

  _certifyPathSAT(points, nav) {
    const spacing = nav.pathSampleSpacing;
    const shape = nav.shape;

    // Test at each path point and interpolated points between
    for (let k = 0; k < points.length; k++) {
      // Test at path node
      if (RobotGeometry.testRobotCollision(
        shape, points[k].x, points[k].y, 0,
        this.obstacles, this.width, this.height
      )) {
        return false;
      }

      // Also test at 90° rotation (orientation-independence)
      if (RobotGeometry.testRobotCollision(
        shape, points[k].x, points[k].y, Math.PI / 2,
        this.obstacles, this.width, this.height
      )) {
        return false;
      }

      // Interpolate between this point and next
      if (k < points.length - 1) {
        const dx = points[k + 1].x - points[k].x;
        const dy = points[k + 1].y - points[k].y;
        const segLen = Math.hypot(dx, dy);
        const steps = Math.ceil(segLen / spacing);

        for (let s = 1; s < steps; s++) {
          const t = s / steps;
          const ix = points[k].x + dx * t;
          const iy = points[k].y + dy * t;

          if (RobotGeometry.testRobotCollision(
            shape, ix, iy, 0,
            this.obstacles, this.width, this.height
          )) {
            return false;
          }

          // Test rotated
          if (RobotGeometry.testRobotCollision(
            shape, ix, iy, Math.PI / 4,
            this.obstacles, this.width, this.height
          )) {
            return false;
          }
        }
      }
    }

    return true;
  }

  // --- Map Quality Score ---

  _scoreMap(pathResult) {
    if (!pathResult || !pathResult.valid) return 0;

    const path = pathResult.path;
    const obstacleArea = this.obstacles.reduce((sum, o) => sum + o.w * o.h, 0);
    const arenaArea = this.width * this.height;

    let score = 0;
    score += Math.min(this.obstacles.length / 15, 1) * 0.25; // obstacle count
    score += Math.min(obstacleArea / arenaArea / 0.15, 1) * 0.15; // density
    score += Math.min(path.length / 8, 1) * 0.25; // path length (longer = more interesting)
    score += Math.min(path.minimumClearance / 0.5, 1) * 0.20; // clearance consistency
    score += (path.points.length > 5 ? 0.15 : 0.05); // meaningful turns

    return score;
  }

  // --- Perimeter Walls ---

  buildPerimeterWalls() {
    const t = CONFIG.ARENA.WALL_THICKNESS;
    const W = this.width;
    const H = this.height;
    this.perimeter = [
      { id: "wall_bottom", type: "wall", x: 0, y: -t, w: W, h: t },
      { id: "wall_top", type: "wall", x: 0, y: H, w: W, h: t },
      { id: "wall_left", type: "wall", x: -t, y: -t, w: t, h: H + 2 * t },
      { id: "wall_right", type: "wall", x: W, y: -t, w: t, h: H + 2 * t },
    ];
  }

  // --- Segment Compilation for LiDAR & Physics ---

  rebuildSegments() {
    this.segments = [];
    const W = this.width;
    const H = this.height;

    this.segments.push(
      { p1: { x: 0, y: 0 }, p2: { x: W, y: 0 } },
      { p1: { x: W, y: 0 }, p2: { x: W, y: H } },
      { p1: { x: W, y: H }, p2: { x: 0, y: H } },
      { p1: { x: 0, y: H }, p2: { x: 0, y: 0 } }
    );

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

  // --- Dynamic Goal ---

  setGoal(x, y) {
    this.goal.x = Math.max(0.3, Math.min(this.width - 0.3, x));
    this.goal.y = Math.max(0.3, Math.min(this.height - 0.3, y));

    const nav = CONFIG.MAP_NAVIGATION;
    const N = nav.gridResolution;
    const cellSize = this.width / N;
    const inflation = RobotGeometry.getObstacleInflation(nav.profile);
    const result = this._validateMap(N, cellSize, inflation);

    if (result.valid) {
      this.reachablePath = result.path.points;
      const startI = Math.min(N - 1, Math.max(0, Math.floor(this.spawn.x / cellSize)));
      const startJ = Math.min(N - 1, Math.max(0, Math.floor(this.spawn.y / cellSize)));
      const finalGrid = this._buildCSpaceGrid(N, cellSize, inflation);
      this.reachableGrid = this._computeReachableGrid(finalGrid, N, startI, startJ);
    }
  }

  // --- Debug UI ---

  _updateDebugUI() {
    if (typeof document === "undefined") return;
    const el = document.getElementById("map-gen-debug");
    if (!el || !this.lastGenerationResult) return;
    const r = this.lastGenerationResult;
    el.innerHTML =
      `<div class="text-[11px] font-mono space-y-0.5">` +
      `<div>Map Valid: <span class="${r.valid ? 'text-emerald-400' : 'text-red-400'} font-bold">${r.valid ? 'YES' : 'NO'}</span></div>` +
      `<div>Profile: <span class="text-cyan-400">${r.profile}</span></div>` +
      `<div>Path: <span class="text-slate-200">${r.pathLength.toFixed(2)} m</span></div>` +
      `<div>Min Clearance: <span class="text-slate-200">${r.minimumClearance.toFixed(3)} m</span></div>` +
      `<div>Obstacles: <span class="text-slate-200">${r.obstacleCount}/${r.requestedObstacleCount}</span></div>` +
      `<div>Attempts: <span class="text-slate-200">${r.attempts}</span></div>` +
      `</div>`;
  }
}

// --- Binary Min-Heap for A* ---

class MinHeap {
  constructor() {
    this._data = []; // [fScore, value] pairs
    this.size = 0;
  }

  push(priority, value) {
    this._data.push([priority, value]);
    this.size++;
    this._bubbleUp(this.size - 1);
  }

  pop() {
    if (this.size === 0) return null;
    const top = this._data[0];
    this.size--;
    if (this.size > 0) {
      this._data[0] = this._data.pop();
      this._sinkDown(0);
    } else {
      this._data.pop();
    }
    return top;
  }

  _bubbleUp(i) {
    const d = this._data;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (d[i][0] < d[p][0]) {
        [d[i], d[p]] = [d[p], d[i]];
        i = p;
      } else break;
    }
  }

  _sinkDown(i) {
    const d = this._data;
    const n = this.size;
    while (true) {
      let min = i;
      const l = 2 * i + 1;
      const r = 2 * i + 2;
      if (l < n && d[l][0] < d[min][0]) min = l;
      if (r < n && d[r][0] < d[min][0]) min = r;
      if (min !== i) {
        [d[i], d[min]] = [d[min], d[i]];
        i = min;
      } else break;
    }
  }
}

// --- Stress Test Runner ---

/**
 * Run deterministic stress tests across all density presets.
 * Call from browser console: ArenaMap.runStressTest(1000)
 * @param {number} count - Maps per preset
 */
ArenaMap.runStressTest = function(count = 1000) {
  const presets = ["LOW", "MEDIUM", "HIGH"];
  const results = {};
  const nav = CONFIG.MAP_NAVIGATION;
  const testMap = new ArenaMap(CONFIG.ARENA.WIDTH, CONFIG.ARENA.HEIGHT);

  for (const preset of presets) {
    let valid = 0;
    let invalid = 0;
    const failures = [];
    const t0 = performance.now();

    for (let i = 0; i < count; i++) {
      const seed = 100000 + i;
      testMap.generateMap(preset, seed);
      const r = testMap.lastGenerationResult;

      if (!r.valid) {
        invalid++;
        failures.push({ seed, reason: "invalid_result" });
        continue;
      }

      // Verify: obstacles don't illegally overlap
      let obsOverlap = false;
      for (let a = 0; a < testMap.obstacles.length && !obsOverlap; a++) {
        for (let b = a + 1; b < testMap.obstacles.length; b++) {
          const oa = testMap.obstacles[a];
          const ob = testMap.obstacles[b];
          if (oa.x < ob.x + ob.w && oa.x + oa.w > ob.x &&
              oa.y < ob.y + ob.h && oa.y + oa.h > ob.y) {
            obsOverlap = true;
            break;
          }
        }
      }
      if (obsOverlap) {
        invalid++;
        failures.push({ seed, reason: "obstacle_overlap" });
        continue;
      }

      // Verify: spawn and goal safe
      if (RobotGeometry.testRobotCollision(nav.shape, testMap.spawn.x, testMap.spawn.y, 0,
          testMap.obstacles, testMap.width, testMap.height)) {
        invalid++;
        failures.push({ seed, reason: "spawn_unsafe" });
        continue;
      }
      if (RobotGeometry.testRobotCollision(nav.shape, testMap.goal.x, testMap.goal.y, 0,
          testMap.obstacles, testMap.width, testMap.height)) {
        invalid++;
        failures.push({ seed, reason: "goal_unsafe" });
        continue;
      }

      // Verify: path exists and clearance sufficient
      if (r.pathLength <= 0) {
        invalid++;
        failures.push({ seed, reason: "no_path" });
        continue;
      }
      if (r.minimumClearance < nav.minPassageWidth * 0.95) {
        invalid++;
        failures.push({ seed, reason: "clearance_too_low", clearance: r.minimumClearance });
        continue;
      }

      // Verify: SAT re-certification
      const pathValid = testMap._certifyPathSAT(testMap.reachablePath, nav);
      if (!pathValid) {
        invalid++;
        failures.push({ seed, reason: "sat_recheck_failed" });
        continue;
      }

      valid++;
    }

    const elapsed = performance.now() - t0;
    results[preset] = { valid, invalid, total: count, elapsed: Math.round(elapsed), failures: failures.slice(0, 10) };
  }

  // Print summary
  console.log("\n=== MAP GENERATION STRESS TEST ===\n");
  for (const preset of presets) {
    const r = results[preset];
    const status = r.invalid === 0 ? "✅ PASS" : "❌ FAIL";
    console.log(`${preset}: ${r.valid}/${r.total} valid  (${r.elapsed}ms)  ${status}`);
    if (r.failures.length > 0) {
      console.log(`  First failures:`, r.failures);
    }
  }
  console.log("\n=================================\n");

  return results;
};

if (typeof module !== "undefined" && module.exports) {
  module.exports = { ArenaMap, MinHeap };
}
