const assert = require("assert");

// Expose globals for simulator scripts
global.CONFIG = require("./js/config.js");
global.RobotGeometry = require("./js/geometry.js");
const { ArenaMap } = require("./js/map.js");
const { CargoManager } = require("./js/cargo.js");
const { FogOfWar } = require("./js/fog.js");
const { CheckpointManager } = require("./js/checkpoints.js");

console.log("Running self-check tests for dispersed obstacle map generation & dynamic cargo spawning...");

// 1. Verify DENSITY_PRESETS
assert.strictEqual(CONFIG.DENSITY_PRESETS.LOW.minObstacles, 8);
assert.strictEqual(CONFIG.DENSITY_PRESETS.LOW.maxObstacles, 12);
assert.strictEqual(CONFIG.DENSITY_PRESETS.MEDIUM.minObstacles, 14);
assert.strictEqual(CONFIG.DENSITY_PRESETS.MEDIUM.maxObstacles, 18);
assert.strictEqual(CONFIG.DENSITY_PRESETS.HIGH.minObstacles, 18);
assert.strictEqual(CONFIG.DENSITY_PRESETS.HIGH.maxObstacles, 22);

const map = new ArenaMap(5.0, 5.0);
const cargo = new CargoManager();

// Test multiple seeds for MEDIUM preset
const routeMidpoints = [];

for (let seed = 101; seed <= 120; seed++) {
  map.generateMap("MEDIUM", seed);
  const result = map.lastGenerationResult;

  assert(result.valid, `Map generation failed for seed ${seed}`);
  assert(map.obstacles.length >= 14 && map.obstacles.length <= 18,
    `Obstacle count ${map.obstacles.length} out of range [14, 18] for seed ${seed}`);

  // Test inter-obstacle center distance >= 0.45m
  for (let i = 0; i < map.obstacles.length; i++) {
    const o1 = map.obstacles[i];
    const c1x = o1.x + o1.w / 2;
    const c1y = o1.y + o1.h / 2;

    for (let j = i + 1; j < map.obstacles.length; j++) {
      const o2 = map.obstacles[j];
      const c2x = o2.x + o2.w / 2;
      const c2y = o2.y + o2.h / 2;
      const dist = Math.hypot(c1x - c2x, c1y - c2y);
      assert(dist >= 0.44, `Obstacles ${i} and ${j} too close: ${dist.toFixed(3)}m < 0.45m`);
    }
  }

  // Record max deviation from diagonal line along path
  assert(map.reachablePath.length > 5, `Reachable path too short`);
  let maxDev = 0;
  for (const pt of map.reachablePath) {
    const d = Math.abs(pt.x - pt.y) / Math.SQRT2;
    if (d > maxDev) maxDev = d;
  }
  routeMidpoints.push(maxDev);

  // Test Cargo Spawning
  cargo.spawnCubes(map);
  assert.strictEqual(cargo.cubes.length, 3, "Expected 3 cargo cubes");

  for (let i = 0; i < 3; i++) {
    const cube = cargo.cubes[i];
    // Bounds margin
    assert(cube.x >= 0.59 && cube.x <= 4.41, `Cube ${i} x out of bounds: ${cube.x}`);
    assert(cube.y >= 0.59 && cube.y <= 4.41, `Cube ${i} y out of bounds: ${cube.y}`);

    // Distance to spawn & goal
    const distSpawn = Math.hypot(cube.x - map.spawn.x, cube.y - map.spawn.y);
    const distGoal = Math.hypot(cube.x - map.goal.x, cube.y - map.goal.y);
    assert(distSpawn >= 0.89, `Cube ${i} too close to spawn: ${distSpawn.toFixed(3)}m < 0.9m`);
    assert(distGoal >= 0.79, `Cube ${i} too close to goal: ${distGoal.toFixed(3)}m < 0.8m`);

    // Obstacle perimeter clearance >= 0.22m
    for (const obs of map.obstacles) {
      const dx = Math.max(obs.x - cube.x, 0, cube.x - (obs.x + obs.w));
      const dy = Math.max(obs.y - cube.y, 0, cube.y - (obs.y + obs.h));
      const distObs = Math.hypot(dx, dy);
      assert(distObs >= 0.21, `Cube ${i} too close to obstacle: ${distObs.toFixed(3)}m < 0.22m`);
    }

    // Inter-cube distance >= 1.0m
    for (let j = i + 1; j < 3; j++) {
      const other = cargo.cubes[j];
      const distCubes = Math.hypot(cube.x - other.x, cube.y - other.y);
      assert(distCubes >= 0.99, `Cubes ${i} and ${j} too close: ${distCubes.toFixed(3)}m < 1.0m`);
    }

    // Reachability
    assert(map.isPointReachable(cube.x, cube.y), `Cube ${i} at (${cube.x.toFixed(2)}, ${cube.y.toFixed(2)}) is not reachable`);
  }
}

// Check route diversity: paths should branch away from diagonal
let diverseBranches = 0;
for (const dev of routeMidpoints) {
  if (dev > 0.3) {
    diverseBranches++;
  }
}
console.log(`Max path deviations: ${routeMidpoints.map(d => d.toFixed(2)).join(", ")}`);
console.log(`Diverse branches: ${diverseBranches}/${routeMidpoints.length}`);
assert(diverseBranches > 0, "Routes lacked branching diversity across seeds");

// Test LOW and HIGH density presets
map.generateMap("LOW", 42);
assert(map.lastGenerationResult.valid);
assert(map.obstacles.length >= 8 && map.obstacles.length <= 12);

map.generateMap("HIGH", 42);
assert(map.lastGenerationResult.valid);
assert(map.obstacles.length >= 18 && map.obstacles.length <= 22);

console.log("All assertions passed successfully! (20 maps verified)");

// ============================================================
// Fog of War & Checkpoint self-checks
// ============================================================
console.log("\nRunning fog-of-war & checkpoint tests...");

// --- Fog: reveal mask ---
const fog = new FogOfWar(5.0, 5.0, 100);
assert.strictEqual(fog.isExplored(0.5, 0.5), false, "Fog should start fully unexplored");
fog.revealDisk(0.5, 0.5, 0.3);
assert(fog.isExplored(0.5, 0.5), "Center of reveal disc must be explored");
assert(!fog.isExplored(4.0, 4.0), "Far corner must stay fogged");
assert(fog.dirty, "Revealing must raise the dirty flag for texture repaint");

// --- Fog: line-of-sight along a ray stops at the hit point ---
fog.reset();
assert.strictEqual(fog.revealedCount, 0, "Reset must clear the mask");
fog.revealFromRays(0.5, 0.5, [{ x: 1.5, y: 0.5 }]);
assert(fog.isExplored(1.0, 0.5), "Cells along the ray up to the hit must be explored");
assert(!fog.isExplored(3.0, 0.5), "Cells beyond the LiDAR hit must remain fogged");

// --- Checkpoints: placed at cube spawn origins, chained reveal ---
cargo.spawnCubes(map);
const cpMgr = new CheckpointManager();
cpMgr.syncToCargo(cargo);
assert.strictEqual(cpMgr.checkpoints.length, 3, "Expected one checkpoint per cargo cube");
for (let i = 0; i < 3; i++) {
  const cp = cpMgr.checkpoints[i];
  assert.strictEqual(cp.x, cargo.cubes[i].spawnX, `CP ${cp.id} x must match cube spawn x`);
  assert.strictEqual(cp.y, cargo.cubes[i].spawnY, `CP ${cp.id} y must match cube spawn y`);
  assert.strictEqual(cp.discovered, false, `CP ${cp.id} must start undiscovered`);
  assert.strictEqual(cp.isNext, i === 0, `Only the first checkpoint starts flagged as next`);
}

const probe = { x: 0, y: 0 };
probe.x = cpMgr.checkpoints[0].x; probe.y = cpMgr.checkpoints[0].y;
let discoveredEvents = 0;
cpMgr.onDiscover = () => { discoveredEvents++; };
cpMgr.update(1 / 60, probe, map);
assert(cpMgr.checkpoints[0].discovered, "CP-1 must be discovered on approach");
assert.strictEqual(cpMgr.revealedTarget.x, cpMgr.checkpoints[1].x, "CP-1 must reveal CP-2 coordinate");

probe.x = cpMgr.checkpoints[1].x; probe.y = cpMgr.checkpoints[1].y;
cpMgr.update(1 / 60, probe, map);
assert(cpMgr.checkpoints[1].discovered, "CP-2 must be discovered on approach");
assert.strictEqual(cpMgr.revealedTarget.x, cpMgr.checkpoints[2].x, "CP-2 must reveal CP-3 coordinate");

probe.x = cpMgr.checkpoints[2].x; probe.y = cpMgr.checkpoints[2].y;
cpMgr.update(1 / 60, probe, map);
assert(cpMgr.checkpoints[2].discovered, "CP-3 must be discovered on approach");
assert.strictEqual(cpMgr.revealedTarget.label, "FINISH", "Last checkpoint must reveal the finish point");
assert.strictEqual(cpMgr.revealedTarget.x, map.goal.x, "Finish target x must equal arena goal");
assert.strictEqual(discoveredEvents, 3, "Exactly three discovery events expected");

// --- Checkpoints: re-sync on respawn clears discovered state ---
cpMgr.syncToCargo(cargo);
assert.strictEqual(cpMgr.revealedTarget, null, "Re-sync must clear the revealed target");
assert(cpMgr.checkpoints.every(c => !c.discovered), "Re-sync must reset all discovered flags");

// --- Checkpoint data exposed to scripts ---
const cpData = cpMgr.getScriptData();
assert.strictEqual(cpData.list.length, 3);
assert.strictEqual(cpData.discoveredCount, 0);
assert.strictEqual(cpData.navTarget, null);

console.log("Fog-of-war & checkpoint assertions passed successfully!");

