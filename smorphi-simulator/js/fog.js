/**
 * Fog of War - Line-of-Sight Exploration Mask
 * Tracks which arena cells the robot has actually seen via its 360° LiDAR.
 * Walls occlude sight, so fog only clears where the beams reach.
 * Pure logic (no THREE/DOM) so it is unit-testable under Node.
 */

class FogOfWar {
  /**
   * @param {number} width - arena width (m)
   * @param {number} height - arena height (m)
   * @param {number} resolution - grid cells per axis (default matches map nav grid)
   */
  constructor(width, height, resolution = 100) {
    this.width = width;
    this.height = height;
    this.N = resolution;
    this.cellSize = width / this.N;
    this.mask = new Uint8Array(this.N * this.N); // 1 = explored
    this.dirty = true;
    this.revealedCount = 0;
  }

  reset() {
    this.mask.fill(0);
    this.dirty = true;
    this.revealedCount = 0;
  }

  _mark(i, j) {
    if (i < 0 || i >= this.N || j < 0 || j >= this.N) return;
    const idx = j * this.N + i;
    if (!this.mask[idx]) {
      this.mask[idx] = 1;
      this.revealedCount++;
      this.dirty = true;
    }
  }

  /** Reveal a filled disc (meters) centered at world (cx, cy). */
  revealDisk(cx, cy, radius) {
    const rCells = radius / this.cellSize;
    const ccx = cx / this.cellSize;
    const ccy = cy / this.cellSize;
    const i0 = Math.floor(ccx - rCells);
    const i1 = Math.ceil(ccx + rCells);
    const j0 = Math.floor(ccy - rCells);
    const j1 = Math.ceil(ccy + rCells);
    const r2 = rCells * rCells;
    for (let j = j0; j <= j1; j++) {
      for (let i = i0; i <= i1; i++) {
        const dx = (i + 0.5) - ccx;
        const dy = (j + 0.5) - ccy;
        if (dx * dx + dy * dy <= r2) this._mark(i, j);
      }
    }
  }

  /**
   * Reveal line-of-sight along LiDAR beams.
   * @param {number} ox - robot x (m)
   * @param {number} oy - robot y (m)
   * @param {Array<{x:number,y:number}>} hitPoints - world-space beam end points
   */
  revealFromRays(ox, oy, hitPoints) {
    if (!hitPoints) return;
    const step = CONFIG.FOG.RAY_STEP;
    for (let k = 0; k < hitPoints.length; k++) {
      const p = hitPoints[k];
      const dx = p.x - ox;
      const dy = p.y - oy;
      const dist = Math.hypot(dx, dy);
      if (dist < 1e-6) continue;
      const ux = dx / dist;
      const uy = dy / dist;
      for (let d = 0; d <= dist; d += step) {
        const i = Math.floor((ox + ux * d) / this.cellSize);
        const j = Math.floor((oy + uy * d) / this.cellSize);
        if (i < 0 || i >= this.N || j < 0 || j >= this.N) break;
        this._mark(i, j);
      }
    }
  }

  /**
   * Reveal from the robot's current pose + sensor suite (beams block at hits).
   */
  reveal(robot, sensors) {
    if (!CONFIG.FOG.ENABLED) return;
    this.revealDisk(robot.x, robot.y, CONFIG.FOG.REVEAL_RADIUS);
    this.revealFromRays(robot.x, robot.y, sensors.lidarPoints);
  }

  /** Is the world point (x, y) explored? */
  isExplored(x, y) {
    const i = Math.floor(x / this.cellSize);
    const j = Math.floor(y / this.cellSize);
    if (i < 0 || i >= this.N || j < 0 || j >= this.N) return false;
    return this.mask[j * this.N + i] === 1;
  }
}

if (typeof module !== "undefined" && module.exports) {
  module.exports = { FogOfWar };
}
