/**
 * Checkpoint System (AprilTag pads)
 * Every cargo cube spawns on top of a floor AprilTag "checkpoint".
 * When the robot approaches a tag with clear line-of-sight it is scanned,
 * which reveals the coordinate of the NEXT checkpoint (or the finish point).
 * Detection + sensor plumbing only - navigation behavior is the team's job.
 */

class CheckpointManager {
  constructor() {
    this.checkpoints = [];
    this.revealedTarget = null; // { x, y, label } | null
    this.onDiscover = null;     // (checkpoint, target) => void
  }

  /**
   * (Re)build checkpoints from the cargo cubes' *spawn origins*.
   * The pad stays on the floor even after the box is dragged away.
   * @param {CargoManager} cargo
   */
  syncToCargo(cargo) {
    this.checkpoints = [];
    this.revealedTarget = null;
    const colors = CONFIG.CARGO.COLORS;
    for (let i = 0; i < cargo.cubes.length; i++) {
      const cube = cargo.cubes[i];
      const c = colors[i % colors.length];
      this.checkpoints.push({
        id: i + 1,
        x: cube.spawnX,
        y: cube.spawnY,
        colorHex: c.hex,
        colorCss: c.css,
        label: `CP-${i + 1}`,
        discovered: false,
        isNext: i === 0,
      });
    }
  }

  /**
   * Scan for undiscovered checkpoints near the robot.
   * @param {number} dt
   * @param {SmorphiRobot} robot
   * @param {ArenaMap} map
   */
  update(dt, robot, map) {
    for (let i = 0; i < this.checkpoints.length; i++) {
      const cp = this.checkpoints[i];
      if (cp.discovered) continue;

      const dist = Math.hypot(robot.x - cp.x, robot.y - cp.y);
      if (dist > CONFIG.CHECKPOINT.SCAN_RADIUS) continue;
      if (!this._hasLineOfSight(robot.x, robot.y, cp.x, cp.y, map)) continue;

      this._discover(i, map);
    }
  }

  _discover(i, map) {
    const cp = this.checkpoints[i];
    cp.discovered = true;

    let target;
    if (i + 1 < this.checkpoints.length) {
      const next = this.checkpoints[i + 1];
      target = { x: next.x, y: next.y, label: next.label };
      for (let k = 0; k < this.checkpoints.length; k++) {
        this.checkpoints[k].isNext = k === i + 1;
      }
    } else {
      target = { x: map.goal.x, y: map.goal.y, label: "FINISH" };
      for (const c of this.checkpoints) c.isNext = false;
    }

    this.revealedTarget = target;
    if (this.onDiscover) this.onDiscover(cp, target);
  }

  _hasLineOfSight(ax, ay, bx, by, map) {
    if (!map || !map.segments) return true;
    for (const s of map.segments) {
      if (this._segmentsIntersect(ax, ay, bx, by, s.p1.x, s.p1.y, s.p2.x, s.p2.y)) {
        return false;
      }
    }
    return true;
  }

  _crossPt(ox, oy, ax, ay, bx, by) {
    return (ax - ox) * (by - oy) - (ay - oy) * (bx - ox);
  }

  _segmentsIntersect(x1, y1, x2, y2, x3, y3, x4, y4) {
    const d1 = this._crossPt(x3, y3, x4, y4, x1, y1);
    const d2 = this._crossPt(x3, y3, x4, y4, x2, y2);
    const d3 = this._crossPt(x1, y1, x2, y2, x3, y3);
    const d4 = this._crossPt(x1, y1, x2, y2, x4, y4);
    return ((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
           ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0));
  }

  /** Data exposed to the injected navigation script via sensors.checkpoints / sensors.navTarget */
  getScriptData() {
    return {
      list: this.checkpoints.map(c => ({
        id: c.id,
        x: c.x,
        y: c.y,
        colorHex: c.colorHex,
        colorCss: c.colorCss,
        label: c.label,
        discovered: c.discovered,
        isNext: c.isNext,
      })),
      navTarget: this.revealedTarget ? { ...this.revealedTarget } : null,
      discoveredCount: this.checkpoints.filter(c => c.discovered).length,
      total: this.checkpoints.length,
    };
  }
}

if (typeof module !== "undefined" && module.exports) {
  module.exports = { CheckpointManager };
}
