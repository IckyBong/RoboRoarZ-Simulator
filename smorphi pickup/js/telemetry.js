/**
 * Real-Time Telemetry Dashboard & Polar Radar Renderer
 * Renders high-DPI 360° LiDAR polar plot, IMU compass heading gauge,
 * holonomic velocity bars, and morphology shape indicators.
 */

class TelemetryDashboard {
  constructor(radarCanvasId) {
    this.radarCanvas = document.getElementById(radarCanvasId);
    this.radarCtx = this.radarCanvas ? this.radarCanvas.getContext("2d") : null;

    // Elements
    this.dom = {
      imuHeading: document.getElementById("telemetry-heading"),
      imuHeadingRad: document.getElementById("telemetry-heading-rad"),
      imuYawRate: document.getElementById("telemetry-yaw-rate"),
      imuCompassNeedle: document.getElementById("compass-needle"),
      vxBar: document.getElementById("vx-bar"),
      vyBar: document.getElementById("vy-bar"),
      omegaBar: document.getElementById("omega-bar"),
      vxVal: document.getElementById("vx-val"),
      vyVal: document.getElementById("vy-val"),
      omegaVal: document.getElementById("omega-val"),
      poseX: document.getElementById("pose-x"),
      poseY: document.getElementById("pose-y"),
      poseTheta: document.getElementById("pose-theta"),
      distGoal: document.getElementById("dist-goal"),
      collisionCount: document.getElementById("collision-count"),
      activeShapeBadge: document.getElementById("active-shape-badge"),
      shapePreviewGrid: document.getElementById("shape-preview-grid"),
    };

    this.setupRadarCanvas();
  }

  setupRadarCanvas() {
    if (!this.radarCanvas) return;
    const dpr = window.devicePixelRatio || 1;
    const rect = this.radarCanvas.getBoundingClientRect();
    const size = (rect.width > 0 && rect.width < 400) ? rect.width : 260;

    this.radarCanvas.width = size * dpr;
    this.radarCanvas.height = size * dpr;
    if (this.radarCtx) {
      this.radarCtx.setTransform(1, 0, 0, 1, 0, 0);
      this.radarCtx.scale(dpr, dpr);
    }
    this.radarDisplaySize = size;
  }

  /**
   * Update all telemetry components on each animation frame
   */
  update(robot, sensorSuite, arenaMap) {
    this.renderLidarRadar(sensorSuite.lidarRanges, robot.currentShape);
    this.updateIMUDisplay(sensorSuite.imu);
    this.updateVelocityBars(robot.vx, robot.vy, robot.omega);
    this.updatePoseDisplay(robot, sensorSuite.target);
    this.updateShapeDisplay(robot.currentShape, robot.isMorphing);
  }

  /**
   * Render high-DPI 360-degree Polar LiDAR radar
   */
  renderLidarRadar(ranges, currentShape) {
    if (!this.radarCtx || !ranges) return;
    const ctx = this.radarCtx;
    const S = this.radarDisplaySize;
    const cx = S / 2;
    const cy = S / 2;
    const maxRadarMeters = 4.0; // Display range up to 4.0m
    const scale = (cx - 14) / maxRadarMeters;

    // Clear background with subtle persistence
    ctx.fillStyle = "#090d16";
    ctx.fillRect(0, 0, S, S);

    // 1. Concentric Distance Rings
    const rings = [0.6, 1.5, 3.0];
    ctx.lineWidth = 1;

    for (const r of rings) {
      const radius = r * scale;
      ctx.beginPath();
      ctx.arc(cx, cy, radius, 0, Math.PI * 2);

      if (r === 0.6) {
        // Warning threshold ring (0.6m)
        ctx.strokeStyle = "rgba(239, 68, 68, 0.4)";
        ctx.setLineDash([4, 4]);
        ctx.stroke();
        ctx.setLineDash([]);
        // Label
        ctx.fillStyle = "rgba(239, 68, 68, 0.8)";
        ctx.font = "9px monospace";
        ctx.fillText("0.6m", cx + radius - 24, cy - 3);
      } else {
        ctx.strokeStyle = "rgba(100, 116, 139, 0.25)";
        ctx.stroke();
        ctx.fillStyle = "rgba(148, 163, 184, 0.5)";
        ctx.font = "9px monospace";
        ctx.fillText(`${r}m`, cx + radius - 18, cy - 3);
      }
    }

    // 2. Crosshairs & Angles
    ctx.strokeStyle = "rgba(100, 116, 139, 0.2)";
    ctx.beginPath();
    ctx.moveTo(cx, 10);
    ctx.lineTo(cx, S - 10);
    ctx.moveTo(10, cy);
    ctx.lineTo(S - 10, cy);
    ctx.stroke();

    // 3. Render 360 LiDAR Beams / Point Cloud
    // In robot frame: 0° is FRONT (Top in radar), 90° is LEFT, 270° is RIGHT
    ctx.beginPath();
    let first = true;

    for (let i = 0; i < CONFIG.LIDAR.NUM_BEAMS; i += 2) {
      const dist = ranges[i];
      if (dist === undefined) continue;

      // Transform robot angle (0 = front/up, 90 = left) to canvas coordinates
      const angleRad = (i * Math.PI) / 180;
      // Front is -Y, Left is -X
      const px = cx - Math.sin(angleRad) * dist * scale;
      const py = cy - Math.cos(angleRad) * dist * scale;

      // Draw point
      let color;
      if (dist < 0.6) {
        color = "#ef4444"; // Red Danger
      } else if (dist < 1.2) {
        color = "#f59e0b"; // Amber Warning
      } else {
        color = "#06b6d4"; // Cyan Safe
      }

      ctx.fillStyle = color;
      ctx.fillRect(px - 1, py - 1, 2.5, 2.5);
    }

    // 4. Center Robot Glyph
    ctx.fillStyle = "#38bdf8";
    ctx.beginPath();
    ctx.arc(cx, cy, 4, 0, Math.PI * 2);
    ctx.fill();

    // Forward direction indicator arrow
    ctx.strokeStyle = "#38bdf8";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(cx, cy - 12);
    ctx.stroke();
  }

  /**
   * Update IMU Heading Compass & Yaw Rate
   */
  updateIMUDisplay(imu) {
    if (this.dom.imuHeading) {
      this.dom.imuHeading.textContent = `${imu.heading.toFixed(1)}°`;
    }
    if (this.dom.imuHeadingRad) {
      this.dom.imuHeadingRad.textContent = `${imu.headingRad.toFixed(2)} rad`;
    }
    if (this.dom.imuYawRate) {
      this.dom.imuYawRate.textContent = `${imu.yaw_rate.toFixed(1)}°/s`;
    }
    if (this.dom.imuCompassNeedle) {
      // Rotate needle according to heading
      this.dom.imuCompassNeedle.style.transform = `rotate(${imu.heading}deg)`;
    }
  }

  /**
   * Update Holonomic Velocities (Vx, Vy, Omega)
   */
  updateVelocityBars(vx, vy, omega) {
    const maxLin = CONFIG.ROBOT.MAX_LINEAR_SPEED;
    const maxAng = CONFIG.ROBOT.MAX_ANGULAR_SPEED;

    if (this.dom.vxVal) this.dom.vxVal.textContent = `${vx >= 0 ? "+" : ""}${vx.toFixed(2)} m/s`;
    if (this.dom.vyVal) this.dom.vyVal.textContent = `${vy >= 0 ? "+" : ""}${vy.toFixed(2)} m/s`;
    if (this.dom.omegaVal) this.dom.omegaVal.textContent = `${omega >= 0 ? "+" : ""}${omega.toFixed(2)} rad/s`;

    if (this.dom.vxBar) {
      const pct = Math.min(100, Math.abs(vx) / maxLin * 100);
      this.dom.vxBar.style.width = `${pct}%`;
      this.dom.vxBar.className = `h-full rounded-full transition-all ${vx >= 0 ? "bg-cyan-500" : "bg-amber-500"}`;
    }
    if (this.dom.vyBar) {
      const pct = Math.min(100, Math.abs(vy) / maxLin * 100);
      this.dom.vyBar.style.width = `${pct}%`;
      this.dom.vyBar.className = `h-full rounded-full transition-all ${vy >= 0 ? "bg-purple-500" : "bg-pink-500"}`;
    }
    if (this.dom.omegaBar) {
      const pct = Math.min(100, Math.abs(omega) / maxAng * 100);
      this.dom.omegaBar.style.width = `${pct}%`;
      this.dom.omegaBar.className = `h-full rounded-full transition-all ${omega >= 0 ? "bg-emerald-500" : "bg-orange-500"}`;
    }
  }

  /**
   * Update Global Pose and Goal Distance
   */
  updatePoseDisplay(robot, target) {
    if (this.dom.poseX) this.dom.poseX.textContent = `${robot.x.toFixed(2)} m`;
    if (this.dom.poseY) this.dom.poseY.textContent = `${robot.y.toFixed(2)} m`;
    if (this.dom.poseTheta) this.dom.poseTheta.textContent = `${(robot.theta * 180 / Math.PI).toFixed(0)}°`;
    if (this.dom.distGoal) {
      this.dom.distGoal.textContent = `${target.distance.toFixed(2)} m`;
      if (target.reached) {
        this.dom.distGoal.innerHTML = `<span class="text-emerald-400 font-bold animate-pulse">REACHED!</span>`;
      }
    }
    if (this.dom.collisionCount) {
      this.dom.collisionCount.textContent = robot.collisionCount;
      if (robot.inCollision) {
        this.dom.collisionCount.parentElement.classList.add("bg-red-950/40", "border-red-500/50");
      } else {
        this.dom.collisionCount.parentElement.classList.remove("bg-red-950/40", "border-red-500/50");
      }
    }
  }

  /**
   * Update Morphology Preview Badge & Mini Tetromino Visualizer
   */
  updateShapeDisplay(shape, isMorphing) {
    if (this.dom.activeShapeBadge) {
      if (isMorphing) {
        this.dom.activeShapeBadge.textContent = `MORPHING...`;
        this.dom.activeShapeBadge.className = "px-2.5 py-1 text-xs font-mono font-bold rounded-md bg-amber-500/20 text-amber-400 border border-amber-500/40 animate-pulse";
      } else {
        this.dom.activeShapeBadge.textContent = `SHAPE: ${shape}`;
        this.dom.activeShapeBadge.className = "px-2.5 py-1 text-xs font-mono font-bold rounded-md bg-cyan-500/20 text-cyan-400 border border-cyan-500/40";
      }
    }

    if (this.dom.shapePreviewGrid) {
      this.renderMiniShapePreview(shape);
    }
  }

  /**
   * Render 4x4 mini grid showing module blocks configuration
   */
  renderMiniShapePreview(shape) {
    const coords = CONFIG.SHAPES[shape] || CONFIG.SHAPES["O"];
    let html = '<div class="grid grid-cols-4 gap-1 w-20 h-20 p-1.5 bg-slate-900 border border-slate-700 rounded-lg">';

    // Map each module (offset in units of 0.16m) to 4x4 grid (row 0-3, col 0-3)
    const occupied = new Set();
    for (const mod of coords) {
      // x is forward/up, y is left
      const r = Math.round(1.5 - mod.x / 0.16);
      const c = Math.round(1.5 + mod.y / 0.16);
      if (r >= 0 && r < 4 && c >= 0 && c < 4) {
        occupied.add(`${r},${c}`);
      }
    }

    for (let r = 0; r < 4; r++) {
      for (let c = 0; c < 4; c++) {
        const isBlock = occupied.has(`${r},${c}`);
        if (isBlock) {
          html += `<div class="bg-cyan-500 rounded-sm shadow-sm border border-cyan-300/40"></div>`;
        } else {
          html += `<div class="bg-slate-800/40 rounded-sm"></div>`;
        }
      }
    }

    html += '</div>';
    this.dom.shapePreviewGrid.innerHTML = html;
  }
}

if (typeof module !== "undefined" && module.exports) {
  module.exports = TelemetryDashboard;
}
