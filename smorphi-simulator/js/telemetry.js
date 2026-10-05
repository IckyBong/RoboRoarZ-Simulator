/**
 * Real-Time Telemetry Dashboard & Polar Radar Renderer
 * Platform: Single-Block Smorphi Base Unit with Front Cargo Mesh Scoop
 * Renders high-DPI 360° LiDAR polar plot, IMU compass heading gauge,
 * holonomic velocity bars, Cargo Scoop Payload indicators, and CoM shift gauges.
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
    this.renderLidarRadar(sensorSuite.lidarRanges);
    this.updateIMUDisplay(sensorSuite.imu);
    this.updateVelocityBars(robot.vx, robot.vy, robot.omega);
    this.updatePoseDisplay(robot, sensorSuite.target);
    this.updateCargoDisplay(sensorSuite.cargo, robot);
  }

  /**
   * Render high-DPI 360-degree Polar LiDAR radar with Single-Block robot & scoop glyph
   */
  renderLidarRadar(ranges) {
    if (!this.radarCtx || !ranges) return;
    const ctx = this.radarCtx;
    const S = this.radarDisplaySize;
    const cx = S / 2;
    const cy = S / 2;
    const maxRadarMeters = 4.0;
    const scale = (cx - 14) / maxRadarMeters;

    // Clear background
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
        ctx.strokeStyle = "rgba(239, 68, 68, 0.4)";
        ctx.setLineDash([4, 4]);
        ctx.stroke();
        ctx.setLineDash([]);
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

    // 2. Crosshairs
    ctx.strokeStyle = "rgba(100, 116, 139, 0.2)";
    ctx.beginPath();
    ctx.moveTo(cx, 10);
    ctx.lineTo(cx, S - 10);
    ctx.moveTo(10, cy);
    ctx.lineTo(S - 10, cy);
    ctx.stroke();

    // 3. Render 360 LiDAR Points
    for (let i = 0; i < CONFIG.LIDAR.NUM_BEAMS; i += 2) {
      const dist = ranges[i];
      if (dist === undefined) continue;

      const angleRad = (i * Math.PI) / 180;
      // 0° is FRONT (-Y in radar), 90° is LEFT (-X in radar)
      const px = cx - Math.sin(angleRad) * dist * scale;
      const py = cy - Math.cos(angleRad) * dist * scale;

      let color;
      if (dist < 0.6) {
        color = "#ef4444";
      } else if (dist < 1.2) {
        color = "#f59e0b";
      } else {
        color = "#06b6d4";
      }

      ctx.fillStyle = color;
      ctx.fillRect(px - 1, py - 1, 2.5, 2.5);
    }

    // 4. Center Single-Block Robot Glyph with Front Scoop
    // Robot is 170x170mm (scale to radar pixels: 0.17 * scale)
    const robW = 0.17 * scale;
    const robL = 0.17 * scale;

    ctx.fillStyle = "#1e293b";
    ctx.strokeStyle = "#38bdf8";
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.rect(cx - robW / 2, cy - robL / 2, robW, robL);
    ctx.fill();
    ctx.stroke();

    // Front Scoop U-Shape outline at top of robot (-Y in radar)
    const scoopLen = 0.14 * scale;
    const scoopW = 0.19 * scale;
    ctx.strokeStyle = "#06b6d4";
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(cx - scoopW / 2, cy - robL / 2);
    ctx.lineTo(cx - scoopW / 2, cy - robL / 2 - scoopLen);
    ctx.lineTo(cx - scoopW / 2 + 3, cy - robL / 2 - scoopLen);
    ctx.moveTo(cx + scoopW / 2, cy - robL / 2);
    ctx.lineTo(cx + scoopW / 2, cy - robL / 2 - scoopLen);
    ctx.lineTo(cx + scoopW / 2 - 3, cy - robL / 2 - scoopLen);
    ctx.stroke();

    // Forward Direction Arrow
    ctx.strokeStyle = "#f59e0b";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(cx, cy - robL / 2 - scoopLen - 4);
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
   * Update Global Pose and Goal Distance (Passive Target)
   */
  updatePoseDisplay(robot, target) {
    if (this.dom.poseX) this.dom.poseX.textContent = `${robot.x.toFixed(2)} m`;
    if (this.dom.poseY) this.dom.poseY.textContent = `${robot.y.toFixed(2)} m`;
    if (this.dom.poseTheta) this.dom.poseTheta.textContent = `${(robot.theta * 180 / Math.PI).toFixed(0)}°`;
    if (this.dom.distGoal) {
      this.dom.distGoal.textContent = `${target.distance.toFixed(2)} m`;
      if (target.reached) {
        this.dom.distGoal.innerHTML = `<span class="text-cyan-400 font-bold">AT GOAL AREA (${target.distance.toFixed(2)} m)</span>`;
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
   * Update Cargo Scoop Payload Monitor & CoM Shift Gauges
   * @param {object} cargo
   * @param {SmorphiRobot} robot
   */
  updateCargoDisplay(cargo, robot) {
    if (!cargo) return;

    // 1. Update Top-Right Active Badge
    if (this.dom.activeShapeBadge) {
      const count = cargo.count || 0;
      const totalMass = (cargo.totalMassKg || 0).toFixed(2);
      if (count === 3) {
        this.dom.activeShapeBadge.textContent = `CARGO FULL: 3/3 (${totalMass} kg)`;
        this.dom.activeShapeBadge.className = "px-2.5 py-1 text-xs font-mono font-bold rounded-md bg-emerald-500/20 text-emerald-400 border border-emerald-500/40 animate-pulse";
      } else if (count > 0) {
        this.dom.activeShapeBadge.textContent = `CARGO: ${count}/3 (${totalMass} kg)`;
        this.dom.activeShapeBadge.className = "px-2.5 py-1 text-xs font-mono font-bold rounded-md bg-amber-500/20 text-amber-400 border border-amber-500/40";
      } else {
        this.dom.activeShapeBadge.textContent = `SCOOP EMPTY (0/3)`;
        this.dom.activeShapeBadge.className = "px-2.5 py-1 text-xs font-mono font-bold rounded-md bg-slate-800 text-slate-400 border border-slate-700";
      }
    }

    // 2. Update 3-Slot Cargo Scoop Preview Widget
    if (this.dom.shapePreviewGrid) {
      const cubes = cargo.cubes || [];
      const comShiftMm = ((robot.comOffsetX || 0) * 1000).toFixed(1);
      const tractionPct = Math.round((robot.tractionMultiplier || 1.0) * 100);

      let html = `
        <div class="flex flex-col gap-1.5 p-2 bg-slate-900 border border-slate-700 rounded-lg w-44 text-[10px] font-mono">
          <div class="flex justify-between items-center text-slate-300 font-semibold border-b border-slate-800 pb-1">
            <span>MESH SCOOP</span>
            <span class="${cargo.hasContact ? 'text-emerald-400' : 'text-slate-500'}">${cargo.hasContact ? 'CONTACT' : 'CLEAR'}</span>
          </div>

          <!-- 3 Cargo Slot Boxes -->
          <div class="grid grid-cols-3 gap-1 py-0.5">
      `;

      const slotColors = ["#f59e0b", "#06b6d4", "#10b981"];
      const slotNames = ["Amber", "Cyan", "Emerald"];

      for (let s = 0; s < 3; s++) {
        const cube = cubes[s];
        const isCaptured = cube && cube.state === "CAPTURED_INSIDE_MESH";
        const color = slotColors[s];
        const name = slotNames[s];

        if (isCaptured) {
          html += `
            <div class="flex flex-col items-center justify-center p-1 rounded border border-cyan-400/50" style="background-color: ${color}25">
              <span class="w-3 h-3 rounded-sm shadow-sm" style="background-color: ${color}"></span>
              <span class="text-[9px] mt-0.5 font-bold" style="color: ${color}">#0${s+1}</span>
            </div>
          `;
        } else {
          html += `
            <div class="flex flex-col items-center justify-center p-1 rounded bg-slate-950/60 border border-slate-800 text-slate-600">
              <span class="w-3 h-3 rounded-sm border border-dashed border-slate-700"></span>
              <span class="text-[9px] mt-0.5">SLOT ${s+1}</span>
            </div>
          `;
        }
      }

      html += `
          </div>

          <!-- CoM & Traction Metrics -->
          <div class="flex justify-between text-slate-400 pt-0.5 border-t border-slate-800/80">
            <span>CoM Shift:</span>
            <span class="text-cyan-400 font-bold">+${comShiftMm} mm</span>
          </div>
          <div class="flex justify-between text-slate-400">
            <span>Traction:</span>
            <span class="${tractionPct < 80 ? 'text-amber-400' : 'text-emerald-400'} font-bold">${tractionPct}%</span>
          </div>
        </div>
      `;

      this.dom.shapePreviewGrid.innerHTML = html;
    }
  }
}

if (typeof module !== "undefined" && module.exports) {
  module.exports = TelemetryDashboard;
}
