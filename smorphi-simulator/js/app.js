/**
 * Smorphi Robotics Simulator - Main Application Controller
 * Platform: Single-Block Smorphi Base Unit with Front Cargo Mesh Scoop
 * Coordinates physics, 3D viewport, sensor raycasting, CargoManager,
 * Code Editor, Telemetry HUD, audio synthesis, Finish Point state & Replay mechanism.
 */

class SmorphiApp {
  constructor() {
    // Simulator State
    this.isSimRunning = true;
    this.simSpeed = 1.0;
    this.manualControlEnabled = false;
    this.keysDown = {};
    this.audioEnabled = true;

    // Mission State & Finish Management
    this.missionState = "RUNNING"; // "RUNNING" | "FINISHED"
    this.missionTime = 0.0;
    this.roundNumber = 1;
    this.autoRepeatEnabled = false;
    this.autoRepeatTimer = null;
    this.autoRepeatCountdown = 5;

    // Timing
    this.lastFrameTime = performance.now();
    this.simTime = 0.0;
    this.frameCount = 0;
    this.fps = 60;
    this.lastFpsUpdate = performance.now();

    // Instantiate Core Subsystems
    this.map = new ArenaMap(CONFIG.ARENA.WIDTH, CONFIG.ARENA.HEIGHT);
    this.cargo = new CargoManager();
    this.cargo.spawnCubes(this.map);

    // Fog of War + Checkpoints (built before the 3D viewport renders them)
    this.fog = new FogOfWar(this.map.width, this.map.height, CONFIG.MAP_NAVIGATION.gridResolution);
    this.fog.revealDisk(this.map.spawn.x, this.map.spawn.y, CONFIG.FOG.START_BLOB);
    this.checkpoints = new CheckpointManager();
    this.checkpoints.syncToCargo(this.cargo);

    this.robot = new SmorphiRobot(this.map.spawn.x, this.map.spawn.y, 0);
    this.physics = new PhysicsEngine(this.map);
    this.sensors = new SensorSuite(this.map);
    this.codeEngine = new CodeEngine();

    // DOM & Viewports
    this.view3d = new Viewport3D("viewport-3d", this.map, this.fog, this.checkpoints);
    this.telemetry = new TelemetryDashboard("lidar-radar-canvas");

    // Audio Synthesizer (Web Audio API)
    this.audioCtx = null;

    // Code Editor
    this.editor = null;

    this.init();
  }

  init() {
    this.setupCodeEditor();
    this.setupConsoleLogger();
    this.setupUIEventListeners();
    this.setupKeyboardListeners();

    // Initial sensor scan & telemetry update
    this.sensors.update(this.robot, this.cargo, { state: this.missionState, time: this.missionTime, round: this.roundNumber });
    this.telemetry.update(this.robot, this.sensors, this.map);
    this.fog.reveal(this.robot, this.sensors);

    // Checkpoint discovery feedback
    this.checkpoints.onDiscover = (cp, target) => {
      const tx = target.x.toFixed(2);
      const ty = target.y.toFixed(2);
      this.codeEngine.log(`[CHECKPOINT] ${cp.label} terpindai! Target berikutnya: ${target.label} @ (${tx}, ${ty})`);
      this.playBeep(880, 0.14, "sine");
    };

    // Initial code compilation of default template
    if (this.editor) {
      this.codeEngine.compileScript(this.editor.getValue());
    }

    // Start Simulation Loop
    requestAnimationFrame((t) => this.simulationLoop(t));
    console.log("[SmorphiApp] Single-Block Simulator initialized successfully.");
  }

  /**
   * Setup Ace Editor for interactive JavaScript coding
   */
  setupCodeEditor() {
    if (typeof ace !== "undefined") {
      this.editor = ace.edit("code-editor");
      this.editor.setTheme("ace/theme/monokai");
      this.editor.session.setMode("ace/mode/javascript");
      this.editor.setOptions({
        fontSize: "13px",
        showPrintMargin: false,
        wrap: true,
        enableBasicAutocompletion: true,
        enableLiveAutocompletion: true,
        tabSize: 2,
      });

      // Set default script from codeEngine
      this.editor.setValue(this.codeEngine.presets.default_avoidance, -1);
    } else {
      console.warn("[SmorphiApp] Ace Editor not loaded, falling back to textarea");
    }
  }

  /**
   * Hook simulator console logging to UI console tab
   */
  setupConsoleLogger() {
    const consoleLogsContainer = document.getElementById("console-logs");
    if (!consoleLogsContainer) return;

    this.codeEngine.onLog((entry) => {
      const line = document.createElement("div");
      line.className = "text-xs font-mono py-0.5 leading-relaxed flex items-start gap-2";

      let colorClass = "text-slate-300";
      let badge = "INFO";
      let badgeClass = "bg-slate-700/60 text-slate-300";

      if (entry.type === "error") {
        colorClass = "text-red-400 font-semibold";
        badge = "ERR";
        badgeClass = "bg-red-500/20 text-red-400 border border-red-500/40";
        this.playBeep(240, 0.2, "sawtooth");
      } else if (entry.type === "user") {
        colorClass = "text-cyan-300";
        badge = "LOG";
        badgeClass = "bg-cyan-500/20 text-cyan-400 border border-cyan-500/40";
      }

      line.innerHTML = `
        <span class="text-slate-500 select-none">${entry.time}</span>
        <span class="px-1 rounded text-[10px] ${badgeClass}">${badge}</span>
        <span class="${colorClass}">${this.escapeHtml(entry.text)}</span>
      `;

      consoleLogsContainer.appendChild(line);

      while (consoleLogsContainer.childNodes.length > 150) {
        consoleLogsContainer.removeChild(consoleLogsContainer.firstChild);
      }

      consoleLogsContainer.scrollTop = consoleLogsContainer.scrollHeight;
    });
  }

  escapeHtml(str) {
    return str.replace(/[&<>'"]/g, 
      tag => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', "'": '&#39;', '"': '&quot;' }[tag] || tag)
    );
  }

  /**
   * Wire buttons, sliders, camera switches, modal actions, and presets
   */
  setupUIEventListeners() {
    // 1. Code Editor Actions
    const btnRun = document.getElementById("btn-run-code");
    const btnStop = document.getElementById("btn-stop-code");
    const btnResetCode = document.getElementById("btn-reset-code");
    const presetSelect = document.getElementById("code-preset-select");
    const btnClearConsole = document.getElementById("btn-clear-console");

    if (btnRun) {
      btnRun.addEventListener("click", () => {
        this.initAudio();
        const code = this.editor ? this.editor.getValue() : "";
        const res = this.codeEngine.compileScript(code);
        if (res.success) {
          this.codeEngine.start();
          this.manualControlEnabled = false;
          if (this.missionState === "FINISHED") {
            this.restartSimulation(true);
          }
          this.updateModeIndicator("AUTONOMOUS");
          this.playBeep(650, 0.1, "sine");
        }
      });
    }

    if (btnStop) {
      btnStop.addEventListener("click", () => {
        this.codeEngine.stop();
        this.robot.stop();
        this.updateModeIndicator("STOPPED");
      });
    }

    if (btnResetCode) {
      btnResetCode.addEventListener("click", () => {
        const val = presetSelect ? presetSelect.value : "default_avoidance";
        if (this.editor && this.codeEngine.presets[val]) {
          this.editor.setValue(this.codeEngine.presets[val], -1);
          this.codeEngine.log("Editor reset to selected template.");
        }
      });
    }

    if (presetSelect) {
      presetSelect.addEventListener("change", (e) => {
        const key = e.target.value;
        if (this.editor && this.codeEngine.presets[key]) {
          this.editor.setValue(this.codeEngine.presets[key], -1);
          this.codeEngine.log(`Loaded template: ${key}`);
        }
      });
    }

    if (btnClearConsole) {
      btnClearConsole.addEventListener("click", () => {
        const consoleLogsContainer = document.getElementById("console-logs");
        if (consoleLogsContainer) consoleLogsContainer.innerHTML = "";
      });
    }

    // 2. Map Generation & Simulation Controls
    const btnNewMap = document.getElementById("btn-new-map");
    const densitySelect = document.getElementById("map-density-select");
    const btnResetRobot = document.getElementById("btn-reset-robot");
    const btnSimPlayPause = document.getElementById("btn-sim-play-pause");
    const btnSimStep = document.getElementById("btn-sim-step");
    const simSpeedSelect = document.getElementById("sim-speed-select");

    if (btnNewMap) {
      btnNewMap.addEventListener("click", () => {
        this.restartSimulation(false);
      });
    }

    if (densitySelect) {
      densitySelect.addEventListener("change", () => {
        this.restartSimulation(false);
      });
    }

    if (btnResetRobot) {
      btnResetRobot.addEventListener("click", () => {
        this.restartSimulation(true);
      });
    }

    if (btnSimPlayPause) {
      btnSimPlayPause.addEventListener("click", () => {
        this.isSimRunning = !this.isSimRunning;
        btnSimPlayPause.innerHTML = this.isSimRunning
          ? `<i data-lucide="pause" class="w-4 h-4"></i> Pause`
          : `<i data-lucide="play" class="w-4 h-4"></i> Play`;
        if (window.lucide) lucide.createIcons();
      });
    }

    if (btnSimStep) {
      btnSimStep.addEventListener("click", () => {
        this.stepSimulation(CONFIG.SIM.DT);
      });
    }

    if (simSpeedSelect) {
      simSpeedSelect.addEventListener("change", (e) => {
        this.simSpeed = parseFloat(e.target.value) || 1.0;
      });
    }

    // 3. Camera View Switches
    const camButtons = document.querySelectorAll("[data-cam-mode]");
    camButtons.forEach((btn) => {
      btn.addEventListener("click", () => {
        const mode = btn.getAttribute("data-cam-mode");
        this.view3d.setCameraMode(mode);
        camButtons.forEach((b) => b.classList.remove("bg-cyan-500", "text-white"));
        btn.classList.add("bg-cyan-500", "text-white");
      });
    });

    // 4. Visual Toggles (LiDAR Beams, Trail, Sound)
    const toggleLidar = document.getElementById("toggle-lidar-beams");
    if (toggleLidar) {
      toggleLidar.addEventListener("change", (e) => {
        this.view3d.showLidarBeams = e.target.checked;
      });
    }

    const toggleTrail = document.getElementById("toggle-trajectory-trail");
    if (toggleTrail) {
      toggleTrail.addEventListener("change", (e) => {
        this.view3d.showTrajectory = e.target.checked;
      });
    }

    const toggleAudio = document.getElementById("toggle-audio-sfx");
    if (toggleAudio) {
      toggleAudio.addEventListener("change", (e) => {
        this.audioEnabled = e.target.checked;
      });
    }

    const toggleFog = document.getElementById("toggle-fog-of-war");
    if (toggleFog) {
      toggleFog.addEventListener("change", (e) => {
        CONFIG.FOG.ENABLED = e.target.checked;
      });
    }

    // 5. Manual Teleop Mode Toggle
    const btnTeleop = document.getElementById("btn-manual-teleop");
    if (btnTeleop) {
      btnTeleop.addEventListener("click", () => {
        this.initAudio();
        this.manualControlEnabled = !this.manualControlEnabled;
        if (this.manualControlEnabled) {
          this.codeEngine.stop();
          this.updateModeIndicator("MANUAL (WASD)");
          btnTeleop.classList.add("bg-amber-600", "text-white");
          btnTeleop.classList.remove("bg-slate-800", "text-amber-300");
          this.codeEngine.log("Manual keyboard teleoperation enabled (WASD + Q/E).");
        } else {
          this.robot.stop();
          this.updateModeIndicator("IDLE");
          btnTeleop.classList.remove("bg-amber-600", "text-white");
          btnTeleop.classList.add("bg-slate-800", "text-amber-300");
        }
      });
    }

    // 6. Quick Respawn Cargo Cubes Button
    const btnRespawnCubes = document.getElementById("btn-respawn-cubes");
    if (btnRespawnCubes) {
      btnRespawnCubes.addEventListener("click", () => {
        this.cargo.spawnCubes(this.map);
        this.resetExploration();
        this.codeEngine.log("Cargo Cubes respawned at safe waypoints.");
        this.playBeep(520, 0.1, "sine");
      });
    }

    // 7. Finish Modal Action Buttons (Repeat / Next Round)
    const btnModalRestart = document.getElementById("btn-modal-restart");
    if (btnModalRestart) {
      btnModalRestart.addEventListener("click", () => {
        this.restartSimulation(true);
      });
    }

    const btnModalNextRound = document.getElementById("btn-modal-next-round");
    if (btnModalNextRound) {
      btnModalNextRound.addEventListener("click", () => {
        this.restartSimulation(false);
      });
    }

    const btnModalClose = document.getElementById("btn-modal-close");
    if (btnModalClose) {
      btnModalClose.addEventListener("click", () => {
        this.clearAutoRepeatTimer();
        const modal = document.getElementById("finish-modal");
        if (modal) modal.classList.add("hidden");
      });
    }

    const toggleAutoRepeat = document.getElementById("toggle-auto-repeat");
    if (toggleAutoRepeat) {
      toggleAutoRepeat.addEventListener("change", (e) => {
        this.autoRepeatEnabled = e.target.checked;
        if (this.missionState === "FINISHED") {
          if (this.autoRepeatEnabled) {
            this.startAutoRepeatCountdown();
          } else {
            this.clearAutoRepeatTimer();
          }
        }
      });
    }
  }

  /**
   * Listen to Keyboard events for 3-DOF Holonomic Manual Driving
   */
  setupKeyboardListeners() {
    window.addEventListener("keydown", (e) => {
      this.keysDown[e.key.toLowerCase()] = true;

      if (e.code === "Space") {
        this.robot.stop();
        if (this.codeEngine.isRunning) {
          this.codeEngine.stop();
          this.updateModeIndicator("STOPPED");
        }
      }
    });

    window.addEventListener("keyup", (e) => {
      this.keysDown[e.key.toLowerCase()] = false;
    });
  }

  /**
   * Process WASD + Q/E Keyboard Inputs for 3-DOF Holonomic Control
   */
  processManualInput() {
    if (!this.manualControlEnabled) return;

    let vx = 0;
    let vy = 0;
    let omega = 0;
    const maxLin = CONFIG.ROBOT.MAX_LINEAR_SPEED;
    const maxAng = CONFIG.ROBOT.MAX_ANGULAR_SPEED;

    if (this.keysDown["w"] || this.keysDown["arrowup"]) vx += maxLin;
    if (this.keysDown["s"] || this.keysDown["arrowdown"]) vx -= maxLin;

    if (this.keysDown["a"] || this.keysDown["arrowleft"]) vy += maxLin;
    if (this.keysDown["d"] || this.keysDown["arrowright"]) vy -= maxLin;

    if (this.keysDown["q"]) omega += maxAng;
    if (this.keysDown["e"]) omega -= maxAng;

    this.robot.setVelocity(vx, vy, omega);
  }

  updateModeIndicator(modeText) {
    const el = document.getElementById("sim-status-mode");
    if (!el) return;
    el.textContent = modeText;

    if (modeText === "AUTONOMOUS") {
      el.className = "px-2.5 py-1 text-xs font-mono font-bold rounded-md bg-emerald-500/20 text-emerald-400 border border-emerald-500/40 animate-pulse";
    } else if (modeText === "MANUAL (WASD)") {
      el.className = "px-2.5 py-1 text-xs font-mono font-bold rounded-md bg-amber-500/20 text-amber-400 border border-amber-500/40";
    } else if (modeText.includes("FINISH") || modeText.includes("REACHED")) {
      el.className = "px-2.5 py-1 text-xs font-mono font-bold rounded-md bg-emerald-500 text-white shadow-lg shadow-emerald-500/40 animate-pulse";
    } else if (modeText.includes("ERROR")) {
      el.className = "px-2.5 py-1 text-xs font-mono font-bold rounded-md bg-red-500/20 text-red-400 border border-red-500/40";
    } else {
      el.className = "px-2.5 py-1 text-xs font-mono font-bold rounded-md bg-slate-800 text-slate-400 border border-slate-700";
    }
  }

  /**
   * Main 60 FPS RequestAnimationFrame Loop
   */
  simulationLoop(currentTime) {
    requestAnimationFrame((t) => this.simulationLoop(t));

    const dt = Math.min((currentTime - this.lastFrameTime) / 1000, 0.1);
    this.lastFrameTime = currentTime;

    // FPS Meter
    this.frameCount++;
    if (currentTime - this.lastFpsUpdate > 500) {
      this.fps = Math.round((this.frameCount * 1000) / (currentTime - this.lastFpsUpdate));
      this.frameCount = 0;
      this.lastFpsUpdate = currentTime;
      const fpsEl = document.getElementById("sim-fps");
      if (fpsEl) fpsEl.textContent = `${this.fps} FPS`;
    }

    if (this.isSimRunning) {
      const scaledDt = dt * this.simSpeed;
      this.stepSimulation(scaledDt);
    }

    // Render 3D View and Telemetry
    const isGoalReached = this.missionState === "FINISHED" || (this.sensors.target && this.sensors.target.reached);
    this.view3d.update(this.robot, this.sensors, dt, this.cargo, isGoalReached);
    this.telemetry.update(this.robot, this.sensors, this.map);
  }

  /**
   * Step physics, execute code, update sensors, and check finish condition
   */
  stepSimulation(dt) {
    this.simTime += dt;

    if (this.missionState === "RUNNING") {
      this.missionTime += dt;
    }

    // 1. Process Manual Keyboard input (if enabled)
    this.processManualInput();

    // 2. Read Sensors before script execution
    this.sensors.update(this.robot, this.cargo, { state: this.missionState, time: this.missionTime, round: this.roundNumber });

    // 2b. Fog exploration + checkpoint scanning
    this.fog.reveal(this.robot, this.sensors);
    this.checkpoints.update(dt, this.robot, this.map);

    // 3. Execute Autonomous Script (if running and mission not finished)
    if (this.codeEngine.isRunning) {
      const scriptSensors = this.sensors.getScriptInput(this.robot, this.checkpoints);
      this.codeEngine.executeTick(scriptSensors, this.robot, dt);

      if (this.codeEngine.hasError) {
        this.updateModeIndicator("ERROR IN CODE");
      }
    }

    // 4. Run Physics & Multi-Body Collision Detection
    this.physics.step(this.robot, dt, this.cargo);

    // 5. Sound trigger on collision
    if (this.robot.inCollision && Math.random() < 0.08) {
      this.playCollisionSound();
    }

    // 6. Check Finish Point Condition
    if (this.missionState === "RUNNING") {
      const dx = this.robot.x - this.map.goal.x;
      const dy = this.robot.y - this.map.goal.y;
      const distToGoal = Math.hypot(dx, dy);

      if (distToGoal < 0.38) {
        this.handleFinishReached();
      }
    }
  }

  /**
   * Transition to FINISH_REACHED State, log stats, show modal, and handle repeat
   */
  handleFinishReached() {
    this.missionState = "FINISHED";
    this.robot.stop();
    this.updateModeIndicator("FINISH REACHED!");
    this.playVictoryFanfare();

    const capturedCount = this.cargo.cubes.filter(c => c.state === "CAPTURED_INSIDE_MESH").length;
    const cargoMass = (capturedCount * CONFIG.CARGO.MASS).toFixed(2);
    const duration = this.missionTime.toFixed(1);
    const dist = this.robot.totalDistanceTraveled.toFixed(2);
    const hits = this.robot.collisionCount;

    // Log to simulator console
    this.codeEngine.log(`🏁 [FINISH POINT TERCAPAI] Babak #${this.roundNumber} sukses dalam ${duration}s! Kargo: ${capturedCount}/3 (${cargoMass} kg).`);

    // Populate Modal
    const roundBadge = document.getElementById("finish-round-badge");
    if (roundBadge) roundBadge.textContent = `BABAK ${this.roundNumber}`;

    const descEl = document.getElementById("finish-status-desc");
    if (descEl) {
      if (capturedCount === 3) {
        descEl.textContent = "🌟 MISI SEMPURNA! Seluruh 3 kubus muatan berhasil diangkut ke finish point.";
      } else if (capturedCount > 0) {
        descEl.textContent = `⭐ MISI BERHASIL! ${capturedCount} dari 3 kubus muatan berhasil diantar ke finish point.`;
      } else {
        descEl.textContent = "🏁 FINISH POINT TERCAPAI! Smorphi sampai di titik akhir tanpa membawa kargo.";
      }
    }

    const statCargo = document.getElementById("finish-cargo-stat");
    if (statCargo) statCargo.textContent = `${capturedCount}/3 Kubus (${cargoMass} kg)`;

    const statTime = document.getElementById("finish-time-stat");
    if (statTime) statTime.textContent = `${duration} s`;

    const statDist = document.getElementById("finish-dist-stat");
    if (statDist) statDist.textContent = `${dist} m`;

    const statHits = document.getElementById("finish-hits-stat");
    if (statHits) statHits.textContent = `${hits} Hits`;

    // Show modal
    const modal = document.getElementById("finish-modal");
    if (modal) {
      modal.classList.remove("hidden");
      if (window.lucide) lucide.createIcons();
    }

    // Auto-repeat trigger if checked
    if (this.autoRepeatEnabled) {
      this.startAutoRepeatCountdown();
    }
  }

  /**
   * Start 5-second automatic countdown to replay
   */
  startAutoRepeatCountdown() {
    this.clearAutoRepeatTimer();
    this.autoRepeatCountdown = 5;
    const countdownEl = document.getElementById("countdown-text");
    if (countdownEl) {
      countdownEl.classList.remove("hidden");
      countdownEl.textContent = `${this.autoRepeatCountdown}s`;
    }

    this.autoRepeatTimer = setInterval(() => {
      this.autoRepeatCountdown--;
      if (countdownEl) countdownEl.textContent = `${this.autoRepeatCountdown}s`;

      if (this.autoRepeatCountdown <= 0) {
        this.clearAutoRepeatTimer();
        this.restartSimulation(true);
      }
    }, 1000);
  }

  clearAutoRepeatTimer() {
    if (this.autoRepeatTimer) {
      clearInterval(this.autoRepeatTimer);
      this.autoRepeatTimer = null;
    }
    const countdownEl = document.getElementById("countdown-text");
    if (countdownEl) countdownEl.classList.add("hidden");
  }

  /**
   * Replay / Reset simulation
   * @param {boolean} sameMap - true to repeat on current arena, false to generate new maze
   */
  restartSimulation(sameMap = true) {
    this.clearAutoRepeatTimer();

    // Hide modal
    const modal = document.getElementById("finish-modal");
    if (modal) modal.classList.add("hidden");

    const densitySelect = document.getElementById("map-density-select");
    const density = densitySelect ? densitySelect.value : "MEDIUM";

    if (!sameMap) {
      this.roundNumber++;
      this.map.generateMap(density);
      this.cargo.spawnCubes(this.map);
      this.view3d.rebuildObstacleMeshes();
      this.codeEngine.log(`[BABAK BARU #${this.roundNumber}] Labirin baru dimuat. Selamat berjuang!`);
    } else {
      this.cargo.spawnCubes(this.map);
      this.codeEngine.log(`[SIMULASI DIULANG] Posisi robot dan 3 kubus direset ke titik awal.`);
    }

    this.robot.resetPose(this.map.spawn.x, this.map.spawn.y, 0);
    this.resetExploration();
    this.missionState = "RUNNING";
    this.missionTime = 0.0;
    this.codeEngine.resetMemory();

    if (this.manualControlEnabled) {
      this.updateModeIndicator("MANUAL (WASD)");
    } else if (this.codeEngine.isRunning) {
      this.updateModeIndicator("AUTONOMOUS");
    } else {
      this.updateModeIndicator("IDLE");
    }

    this.playBeep(520, 0.1, "sine");
  }

  /**
   * Clear fog exploration and re-sync checkpoints to the freshly spawned cubes.
   */
  resetExploration() {
    this.checkpoints.syncToCargo(this.cargo);
    this.view3d.resetExploration();
  }

  // --- Audio Synthesis Engine ---
  initAudio() {
    if (!this.audioEnabled) return;
    if (!this.audioCtx) {
      const AudioContext = window.AudioContext || window.webkitAudioContext;
      if (AudioContext) {
        this.audioCtx = new AudioContext();
      }
    }
    if (this.audioCtx && this.audioCtx.state === "suspended") {
      this.audioCtx.resume();
    }
  }

  playBeep(freq = 440, duration = 0.1, type = "sine") {
    if (!this.audioEnabled || !this.audioCtx) return;
    try {
      const osc = this.audioCtx.createOscillator();
      const gain = this.audioCtx.createGain();
      osc.type = type;
      osc.frequency.setValueAtTime(freq, this.audioCtx.currentTime);
      gain.gain.setValueAtTime(0.08, this.audioCtx.currentTime);
      gain.gain.exponentialRampToValueAtTime(0.001, this.audioCtx.currentTime + duration);
      osc.connect(gain);
      gain.connect(this.audioCtx.destination);
      osc.start();
      osc.stop(this.audioCtx.currentTime + duration);
    } catch (e) {}
  }

  playCollisionSound() {
    if (!this.audioEnabled || !this.audioCtx) return;
    try {
      const osc = this.audioCtx.createOscillator();
      const gain = this.audioCtx.createGain();
      osc.type = "triangle";
      osc.frequency.setValueAtTime(180, this.audioCtx.currentTime);
      osc.frequency.exponentialRampToValueAtTime(40, this.audioCtx.currentTime + 0.08);
      gain.gain.setValueAtTime(0.12, this.audioCtx.currentTime);
      gain.gain.exponentialRampToValueAtTime(0.001, this.audioCtx.currentTime + 0.09);
      osc.connect(gain);
      gain.connect(this.audioCtx.destination);
      osc.start();
      osc.stop(this.audioCtx.currentTime + 0.09);
    } catch (e) {}
  }

  playVictoryFanfare() {
    if (!this.audioEnabled) return;
    this.initAudio();
    if (!this.audioCtx) return;

    // Ascending celebratory fanfare: C5 (523Hz), E5 (659Hz), G5 (784Hz), C6 (1046Hz)
    const notes = [
      { freq: 523.25, time: 0.00, dur: 0.12 },
      { freq: 659.25, time: 0.12, dur: 0.12 },
      { freq: 783.99, time: 0.24, dur: 0.14 },
      { freq: 1046.50, time: 0.38, dur: 0.35 },
    ];

    const now = this.audioCtx.currentTime;
    notes.forEach((note) => {
      try {
        const osc = this.audioCtx.createOscillator();
        const gain = this.audioCtx.createGain();
        osc.type = "triangle";
        osc.frequency.setValueAtTime(note.freq, now + note.time);
        gain.gain.setValueAtTime(0.12, now + note.time);
        gain.gain.exponentialRampToValueAtTime(0.001, now + note.time + note.dur);
        osc.connect(gain);
        gain.connect(this.audioCtx.destination);
        osc.start(now + note.time);
        osc.stop(now + note.time + note.dur);
      } catch (e) {}
    });
  }
}

// Bootstrap once DOM is ready
window.addEventListener("DOMContentLoaded", () => {
  window.smorphiApp = new SmorphiApp();
});
