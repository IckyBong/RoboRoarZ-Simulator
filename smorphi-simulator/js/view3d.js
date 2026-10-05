/**
 * Three.js 3D Viewport & Visualizer
 * Platform: Single-Block Smorphi Base Unit with Front Cargo Mesh Scoop & 3 Cargo Cubes
 * Features 4-tier multi-stack chassis (Drive, Power, Mini PC, Planar LiDAR),
 * 4 spinning Mecanum wheels, detailed wire-mesh cargo scoop, 3 physical Cargo Cubes,
 * 360° LiDAR laser beams/point cloud, shadows, and multi-camera modes.
 */

class Viewport3D {
  constructor(containerId, arenaMap) {
    this.container = document.getElementById(containerId);
    this.map = arenaMap;

    this.scene = null;
    this.camera = null;
    this.renderer = null;
    this.controls = null;

    // Camera modes: "perspective" | "topdown" | "follow"
    this.cameraMode = "perspective";

    // Visual elements
    this.robotGroup = null;
    this.wheelMeshes = [];
    this.lidarScannerMesh = null;
    this.lidarLaserLines = null;
    this.lidarPointGeometry = null;
    this.lidarPointsMesh = null;
    this.obstacleMeshes = [];
    this.trajectoryLine = null;
    this.goalGroup = null;
    this.cargoMeshes = [];

    // Visualization toggles
    this.showLidarBeams = true;
    this.showTrajectory = true;
    this.showBoundingBoxes = false;

    this.init();
  }

  init() {
    if (!this.container) return;

    const width = this.container.clientWidth || 800;
    const height = this.container.clientHeight || 600;

    // 1. Scene setup
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x090d16);
    this.scene.fog = new THREE.FogExp2(0x090d16, 0.035);

    // 2. Camera setup
    this.camera = new THREE.PerspectiveCamera(45, width / height, 0.05, 50);
    this.camera.position.set(2.5, 4.8, 6.2);
    this.camera.lookAt(2.5, 0, 2.5);

    // 3. WebGL Renderer
    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
    this.renderer.setSize(width, height);
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    this.container.appendChild(this.renderer.domElement);

    // 4. OrbitControls
    if (typeof THREE.OrbitControls !== "undefined") {
      this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
      this.controls.enableDamping = true;
      this.controls.dampingFactor = 0.08;
      this.controls.target.set(2.5, 0, 2.5);
      this.controls.maxPolarAngle = Math.PI / 2 - 0.05;
      this.controls.minDistance = 1.0;
      this.controls.maxDistance = 12.0;
    }

    // 5. Lighting
    this.setupLighting();

    // 6. Arena Environment
    this.buildArenaEnvironment();

    // 7. Single-Block Smorphi Robot & Front Scoop
    this.buildRobotModel();

    // 8. 3 Cargo Cubes
    this.buildCargoCubes();

    // 9. LiDAR Ray Visualization
    this.buildLidarVisualizer();

    // 10. Trajectory Trail
    this.buildTrajectoryVisualizer();

    // 11. Passive Goal Marker
    this.buildGoalMarker();

    // 12. Obstacles
    this.rebuildObstacleMeshes();

    // 13. Resize listener
    window.addEventListener("resize", () => this.onWindowResize());
  }

  setupLighting() {
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.65);
    this.scene.add(ambientLight);

    const sunLight = new THREE.DirectionalLight(0xffffff, 0.95);
    sunLight.position.set(4, 9, 3);
    sunLight.castShadow = true;
    sunLight.shadow.mapSize.width = 2048;
    sunLight.shadow.mapSize.height = 2048;
    sunLight.shadow.camera.near = 0.5;
    sunLight.shadow.camera.far = 18;
    const d = 4.0;
    sunLight.shadow.camera.left = -d;
    sunLight.shadow.camera.right = d;
    sunLight.shadow.camera.top = d;
    sunLight.shadow.camera.bottom = -d;
    sunLight.shadow.bias = -0.0005;
    this.scene.add(sunLight);

    const fillLight = new THREE.DirectionalLight(0x06b6d4, 0.35);
    fillLight.position.set(-2, 4, -2);
    this.scene.add(fillLight);
  }

  buildArenaEnvironment() {
    const W = this.map.width;
    const H = this.map.height;

    // Floor Mesh
    const floorGeo = new THREE.PlaneGeometry(W, H);
    const floorMat = new THREE.MeshStandardMaterial({
      color: 0x0f172a,
      roughness: 0.85,
      metalness: 0.2,
    });
    const floor = new THREE.Mesh(floorGeo, floorMat);
    floor.rotation.x = -Math.PI / 2;
    floor.position.set(W / 2, 0, H / 2);
    floor.receiveShadow = true;
    this.scene.add(floor);

    // Metric Grid (every 0.5m)
    const gridHelper = new THREE.GridHelper(W, 10, 0x38bdf8, 0x1e293b);
    gridHelper.position.set(W / 2, 0.002, H / 2);
    this.scene.add(gridHelper);

    // Perimeter Border Walls
    const wallMat = new THREE.MeshStandardMaterial({
      color: 0x1e293b,
      roughness: 0.6,
      metalness: 0.4,
    });
    const wallThick = CONFIG.ARENA.WALL_THICKNESS;
    const wallH = CONFIG.ARENA.WALL_HEIGHT;

    const wallsData = [
      { w: W + wallThick * 2, h: wallH, d: wallThick, x: W / 2, y: wallH / 2, z: -wallThick / 2 },
      { w: W + wallThick * 2, h: wallH, d: wallThick, x: W / 2, y: wallH / 2, z: H + wallThick / 2 },
      { w: wallThick, h: wallH, d: H, x: -wallThick / 2, y: wallH / 2, z: H / 2 },
      { w: wallThick, h: wallH, d: H, x: W + wallThick / 2, y: wallH / 2, z: H / 2 },
    ];

    for (const w of wallsData) {
      const geo = new THREE.BoxGeometry(w.w, w.h, w.d);
      const mesh = new THREE.Mesh(geo, wallMat);
      mesh.position.set(w.x, w.y, w.z);
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      this.scene.add(mesh);

      // Warning neon stripe on top of perimeter
      const stripeGeo = new THREE.BoxGeometry(w.w, 0.02, w.d);
      const stripeMat = new THREE.MeshBasicMaterial({ color: 0x0284c7 });
      const stripe = new THREE.Mesh(stripeGeo, stripeMat);
      stripe.position.set(w.x, wallH + 0.01, w.z);
      this.scene.add(stripe);
    }
  }

  /**
   * Build 3D Single-Block Smorphi Base Unit with 4 Tiers and Front Cargo Scoop
   */
  buildRobotModel() {
    this.robotGroup = new THREE.Group();
    this.scene.add(this.robotGroup);

    const W = CONFIG.ROBOT.WIDTH;  // 0.170m
    const L = CONFIG.ROBOT.LENGTH; // 0.170m

    // ==========================================
    // TIER 1: CHASSIS BASEPLATE & MECANUM WHEELS (y = 0 to 0.065)
    // ==========================================
    const basePlateGeo = new THREE.BoxGeometry(L * 0.98, 0.016, W * 0.98);
    const basePlateMat = new THREE.MeshStandardMaterial({
      color: 0x0f172a,
      roughness: 0.4,
      metalness: 0.8,
    });
    const basePlate = new THREE.Mesh(basePlateGeo, basePlateMat);
    basePlate.position.y = 0.032;
    basePlate.castShadow = true;
    basePlate.receiveShadow = true;
    this.robotGroup.add(basePlate);

    // 4 Mecanum Wheels at chassis corners
    const wheelR = CONFIG.ROBOT.WHEEL_RADIUS; // 0.030m
    const wheelW = CONFIG.ROBOT.WHEEL_WIDTH;  // 0.025m
    const wheelOffsets = CONFIG.ROBOT.WHEEL_OFFSETS; // [FL, FR, RL, RR]

    this.wheelMeshes = [];

    for (let w = 0; w < 4; w++) {
      const off = wheelOffsets[w];
      const wheelGroup = new THREE.Group();

      const wheelGeo = new THREE.CylinderGeometry(wheelR, wheelR, wheelW, 16);
      const wheelMat = new THREE.MeshStandardMaterial({
        color: 0x1e293b,
        roughness: 0.4,
        metalness: 0.8,
      });
      const wheelMesh = new THREE.Mesh(wheelGeo, wheelMat);
      wheelMesh.rotation.x = Math.PI / 2; // Orient along lateral axis
      wheelMesh.castShadow = true;
      wheelGroup.add(wheelMesh);

      // Angled Mecanum Rollers accent ring
      const rollerRingGeo = new THREE.TorusGeometry(wheelR * 0.82, 0.0035, 6, 16);
      const rollerMat = new THREE.MeshBasicMaterial({ color: 0x94a3b8 });
      const rollerRing = new THREE.Mesh(rollerRingGeo, rollerMat);
      wheelGroup.add(rollerRing);

      // In Three.js: Robot +X is local X, Robot +Y (Left) is local -Z!
      wheelGroup.position.set(off.lx, wheelR, -off.ly);
      this.robotGroup.add(wheelGroup);
      this.wheelMeshes.push(wheelMesh);
    }

    // ==========================================
    // TIER 2: MOTOR DRIVER & POWER DISTRIBUTION DECK (y = 0.065 to 0.150)
    // ==========================================
    const tier2Geo = new THREE.BoxGeometry(L * 0.92, 0.080, W * 0.92);
    const tier2Mat = new THREE.MeshStandardMaterial({
      color: 0x1e293b,
      roughness: 0.5,
      metalness: 0.6,
    });
    const tier2 = new THREE.Mesh(tier2Geo, tier2Mat);
    tier2.position.y = 0.040 + 0.040 + 0.025; // y = 0.105
    tier2.castShadow = true;
    tier2.receiveShadow = true;
    this.robotGroup.add(tier2);

    // Heatsink fins on sides
    const finMat = new THREE.MeshStandardMaterial({ color: 0x475569, metalness: 0.9, roughness: 0.2 });
    for (let f = -2; f <= 2; f++) {
      const finGeo = new THREE.BoxGeometry(0.008, 0.04, 0.004);
      const finMeshL = new THREE.Mesh(finGeo, finMat);
      finMeshL.position.set(-0.065, 0.105, f * 0.025);
      this.robotGroup.add(finMeshL);
    }

    // Power status LED (emerald green)
    const ledGeo = new THREE.CylinderGeometry(0.005, 0.005, 0.006, 12);
    const ledMat = new THREE.MeshBasicMaterial({ color: 0x10b981 });
    const led = new THREE.Mesh(ledGeo, ledMat);
    led.position.set(-0.065, 0.146, 0.05);
    this.robotGroup.add(led);

    // ==========================================
    // TIER 3: MINI PC / SBC COMPUTE & IMU 6-DOF DECK (y = 0.150 to 0.245)
    // ==========================================
    const tier3Geo = new THREE.BoxGeometry(L * 0.88, 0.085, W * 0.88);
    const tier3Mat = new THREE.MeshStandardMaterial({
      color: 0x0f172a,
      roughness: 0.35,
      metalness: 0.7,
    });
    const tier3 = new THREE.Mesh(tier3Geo, tier3Mat);
    tier3.position.y = 0.190;
    tier3.castShadow = true;
    tier3.receiveShadow = true;
    this.robotGroup.add(tier3);

    // Cyan glowing accent rim on Mini PC
    const accentGeo = new THREE.BoxGeometry(L * 0.89, 0.006, W * 0.89);
    const accentMat = new THREE.MeshBasicMaterial({ color: 0x06b6d4 });
    const accent = new THREE.Mesh(accentGeo, accentMat);
    accent.position.y = 0.233;
    this.robotGroup.add(accent);

    // ==========================================
    // TIER 4: 2D PLANAR LIDAR SCANNER TURRET (y = 0.245 to 0.315)
    // ==========================================
    const lidarPedestalGeo = new THREE.CylinderGeometry(0.035, 0.042, 0.028, 16);
    const lidarPedestalMat = new THREE.MeshStandardMaterial({ color: 0x1e293b, metalness: 0.8, roughness: 0.3 });
    const lidarPedestal = new THREE.Mesh(lidarPedestalGeo, lidarPedestalMat);
    lidarPedestal.position.y = 0.252;
    this.robotGroup.add(lidarPedestal);

    const lidarHeadGeo = new THREE.CylinderGeometry(0.032, 0.032, 0.024, 16);
    const lidarHeadMat = new THREE.MeshStandardMaterial({ color: 0x090d16, metalness: 0.9, roughness: 0.2 });
    this.lidarScannerMesh = new THREE.Mesh(lidarHeadGeo, lidarHeadMat);
    this.lidarScannerMesh.position.y = 0.278;

    // Glowing laser diode
    const diodeGeo = new THREE.BoxGeometry(0.012, 0.006, 0.006);
    const diodeMat = new THREE.MeshBasicMaterial({ color: 0x10b981 });
    const diode = new THREE.Mesh(diodeGeo, diodeMat);
    diode.position.set(0.03, 0, 0);
    this.lidarScannerMesh.add(diode);

    this.robotGroup.add(this.lidarScannerMesh);

    // ==========================================
    // FRONT CARGO MESH SCOOP (Mounted on front face: x = +0.085 to +0.225)
    // ==========================================
    this.buildCargoScoop();
  }

  /**
   * Build the Front Cargo Mesh Scoop mechanical attachment
   */
  buildCargoScoop() {
    const scoopGroup = new THREE.Group();

    const mountX = CONFIG.SCOOP ? CONFIG.SCOOP.MOUNT_X : 0.085;
    const scoopLen = CONFIG.SCOOP ? CONFIG.SCOOP.LENGTH : 0.140;
    const outerW = CONFIG.SCOOP ? CONFIG.SCOOP.OUTER_WIDTH : 0.190;
    const scoopH = CONFIG.SCOOP ? CONFIG.SCOOP.HEIGHT : 0.085;
    const wallThick = CONFIG.SCOOP ? CONFIG.SCOOP.WALL_THICKNESS : 0.012;

    const centerX = mountX + scoopLen / 2;
    const halfOuterW = outerW / 2;

    // Material for wire mesh walls
    const meshMat = new THREE.MeshStandardMaterial({
      color: 0x64748b,
      metalness: 0.85,
      roughness: 0.25,
      wireframe: false,
    });

    const meshRimMat = new THREE.MeshStandardMaterial({
      color: 0x38bdf8,
      metalness: 0.6,
      roughness: 0.3,
    });

    // 1. Thin Bottom Lip Plate (slides under cubes)
    const lipGeo = new THREE.BoxGeometry(scoopLen, 0.004, outerW - 0.01);
    const lipMat = new THREE.MeshStandardMaterial({ color: 0x334155, metalness: 0.9, roughness: 0.3 });
    const lipMesh = new THREE.Mesh(lipGeo, lipMat);
    lipMesh.position.set(centerX, 0.006, 0);
    lipMesh.castShadow = true;
    lipMesh.receiveShadow = true;
    scoopGroup.add(lipMesh);

    // 2. Left Mesh Wall Prong (In Three.js Left is -Z)
    const sideWallGeo = new THREE.BoxGeometry(scoopLen, scoopH, wallThick);
    const leftWall = new THREE.Mesh(sideWallGeo, meshMat);
    leftWall.position.set(centerX, scoopH / 2 + 0.006, -(halfOuterW - wallThick / 2));
    leftWall.castShadow = true;
    scoopGroup.add(leftWall);

    const leftRimGeo = new THREE.BoxGeometry(scoopLen, 0.005, wallThick + 0.002);
    const leftRim = new THREE.Mesh(leftRimGeo, meshRimMat);
    leftRim.position.set(centerX, scoopH + 0.006, -(halfOuterW - wallThick / 2));
    scoopGroup.add(leftRim);

    // 3. Right Mesh Wall Prong (+Z)
    const rightWall = new THREE.Mesh(sideWallGeo, meshMat);
    rightWall.position.set(centerX, scoopH / 2 + 0.006, (halfOuterW - wallThick / 2));
    rightWall.castShadow = true;
    scoopGroup.add(rightWall);

    const rightRim = new THREE.Mesh(leftRimGeo, meshRimMat);
    rightRim.position.set(centerX, scoopH + 0.006, (halfOuterW - wallThick / 2));
    scoopGroup.add(rightRim);

    // 4. Back Mesh Wall against robot front chassis
    const backWallGeo = new THREE.BoxGeometry(wallThick, scoopH, outerW);
    const backWall = new THREE.Mesh(backWallGeo, meshMat);
    backWall.position.set(mountX + wallThick / 2, scoopH / 2 + 0.006, 0);
    backWall.castShadow = true;
    scoopGroup.add(backWall);

    // Tactile sensor plate on back mesh
    const sensorPlateGeo = new THREE.BoxGeometry(0.004, 0.025, 0.08);
    const sensorPlateMat = new THREE.MeshBasicMaterial({ color: 0x10b981 });
    const sensorPlate = new THREE.Mesh(sensorPlateGeo, sensorPlateMat);
    sensorPlate.position.set(mountX + wallThick + 0.002, 0.035, 0);
    scoopGroup.add(sensorPlate);

    this.robotGroup.add(scoopGroup);
  }

  /**
   * Build 3 Physical Cargo Cubes with industrial metallic bevel look and neon ID badges
   */
  buildCargoCubes() {
    this.cargoMeshes = [];
    const size = CONFIG.CARGO.SIZE; // 0.090m
    const colors = CONFIG.CARGO.COLORS; // [Amber, Cyan, Emerald]

    for (let i = 0; i < 3; i++) {
      const cubeGroup = new THREE.Group();

      // Main Beveled Cube Body
      const bodyGeo = new THREE.BoxGeometry(size * 0.94, size * 0.94, size * 0.94);
      const bodyMat = new THREE.MeshStandardMaterial({
        color: colors[i].hex,
        roughness: 0.35,
        metalness: 0.7,
      });
      const body = new THREE.Mesh(bodyGeo, bodyMat);
      body.castShadow = true;
      body.receiveShadow = true;
      cubeGroup.add(body);

      // Metallic corner reinforcement trims
      const trimGeo = new THREE.BoxGeometry(size, 0.008, 0.008);
      const trimMat = new THREE.MeshStandardMaterial({ color: 0x1e293b, metalness: 0.9, roughness: 0.2 });
      const topTrimF = new THREE.Mesh(trimGeo, trimMat);
      topTrimF.position.set(0, size * 0.46, size * 0.46);
      cubeGroup.add(topTrimF);

      const topTrimB = new THREE.Mesh(trimGeo, trimMat);
      topTrimB.position.set(0, size * 0.46, -size * 0.46);
      cubeGroup.add(topTrimB);

      // Top glowing badge with label
      const badgeGeo = new THREE.BoxGeometry(size * 0.5, 0.004, size * 0.5);
      const badgeMat = new THREE.MeshBasicMaterial({ color: 0xffffff });
      const badge = new THREE.Mesh(badgeGeo, badgeMat);
      badge.position.y = size * 0.47 + 0.002;
      cubeGroup.add(badge);

      cubeGroup.position.set(2.5, size / 2, 2.5);
      this.scene.add(cubeGroup);
      this.cargoMeshes.push(cubeGroup);
    }
  }

  buildLidarVisualizer() {
    const numBeams = CONFIG.LIDAR.NUM_BEAMS;

    // Point cloud
    this.lidarPointGeometry = new THREE.BufferGeometry();
    const positions = new Float32Array(numBeams * 3);
    const colors = new Float32Array(numBeams * 3);

    this.lidarPointGeometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
    this.lidarPointGeometry.setAttribute("color", new THREE.BufferAttribute(colors, 3));

    const pMat = new THREE.PointsMaterial({
      size: 0.06,
      vertexColors: true,
      transparent: true,
      opacity: 0.9,
    });
    this.lidarPointsMesh = new THREE.Points(this.lidarPointGeometry, pMat);
    this.scene.add(this.lidarPointsMesh);

    // Laser Lines Fan
    const lineGeo = new THREE.BufferGeometry();
    const linePositions = new Float32Array(numBeams * 2 * 3);
    lineGeo.setAttribute("position", new THREE.BufferAttribute(linePositions, 3));

    const lineMat = new THREE.LineBasicMaterial({
      color: 0x06b6d4,
      transparent: true,
      opacity: 0.18,
    });
    this.lidarLaserLines = new THREE.LineSegments(lineGeo, lineMat);
    this.scene.add(this.lidarLaserLines);
  }

  buildTrajectoryVisualizer() {
    const maxPoints = 300;
    const geom = new THREE.BufferGeometry();
    const positions = new Float32Array(maxPoints * 3);
    geom.setAttribute("position", new THREE.BufferAttribute(positions, 3));

    const mat = new THREE.LineBasicMaterial({
      color: 0x38bdf8,
      linewidth: 2,
      transparent: true,
      opacity: 0.7,
    });

    this.trajectoryLine = new THREE.Line(geom, mat);
    this.scene.add(this.trajectoryLine);
  }

  buildGoalMarker() {
    this.goalGroup = new THREE.Group();

    // Floating passive beacon
    const beaconGeo = new THREE.OctahedronGeometry(0.16, 0);
    const beaconMat = new THREE.MeshStandardMaterial({
      color: 0xfacc15,
      emissive: 0xeab308,
      emissiveIntensity: 0.6,
      metalness: 0.8,
      roughness: 0.2,
    });
    this.goalBeacon = new THREE.Mesh(beaconGeo, beaconMat);
    this.goalBeacon.position.y = 0.42;
    this.goalGroup.add(this.goalBeacon);

    // Vertical hologram light cylinder
    const cylGeo = new THREE.CylinderGeometry(0.12, 0.12, 0.85, 16, 1, true);
    const cylMat = new THREE.MeshBasicMaterial({
      color: 0xfacc15,
      transparent: true,
      opacity: 0.2,
      side: THREE.DoubleSide,
    });
    const cyl = new THREE.Mesh(cylGeo, cylMat);
    cyl.position.y = 0.42;
    this.goalGroup.add(cyl);

    // Base ring on floor
    const ringGeo = new THREE.RingGeometry(0.18, 0.26, 32);
    const ringMat = new THREE.MeshBasicMaterial({
      color: 0xfacc15,
      side: THREE.DoubleSide,
    });
    const ring = new THREE.Mesh(ringGeo, ringMat);
    ring.rotation.x = -Math.PI / 2;
    ring.position.y = 0.005;
    this.goalGroup.add(ring);

    this.scene.add(this.goalGroup);
  }

  rebuildObstacleMeshes() {
    for (const mesh of this.obstacleMeshes) {
      this.scene.remove(mesh);
    }
    this.obstacleMeshes = [];

    const crateMat = new THREE.MeshStandardMaterial({
      color: 0x475569,
      roughness: 0.5,
      metalness: 0.3,
    });

    const corridorMat = new THREE.MeshStandardMaterial({
      color: 0x334155,
      roughness: 0.4,
      metalness: 0.5,
    });

    const obsH = 0.35; // 35 cm obstacle height

    for (const obs of this.map.obstacles) {
      let mesh;
      const isCorridor = obs.type === "corridor";

      if (obs.type === "pillar") {
        const radius = Math.min(obs.w, obs.h) / 2;
        const geo = new THREE.CylinderGeometry(radius, radius, obsH, 16);
        mesh = new THREE.Mesh(geo, crateMat);
        mesh.position.set(obs.x + obs.w / 2, obsH / 2, obs.y + obs.h / 2);
      } else {
        const geo = new THREE.BoxGeometry(obs.w, obsH, obs.h);
        mesh = new THREE.Mesh(geo, isCorridor ? corridorMat : crateMat);
        mesh.position.set(obs.x + obs.w / 2, obsH / 2, obs.y + obs.h / 2);

        if (isCorridor) {
          const edgeGeo = new THREE.BoxGeometry(obs.w, 0.02, obs.h);
          const edgeMat = new THREE.MeshBasicMaterial({ color: 0xf59e0b });
          const edge = new THREE.Mesh(edgeGeo, edgeMat);
          edge.position.y = obsH / 2 + 0.01;
          mesh.add(edge);
        }
      }

      mesh.castShadow = true;
      mesh.receiveShadow = true;
      this.scene.add(mesh);
      this.obstacleMeshes.push(mesh);
    }
  }

  /**
   * Update 3D elements on each frame
   * @param {SmorphiRobot} robot
   * @param {SensorSuite} sensorSuite
   * @param {number} dt
   * @param {CargoManager} [cargoManager]
   * @param {boolean} [isGoalReached]
   */
  update(robot, sensorSuite, dt, cargoManager = null, isGoalReached = false) {
    // 1. Update Robot Position & Heading
    this.robotGroup.position.set(robot.x, 0, robot.y);
    this.robotGroup.rotation.y = -robot.theta;

    // 2. Spin 4 Mecanum Wheels
    for (let w = 0; w < 4; w++) {
      if (this.wheelMeshes[w]) {
        this.wheelMeshes[w].rotation.x = robot.wheelAngles[w];
      }
    }

    // 3. Spin LiDAR Turret (10 Hz)
    if (this.lidarScannerMesh) {
      this.lidarScannerMesh.rotation.y += Math.PI * 2 * 10 * dt;
    }

    // 4. Update 3 Cargo Cubes
    if (cargoManager && cargoManager.cubes) {
      for (let i = 0; i < 3; i++) {
        const cube = cargoManager.cubes[i];
        if (this.cargoMeshes[i]) {
          this.cargoMeshes[i].position.set(cube.x, cube.size / 2, cube.y);
          this.cargoMeshes[i].rotation.y = -cube.theta;
        }
      }
    }

    // 5. Update LiDAR Rays & Points
    this.updateLidarVisualization(sensorSuite.lidarPoints, robot);

    // 6. Update Trajectory Trail
    this.updateTrajectoryVisualization(robot.trajectory);

    // 7. Update Goal Beacon (with dynamic celebration effect if reached)
    if (this.goalGroup) {
      this.goalGroup.position.set(this.map.goal.x, 0, this.map.goal.y);
      if (isGoalReached) {
        this.goalBeacon.rotation.y += 4.5 * dt;
        this.goalBeacon.rotation.x += 2.0 * dt;
        this.goalBeacon.position.y = 0.42 + Math.sin(Date.now() * 0.008) * 0.08;
        if (this.goalBeacon.material) {
          this.goalBeacon.material.color.setHex(0x10b981);
          this.goalBeacon.material.emissive.setHex(0x059669);
          this.goalBeacon.material.emissiveIntensity = 0.9 + Math.sin(Date.now() * 0.01) * 0.4;
        }
      } else {
        this.goalBeacon.rotation.y += 1.8 * dt;
        this.goalBeacon.rotation.x += 0.8 * dt;
        this.goalBeacon.position.y = 0.42;
        if (this.goalBeacon.material) {
          this.goalBeacon.material.color.setHex(0xfacc15);
          this.goalBeacon.material.emissive.setHex(0xeab308);
          this.goalBeacon.material.emissiveIntensity = 0.6;
        }
      }
    }

    // 8. Camera
    this.updateCamera(robot);

    // 9. Render WebGL Scene
    if (this.renderer && this.scene && this.camera) {
      this.renderer.render(this.scene, this.camera);
    }
  }

  updateLidarVisualization(points, robot) {
    if (!this.lidarPointsMesh || !points || points.length === 0) return;

    this.lidarPointsMesh.visible = this.showLidarBeams;
    this.lidarLaserLines.visible = this.showLidarBeams;

    if (!this.showLidarBeams) return;

    const posAttr = this.lidarPointGeometry.attributes.position;
    const colAttr = this.lidarPointGeometry.attributes.color;
    const linePosAttr = this.lidarLaserLines.geometry.attributes.position;

    const sensorY = CONFIG.LIDAR.ELEVATION_Y || 0.285;

    for (let i = 0; i < points.length; i++) {
      const pt = points[i];
      const hitX = pt.x;
      const hitY = sensorY;
      const hitZ = pt.y;

      posAttr.setXYZ(i, hitX, hitY, hitZ);

      // Color coding: Red < 0.6m, Amber < 1.2m, Cyan >= 1.2m
      if (pt.distance < 0.6) {
        colAttr.setXYZ(i, 0.94, 0.27, 0.27);
      } else if (pt.distance < 1.2) {
        colAttr.setXYZ(i, 0.96, 0.62, 0.07);
      } else {
        colAttr.setXYZ(i, 0.02, 0.71, 0.83);
      }

      // Laser lines from sensor center to hit point
      const lineIdx = i * 2;
      linePosAttr.setXYZ(lineIdx, robot.x, sensorY, robot.y);
      linePosAttr.setXYZ(lineIdx + 1, hitX, hitY, hitZ);
    }

    this.lidarPointGeometry.attributes.position.needsUpdate = true;
    this.lidarPointGeometry.attributes.color.needsUpdate = true;
    this.lidarLaserLines.geometry.attributes.position.needsUpdate = true;
  }

  updateTrajectoryVisualization(trajectory) {
    if (!this.trajectoryLine) return;
    this.trajectoryLine.visible = this.showTrajectory;
    if (!this.showTrajectory || !trajectory || trajectory.length === 0) {
      this.trajectoryLine.geometry.setDrawRange(0, 0);
      return;
    }

    const positions = this.trajectoryLine.geometry.attributes.position;
    const count = Math.min(trajectory.length, 300);

    for (let i = 0; i < count; i++) {
      positions.setXYZ(i, trajectory[i].x, 0.008, trajectory[i].y);
    }

    this.trajectoryLine.geometry.setDrawRange(0, count);
    positions.needsUpdate = true;
  }

  updateCamera(robot) {
    if (this.cameraMode === "topdown") {
      this.camera.position.set(this.map.width / 2, 7.5, this.map.height / 2);
      this.camera.lookAt(this.map.width / 2, 0, this.map.height / 2);
      if (this.controls) this.controls.enabled = false;
    } else if (this.cameraMode === "follow") {
      if (this.controls) this.controls.enabled = false;
      const dist = 1.6;
      const height = 1.2;
      const cosT = Math.cos(robot.theta);
      const sinT = Math.sin(robot.theta);

      const targetCamX = robot.x - cosT * dist;
      const targetCamZ = robot.y - sinT * dist;

      this.camera.position.x += (targetCamX - this.camera.position.x) * 0.12;
      this.camera.position.y += (height - this.camera.position.y) * 0.12;
      this.camera.position.z += (targetCamZ - this.camera.position.z) * 0.12;

      this.camera.lookAt(robot.x + cosT * 0.5, 0.15, robot.y + sinT * 0.5);
    } else {
      if (this.controls) {
        this.controls.enabled = true;
        this.controls.update();
      }
    }
  }

  setCameraMode(mode) {
    this.cameraMode = mode;
    if (mode === "perspective" && this.controls) {
      this.controls.enabled = true;
      this.camera.position.set(2.5, 4.8, 6.2);
      this.controls.target.set(2.5, 0, 2.5);
    }
  }

  onWindowResize() {
    if (!this.container || !this.renderer || !this.camera) return;
    const width = this.container.clientWidth || 800;
    const height = this.container.clientHeight || 600;
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height);
  }
}

if (typeof module !== "undefined" && module.exports) {
  module.exports = Viewport3D;
}
