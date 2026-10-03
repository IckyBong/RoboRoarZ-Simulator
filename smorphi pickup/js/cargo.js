/**
 * Cargo Box & Warehouse Mission Management
 * Handles Cargo States [WAITING_PICKUP, ATTACHED_TO_ROBOT, DELIVERED],
 * proximity attachment, delivery zone verification, and mission timing.
 */

class CargoManager {
  constructor(arenaMap) {
    this.map = arenaMap;

    // States: "WAITING_PICKUP" | "ATTACHED_TO_ROBOT" | "DELIVERED"
    this.state = "WAITING_PICKUP";

    // Initial position inside Pickup Zone
    this.x = CONFIG.ARENA.PICKUP_ZONE.x;
    this.y = CONFIG.ARENA.PICKUP_ZONE.y;
    this.z = CONFIG.CARGO.SIZE / 2; // Floor resting elevation

    // Timers & Mission Lifecycle
    this.dockTimer = 0.0;
    this.dropTimer = 0.0;
    this.missionDuration = 0.0;
    this.isMissionActive = false;
    this.isMissionCompleted = false;

    // Callbacks
    this.onPickupCallbacks = [];
    this.onDeliverCallbacks = [];
  }

  /**
   * Start mission timer
   */
  startMission() {
    this.isMissionActive = true;
    this.isMissionCompleted = false;
  }

  /**
   * Pause/Stop mission timer
   */
  stopMission() {
    this.isMissionActive = false;
  }

  /**
   * Reset cargo to original Pickup Zone position and reset timer to 0
   */
  reset() {
    this.state = "WAITING_PICKUP";
    this.x = CONFIG.ARENA.PICKUP_ZONE.x;
    this.y = CONFIG.ARENA.PICKUP_ZONE.y;
    this.z = CONFIG.CARGO.SIZE / 2;
    this.dockTimer = 0.0;
    this.dropTimer = 0.0;
    this.missionDuration = 0.0;
    this.isMissionActive = false;
    this.isMissionCompleted = false;
  }

  /**
   * Update cargo physical state and proximity docking logic
   */
  update(robot, dt) {
    // Auto-activate mission if robot starts moving
    if (!this.isMissionActive && !this.isMissionCompleted) {
      if (Math.hypot(robot.vx, robot.vy) > 0.02 || Math.abs(robot.omega) > 0.05) {
        this.isMissionActive = true;
      }
    }

    // Accumulate time only when mission is active and not finished
    if (this.isMissionActive && !this.isMissionCompleted) {
      this.missionDuration += dt;
    }

    const distToRobot = Math.hypot(robot.x - this.x, robot.y - this.y);
    const isRobotStationary = Math.abs(robot.vx) < 0.08 && Math.abs(robot.vy) < 0.08 && Math.abs(robot.omega) < 0.3;

    // STATE 1: WAITING_PICKUP
    if (this.state === "WAITING_PICKUP") {
      // Auto-docking: If robot is within pickup radius (<0.28m) and stops for 1.0s
      if (distToRobot < CONFIG.CARGO.PICKUP_RADIUS && isRobotStationary) {
        this.dockTimer += dt;
        if (this.dockTimer >= CONFIG.CARGO.PICKUP_DOCK_TIME) {
          this.attach(robot, "auto");
        }
      } else {
        this.dockTimer = Math.max(0, this.dockTimer - dt * 0.5);
      }
    }

    // STATE 2: ATTACHED_TO_ROBOT
    else if (this.state === "ATTACHED_TO_ROBOT") {
      // Synchronize cargo coordinates with robot chassis
      this.x = robot.x;
      this.y = robot.y;
      this.z = CONFIG.ROBOT.MODULE_HEIGHT + CONFIG.CARGO.SIZE / 2 + 0.02;

      // Check distance to Delivery Zone
      const deliveryX = CONFIG.ARENA.DELIVERY_ZONE.x;
      const deliveryY = CONFIG.ARENA.DELIVERY_ZONE.y;
      const distToDelivery = Math.hypot(robot.x - deliveryX, robot.y - deliveryY);

      // Auto-dropoff: If robot reaches Delivery Zone (<0.45m) and stops for 0.8s
      if (distToDelivery < CONFIG.CARGO.DROP_RADIUS && isRobotStationary) {
        this.dropTimer += dt;
        if (this.dropTimer >= CONFIG.CARGO.DROP_DOCK_TIME) {
          this.deliver(robot);
        }
      } else {
        this.dropTimer = Math.max(0, this.dropTimer - dt * 0.5);
      }
    }

    // STATE 3: DELIVERED
    else if (this.state === "DELIVERED") {
      this.z = CONFIG.CARGO.SIZE / 2;
    }
  }

  /**
   * Lock/Attach Cargo to Robot Smorphi
   */
  attach(robot, source = "manual") {
    if (this.state !== "WAITING_PICKUP") return false;

    const dist = Math.hypot(robot.x - this.x, robot.y - this.y);
    // Allow pickup within reasonable reach
    if (dist > 0.40) {
      console.warn(`[CargoManager] Cannot attach: Robot too far (${dist.toFixed(2)}m > 0.40m)`);
      return false;
    }

    this.state = "ATTACHED_TO_ROBOT";
    this.dockTimer = 0.0;
    this.x = robot.x;
    this.y = robot.y;
    this.z = CONFIG.ROBOT.MODULE_HEIGHT + CONFIG.CARGO.SIZE / 2 + 0.02;

    for (const cb of this.onPickupCallbacks) {
      cb({ robot, source });
    }
    return true;
  }

  /**
   * Release / Drop off Cargo
   */
  release(robot) {
    if (this.state !== "ATTACHED_TO_ROBOT") return false;

    const deliveryX = CONFIG.ARENA.DELIVERY_ZONE.x;
    const deliveryY = CONFIG.ARENA.DELIVERY_ZONE.y;
    const distToDelivery = Math.hypot(robot.x - deliveryX, robot.y - deliveryY);

    if (distToDelivery <= CONFIG.CARGO.DROP_RADIUS) {
      return this.deliver(robot);
    } else {
      // Dropped outside delivery zone
      this.state = "WAITING_PICKUP";
      this.x = robot.x;
      this.y = robot.y;
      this.z = CONFIG.CARGO.SIZE / 2;
      return true;
    }
  }

  /**
   * Finalize Delivery inside Drop-off Zone
   */
  deliver(robot) {
    if (this.state === "DELIVERED") return true;

    this.state = "DELIVERED";
    this.x = CONFIG.ARENA.DELIVERY_ZONE.x;
    this.y = CONFIG.ARENA.DELIVERY_ZONE.y;
    this.z = CONFIG.CARGO.SIZE / 2;
    this.isMissionActive = false;
    this.isMissionCompleted = true;

    for (const cb of this.onDeliverCallbacks) {
      cb({
        robot,
        duration: this.missionDuration,
        collisions: robot.collisionCount,
        distance: robot.totalDistanceTraveled,
      });
    }
    return true;
  }

  onPickup(cb) {
    this.onPickupCallbacks.push(cb);
  }

  onDeliver(cb) {
    this.onDeliverCallbacks.push(cb);
  }
}

if (typeof module !== "undefined" && module.exports) {
  module.exports = CargoManager;
}
