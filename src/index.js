/*


*/

class NDEPhysics {
  constructor() {
    this.iterations = 15;

    this.staticFrictionThreshold = 0.01; // m/s
    this.staticFrictionRatio = 1.2; // frictionStatic/frictionDynamic

    this.restitution = 0.2;
    this.dampingLinear = 0.02;
    this.dampingRotational = 0.02;

    this.obs = []
    this.constraints = [];

    this.gridMoving = new PhysicsGrid();
    this.gridStatic = new PhysicsGrid();
  }

  addOb(ob) {
    if (ob.mass != 0) this.gridMoving.addOb(ob);
    this.gridStatic.addOb(ob);

    this.obs.push(ob);
  }
  removeOb(ob) {
    this.gridMoving.removeOb(ob);
    this.gridStatic.removeOb(ob);

    let index = this.obs.findIndex(ob);
    if (index != -1) this.obs.splice(index, 1);
  }
  clearObs() {
    this.gridMoving.clearObs();
    this.gridStatic.clearObs();

    this.obs.length = 0;
  }

  addConstraint(constraint) {
    this.constraints.push(constraint);
  }
  removeConstraint(constraint) {
    let index = this.constraints.findIndex(constraint);
    if (index != -1) this.constraints.splice(index, 1);
  }
  clearConstraints() {
    this.constraints.length = 0;
  }

  update(_dt) {
    let dt = _dt / this.iterations;
    for (let iteration = 0; iteration < this.iterations; iteration++) {
      this.integrate(dt);

      let collisions = this.getCollisions();

      for (let i = 0; i < this.constraints.length; i++) {
        this.constraints[i].solve();
      }

      outer: for (let i = 0; i < collisions.length; i++) {
        let coll = collisions[i];

        for (let j = 0; j < this.constraints.length; j++) {
          let con = this.constraints[j];
          if ((con.a == coll.a && con.b == coll.b) || (con.a == coll.b && con.b == coll.a)) continue outer;
        }

        this.resolveCollision(coll);
      } 

    }

    for (let i = 0; i < this.obs.length; i++) {
      this.obs[i].force.set(0, 0);
      this.obs[i].torque = 0;
    }
  }

  getCollisions() {
    //collisionPoints.length = [];

    this.gridMoving.partition();
    this.gridStatic.partition();

    let potentialCollisions = [];
    this.gridMoving.getPotentialCollisions(potentialCollisions, false);
    this.gridStatic.getPotentialCollisions(potentialCollisions, true);
    let collisions = [];

    for (let i = 0; i < potentialCollisions.length; i += 2) {
      let collision = createCollision(potentialCollisions[i], potentialCollisions[i+1]);
      if (collision) collisions.push(collision);
    }

    return collisions;
  }

  resolveCollision(coll) {
    let a = coll.a;
    let b = coll.b;

    
    //collisionPoints.push(coll);

    // --- Relative positions from COM to contact point ---
    let rAx = coll.x - a.pos.x;
    let rAy = coll.y - a.pos.y;
    let rBx = coll.x - b.pos.x;
    let rBy = coll.y - b.pos.y;

    // --- Velocities at contact point ---
    let vAx = a.vel.x - a.av * rAy; // ω × r in 2D: (-ω * ry, ω * rx)
    let vAy = a.vel.y + a.av * rAx;
    let vBx = b.vel.x - b.av * rBy;
    let vBy = b.vel.y + b.av * rBx;

    // Relative velocity along normal
    let rvx = vBx - vAx;
    let rvy = vBy - vAy;
    let velAlongNormal = rvx * coll.nx + rvy * coll.ny;
    if (velAlongNormal > 0) return; // separating

    

    // --- Effective mass (scalar) ---
    let raCrossN = rAx * coll.ny - rAy * coll.nx;
    let rbCrossN = rBx * coll.ny - rBy * coll.nx;
    let invMassSum = a.invMass + b.invMass + raCrossN * raCrossN * a.invMoi + rbCrossN * rbCrossN * b.invMoi;

    // --- Impulse scalar ---
    let j = -(1 + this.restitution) * velAlongNormal / invMassSum;
    let ix = j * coll.nx;
    let iy = j * coll.ny;

    // --- Apply impulses ---
    a.applyImpulseAtPointFast(-ix, -iy, coll.x, coll.y);
    b.applyImpulseAtPointFast(ix, iy, coll.x, coll.y);

    // --- Recalculate relative velocity at contact point ---
    let vAxF = a.vel.x - a.av * rAy;
    let vAyF = a.vel.y + a.av * rAx;
    let vBxF = b.vel.x - b.av * rBy;
    let vByF = b.vel.y + b.av * rBx;

    let rvxF = vBxF - vAxF;
    let rvyF = vByF - vAyF;

    // --- Tangent vector (perpendicular to normal) ---
    let tx = -coll.ny;
    let ty = coll.nx;

    // --- Project relative velocity onto tangent ---
    let velAlongTangent = rvxF * tx + rvyF * ty;
    if (Math.abs(velAlongTangent) < 1e-6) return; // No significant tangential motion

    // --- Compute friction impulse scalar ---
    let raCrossT = rAx * ty - rAy * tx;
    let rbCrossT = rBx * ty - rBy * tx;
    let invMassTangent = a.invMass + b.invMass + raCrossT * raCrossT * a.invMoi + rbCrossT * rbCrossT * b.invMoi;

    let jt = -velAlongTangent / invMassTangent;

    let friction = a.friction * b.friction;
    if (velAlongTangent < this.staticFrictionThreshold) friction *= this.staticFrictionRatio;

    // --- Coulomb's Law: clamp friction impulse ---
    let maxFriction = j * friction; // j is the normal impulse scalar
    jt = Math.max(-maxFriction, Math.min(jt, maxFriction));

    // --- Apply friction impulse ---
    let fx = jt * tx;
    let fy = jt * ty;

    a.applyImpulseAtPointFast(-fx, -fy, coll.x, coll.y);
    b.applyImpulseAtPointFast(fx, fy, coll.x, coll.y);

    // --- Positional correction (linear only) ---
    let percent = 0.2;
    let correction = (coll.penetration / (a.invMass + b.invMass)) * percent;

    a.pos.x -= correction * coll.nx * a.invMass;
    a.pos.y -= correction * coll.ny * a.invMass;
    b.pos.x += correction * coll.nx * b.invMass;
    b.pos.y += correction * coll.ny * b.invMass;
  }


  integrate(dt) {
    for (let i = 0; i < this.obs.length; i++) {
      this.obs[i].integrate(dt);

      this.obs[i].vel.mul(Math.exp(-this.dampingLinear * dt));
      this.obs[i].av *= Math.exp(-this.dampingRotational * dt);
    }
  }
}