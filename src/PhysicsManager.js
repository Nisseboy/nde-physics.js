/*


*/

class PhysicsManager extends Component {
  constructor(props = {}) {
    super();
    
    this.iterations = 15;

    this.obs = []
    this.rbs = [];
    this.layers = props.layers ?? [
      new PhysicsLayer({size: 2, filterF: (collider) => {
        return collider.r < 2;
      }}),
      new PhysicsLayer({size: 4, filterF: (collider) => {
        return collider.r < 4;
      }}),
      new PhysicsLayer({size: 100, filterF: (collider) => {
        return collider.r < 100;
      }}),
    ];

    /*
      0- no interaction
      1- y checks with x

          0 1 2 3 4
          ---------
      0 | 1 O O O O
      1 | 1 1 O O O
      2 | 0 0 0 O O
      3 | 0 0 0 0 O
      4 | 0 0 0 0 0
    */
    this.layerInteractions = props.layerInteractions ?? [
      1,0,0,
      1,1,0,
      1,1,1,
    ];

    this.staticFrictionThreshold = 0.01; // m/s
    this.staticFrictionRatio = 1.2; // frictionStatic/frictionDynamic

    this.dampingLinear = 0.02;
    this.dampingRotational = 0.02;

    this.constraints = [];


    this.collisionMap = new Map();
  }

  

  addOb(ob) {
    let c = ob.getComponent(Collider);
    let layer = this.calculateLayer(c);
    if (layer == undefined) return;

    c.physicsLayer = layer;
    this.layers[layer].addCollider(c);

    let index = this.obs.indexOf(undefined);
    if (index == -1) index = this.obs.length;

    this.obs[index] = ob;
    this.rbs[index] = ob.getComponent(RigidBody);
  }
  removeOb(ob) {
    let index = this.obs.indexOf(ob);
    if (index == -1) return;
    
    this.obs[index] = undefined;
    this.rbs[index] = undefined;

    let c = ob.getComponent(Collider);
    if (c.physicsLayer != undefined) {
      this.layers[c.physicsLayer].removeCollider(c);
      c.physicsLayer = undefined;
    }
  }
  addObs(ob, removeOthers = false) {
    let colliders = ob.getComponents(Collider);
    
    for (let i = 0; i < colliders.length; i++) {
      let ob = colliders[i].ob;

      if (this.obs.includes(ob)) continue;

      this.addOb(ob);
    }
  }

  calculateLayer(collider) {
    for (let i = 0; i < this.layers.length; i++) {
      if (this.layers[i].filterF(collider)) return i;
    }
  }

  clear() {
    this.addObs(new Ob());
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
    let collisions;

    let dt = _dt / this.iterations;
    for (let iteration = 0; iteration < this.iterations; iteration++) {
      this.integrate(dt);

      collisions = this.getCollisions();

      for (let i = 0; i < this.constraints.length; i++) {
        this.constraints[i].solve();
      }
      
      outer: for (let i = 0; i < collisions.length; i++) {
        let coll = collisions[i];

        for (let j = 0; j < this.constraints.length; j++) {
          let con = this.constraints[j];
          if ((con.a == coll.a && con.b == coll.b) || (con.a == coll.b && con.b == coll.a)) continue outer;
        }

        //collisionPoints.push(new Vec(coll.x, coll.y));

        this.resolveCollision(coll);
      } 
    }

    this.fireCollisionEvents(collisions);

    this.resetForces();
  }

  getCollisions() {
    let num = this.layers.length;

    for (let i = 0; i < num; i++) {
      let layer = this.layers[i];

      layer.partition();
    }

    
    let potentialCollisions = [];
    for (let i = 0; i < num; i++) {
      for (let j = 0; j < num; j++) {
        let interaction = this.layerInteractions[j + i * num];
        
        if (interaction == 0) continue;
        
        this.layers[j].getPotentialCollisions(this.layers[i], potentialCollisions);
      }
    }
    
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

    let aRB = a.getComponent(RigidBody);
    let bRB = b.getComponent(RigidBody);


    if (!aRB || !bRB || (aRB.static && bRB.static)) return;

    //collisionPoints.push(coll);

    // --- Relative positions from COM to contact point ---
    let rAx = coll.x - a.transform.pos.x;
    let rAy = coll.y - a.transform.pos.y;
    let rBx = coll.x - b.transform.pos.x;
    let rBy = coll.y - b.transform.pos.y;

    // --- Velocities at contact point ---
    let vAx = aRB.vel.x - aRB.av * rAy; // ω × r in 2D: (-ω * ry, ω * rx)
    let vAy = aRB.vel.y + aRB.av * rAx;
    let vBx = bRB.vel.x - bRB.av * rBy;
    let vBy = bRB.vel.y + bRB.av * rBx;

    // Relative velocity along normal
    let rvx = vBx - vAx;
    let rvy = vBy - vAy;
    let velAlongNormal = rvx * coll.nx + rvy * coll.ny;
    if (velAlongNormal > 0) return; // separating

    

    // --- Effective mass (scalar) ---
    let raCrossN = rAx * coll.ny - rAy * coll.nx;
    let rbCrossN = rBx * coll.ny - rBy * coll.nx;
    let invMassSum = aRB.invMass + bRB.invMass + raCrossN * raCrossN * aRB.invMoi + rbCrossN * rbCrossN * bRB.invMoi;

    // --- Impulse scalar ---
    let j = -(1 + aRB.restitution * bRB.restitution) * velAlongNormal / invMassSum;
    let ix = j * coll.nx;
    let iy = j * coll.ny;
    
    // --- Apply impulses ---
    aRB.applyImpulseAtPointFast(-ix, -iy, coll.x, coll.y);
    bRB.applyImpulseAtPointFast(ix, iy, coll.x, coll.y);
    

    // --- Recalculate relative velocity at contact point ---
    let vAxF = aRB.vel.x - aRB.av * rAy;
    let vAyF = aRB.vel.y + aRB.av * rAx;
    let vBxF = bRB.vel.x - bRB.av * rBy;
    let vByF = bRB.vel.y + bRB.av * rBx;

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
    let invMassTangent = aRB.invMass + bRB.invMass + raCrossT * raCrossT * aRB.invMoi + rbCrossT * rbCrossT * bRB.invMoi;

    let jt = -velAlongTangent / invMassTangent;

    let friction = aRB.friction * bRB.friction;
    if (velAlongTangent < this.staticFrictionThreshold) friction *= this.staticFrictionRatio;

    // --- Coulomb's Law: clamp friction impulse ---
    let maxFriction = j * friction; // j is the normal impulse scalar
    jt = Math.max(-maxFriction, Math.min(jt, maxFriction));

    // --- Apply friction impulse ---
    let fx = jt * tx;
    let fy = jt * ty;

    aRB.applyImpulseAtPointFast(-fx, -fy, coll.x, coll.y);
    bRB.applyImpulseAtPointFast(fx, fy, coll.x, coll.y);

    // --- Positional correction (linear only) ---
    let percent = 0.2;
    let correction = (coll.penetration / (aRB.invMass + bRB.invMass)) * percent;

    if (!aRB.static) {
      a.transform.pos.x -= correction * coll.nx * aRB.invMass;
      a.transform.pos.y -= correction * coll.ny * aRB.invMass;
    }
    if (!bRB.static) {
      b.transform.pos.x += correction * coll.nx * bRB.invMass;
      b.transform.pos.y += correction * coll.ny * bRB.invMass;
    }
  }

  fireCollisionEvents(collisions) {
    let currentCollisions = new Set();

    let coll, a, b, key;
    for (let i = 0; i < collisions.length; i++) {
      coll = collisions[i];

      a = coll.a.ob.id < coll.b.ob.id ? coll.a.ob.id : coll.b.ob.id;
      b = coll.a.ob.id < coll.b.ob.id ? coll.b.ob.id : coll.a.ob.id;

      key = (0.5 * (a + b) * (a + b + 1) + b);

      currentCollisions.add(key);

      if (!this.collisionMap.has(key)) {
        this.collisionMap.set(key, [coll.a, coll.b]);

        coll.a.fire("collisionEnter", coll.b);
        coll.b.fire("collisionEnter", coll.a);
      }
    }

    for (let [key, pair] of this.collisionMap) {
      if (!currentCollisions.has(key)) {
        this.collisionMap.delete(key);

        pair[0].fire("collisionExit", pair[1]);
        pair[1].fire("collisionExit", pair[0]);
      }
    }
  }


  integrate(dt) {
    let linearDampingFactor = Math.pow(1 - this.dampingLinear, dt);
    let rotationalDampingFactor = Math.pow(1 - this.dampingRotational, dt);

    let i,rb;
    for (i = 0; i < this.rbs.length; i++) {
      rb = this.rbs[i];
      if (!rb) continue;

      rb.vel.mul(linearDampingFactor);
      rb.av *= rotationalDampingFactor;

      rb.integrate(dt);
    }
  }

  accelerate(acc) {
    let i,rb;
    for (i = 0; i < this.rbs.length; i++) {
      rb = this.rbs[i];
      if (!rb) continue;

      rb.force.addV(acc._mul(rb.mass));
    }
  }

  resetForces() {
    let i,rb;
    for (i = 0; i < this.rbs.length; i++) {
      rb = this.rbs[i];
      if (!rb) continue;

      rb.force.set(0, 0);
      rb.torque = 0;
    }
  }
}