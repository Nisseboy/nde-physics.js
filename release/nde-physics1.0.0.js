
/*
This is a built version of nde-physics and is all the source files stitched together, go to the github for source


*/
/* src/Collider/ColliderBase.js */
class ColliderBase extends Serializable {
  constructor() {
    super();

    this.r = undefined;
  }

  calculateProps() {}

  estimateParentMass(parent) {}
  estimateParentMOI(parent) {}

  from(data) {
    super.from(data);

    this.r = data.r;

    return this;
  }
}





/* src/Collider/ColliderCircle.js */
class ColliderCircle extends ColliderBase {
  constructor(r) {
    super();

    this.r = r;
  }

  estimateParentMass(parent) {
    parent.mass = this.r ** 2;
  }
  estimateParentMOI(parent) {
    parent.moi = parent.mass * this.r ** 2;
  }
}





/* src/Collider/ColliderRect.js */
class ColliderRect extends ColliderBase {
  constructor(size) {
    super();

    this.size = size;
    this.halfSize = undefined;
  }


  calculateProps() {
    this.r = Math.sqrt((this.size.x / 2) ** 2 + (this.size.y / 2) ** 2);
    this.halfSize = this.size._div(2);
  }

  estimateParentMass(parent) {
    parent.mass = this.size.x * this.size.y;
  }
  estimateParentMOI(parent) {
    parent.moi = parent.mass * ((this.size.x + this.size.y) / 4) ** 2;
  }

  from(data) {
    super.from(data);

    this.size = new Vec().from(data.size);
    this.halfSize = new Vec().from(data.halfSize);
    this.calculateProps();

    return this;
  }
}





/* src/Collider/Collision.js */
class Collision {
  constructor(a, b) {
    this.a = a;
    this.b = b;

    this.x = undefined; //hit point
    this.y = undefined; 

    this.nx = undefined; //normal
    this.ny = undefined;

    this.penetration = undefined; //depth
  }
}



function getOBB(entity) {
  let size = entity.collider.size;
  let angle = entity.dir;
  let hw = size.x * 0.5;
  let hh = size.y * 0.5;
  let cos = Math.cos(angle);
  let sin = Math.sin(angle);

  let axes = [
    { x: cos, y: sin },       // local x-axis
    { x: -sin, y: cos }       // local y-axis
  ];

  // corners
  let cx = entity.pos.x;
  let cy = entity.pos.y;
  let corners = [
    { x: cx + cos*hw - sin*hh, y: cy + sin*hw + cos*hh },
    { x: cx - cos*hw - sin*hh, y: cy - sin*hw + cos*hh },
    { x: cx - cos*hw + sin*hh, y: cy - sin*hw - cos*hh },
    { x: cx + cos*hw + sin*hh, y: cy + sin*hw - cos*hh }
  ];

  return { x: cx, y: cy, hw, hh, angle, axes, corners };
}

function projectOntoAxis(corners, axis) {
  let min = Infinity;
  let max = -Infinity;
  for (let i = 0; i < corners.length; i++) {
    let c = corners[i];
    let cross = c.x * axis.x + c.y * axis.y;

    min = Math.min(min, cross);
    max = Math.max(max, cross);
  }
  return { min, max };
}

function overlap1D(projA, projB) {
  return Math.min(projA.max, projB.max) - Math.max(projA.min, projB.min);
}
function supportPoint(corners, nx, ny) {
  let best = corners[0];
  let bestProj = best.x * nx + best.y * ny;
  for (let i = 1; i < corners.length; i++) {
    let c = corners[i];
    let proj = c.x * nx + c.y * ny;
    if (proj > bestProj) {
      best = c;
      bestProj = proj;
    }
  }
  return best;
}


// helper: normalize a vector (returns {x,y} and length)
function normalize(x, y) {
  let L = Math.hypot(x, y) || 1e-9;
  return { x: x / L, y: y / L, len: L };
}

// helper: dot
function dot(ax, ay, bx, by) { return ax * bx + ay * by; }

// clip a segment (p0,p1) to the half-space dot(n,p) >= offset if keepGreater=true,
// otherwise keep dot(n,p) <= offset.
function clipSegmentToPlane(p0, p1, nx, ny, offset, keepGreater) {
  let out = [];

  let d0 = dot(nx, ny, p0.x, p0.y) - offset;
  let d1 = dot(nx, ny, p1.x, p1.y) - offset;

  // helper to decide if a point is inside
  const inside = (d) => keepGreater ? (d >= 0) : (d <= 0);

  if (inside(d0)) out.push(p0);
  if (inside(d1)) out.push(p1);

  if (d0 * d1 < 0) {
    // segment intersects plane; compute t of intersection from p0->p1
    let t = d0 / (d0 - d1);
    let ix = p0.x + (p1.x - p0.x) * t;
    let iy = p0.y + (p1.y - p0.y) * t;
    out.push({ x: ix, y: iy });
  }

  return out;
}

// find the face index on box whose edge-normal is most aligned with given normal
function findBestFaceIndex(box, nx, ny) {
  let best = 0, bestDot = -Infinity;
  for (let i = 0; i < 4; i++) {
    let p0 = box.corners[i];
    let p1 = box.corners[(i + 1) % 4];
    // edge vector
    let ex = p1.x - p0.x;
    let ey = p1.y - p0.y;
    // outward normal candidate = (ey, -ex) (perp to edge)
    let n = normalize(ey, -ex); // normalize for dot product comparisions
    let d = dot(n.x, n.y, nx, ny);
    if (d > bestDot) {
      bestDot = d;
      best = i;
    }
  }
  return best;
}

function createCollision(a, b) {
  let sqd = (a.pos.x - b.pos.x) ** 2 + (a.pos.y - b.pos.y) ** 2;
  if (sqd > (a.r + b.r) ** 2) return;
  if (a.mass == 0 && b.mass == 0) return;


  let c = new Collision(a, b);

  let aRect = a.collider.size != undefined;
  let bRect = a.collider.size != undefined;

  if (aRect && bRect) {
    let A = getOBB(a);
    let B = getOBB(b);

    let smallestOverlap = Infinity;
    let smallestAxis = null;
    let axisSource = null; // 'A' or 'B'
    // candidate axes (track source)
    let axes = [
      { x: A.axes[0].x, y: A.axes[0].y, source: 'A' },
      { x: A.axes[1].x, y: A.axes[1].y, source: 'A' },
      { x: B.axes[0].x, y: B.axes[0].y, source: 'B' },
      { x: B.axes[1].x, y: B.axes[1].y, source: 'B' }
    ];

    for (let ax of axes) {
      let projA = projectOntoAxis(A.corners, ax);
      let projB = projectOntoAxis(B.corners, ax);
      let overlap = overlap1D(projA, projB);
      if (overlap <= 0) return null; // separating axis
      if (overlap < smallestOverlap) {
        smallestOverlap = overlap;
        smallestAxis = { x: ax.x, y: ax.y };
        axisSource = ax.source;
      }
    }

    // Ensure normal points from A -> B (consistent with earlier code)
    let dirx = B.x - A.x, diry = B.y - A.y;
    let dotDir = dirx * smallestAxis.x + diry * smallestAxis.y;
    let normalX = dotDir < 0 ? -smallestAxis.x : smallestAxis.x;
    let normalY = dotDir < 0 ? -smallestAxis.y : smallestAxis.y;

    c.penetration = smallestOverlap;
    c.nx = normalX;
    c.ny = normalY;

    // reference box is the one whose axis gave the smallest penetration
    let refBox = axisSource === 'A' ? A : B;
    let incBox = axisSource === 'A' ? B : A;
    // if reference is B, we want normal still point from A->B, but clipping uses refBox as chosen

    // Find reference face (edge index) on reference box
    let refFaceIdx = findBestFaceIndex(refBox, normalX, normalY);

    // reference face vertices (in world space)
    let refV1 = refBox.corners[refFaceIdx];
    let refV2 = refBox.corners[(refFaceIdx + 1) % 4];

    // tangent along the reference face (v2 - v1)
    let t = normalize(refV2.x - refV1.x, refV2.y - refV1.y);
    let tangentX = t.x, tangentY = t.y;

    // endpoints of incident face: pick the edge on incident box whose normal is most opposite to ref normal
    // (i.e. minimal dot with reference normal)
    let bestInc = 0, bestDot = Infinity;
    for (let i = 0; i < 4; i++) {
      let p0 = incBox.corners[i];
      let p1 = incBox.corners[(i + 1) % 4];
      let ex = p1.x - p0.x, ey = p1.y - p0.y;
      let en = normalize(ey, -ex); // edge normal
      let d = dot(en.x, en.y, normalX, normalY);
      if (d < bestDot) { bestDot = d; bestInc = i; }
    }
    let incV1 = incBox.corners[bestInc];
    let incV2 = incBox.corners[(bestInc + 1) % 4];

    // now we have an incident segment [incV1, incV2]; clip it against the two side planes of the reference face
    // plane1: dot(t, p) >= dot(t, refV1)  (inside on one side)
    // plane2: dot(t, p) <= dot(t, refV2)  (inside on the other side)
    let refT1 = dot(tangentX, tangentY, refV1.x, refV1.y);
    let refT2 = dot(tangentX, tangentY, refV2.x, refV2.y);
    let minT = Math.min(refT1, refT2);
    let maxT = Math.max(refT1, refT2);

    // perform clipping of incident segment
    let clipped = clipSegmentToPlane(incV1, incV2, tangentX, tangentY, minT, true);  // keep dot >= minT
    if (clipped.length === 0) {
      // fallback to support average (should be rare)
      let pA = supportPoint(A.corners, normalX, normalY);
      let pB = supportPoint(B.corners, -normalX, -normalY);
      c.x = (pA.x + pB.x) * 0.5;
      c.y = (pA.y + pB.y) * 0.5;
      c.points = [{ x: c.x, y: c.y, penetration: c.penetration }];
      return c;
    }

    // clipped might have up to 3 points (segment intersects plane). If more than 1, construct segment using first two unique points
    // now clip against the opposite bound: dot <= maxT  => keep dot - maxT <= 0  => use keepGreater = false with normal = tangent, offset = maxT
    // we need to convert clipped array (could be 1..3) into a segment candidate; choose the two points with extreme t values
    // build unique list
    let uniq = [];
    for (let p of clipped) {
      if (!uniq.some(q => Math.hypot(q.x - p.x, q.y - p.y) < 1e-8)) uniq.push(p);
    }
    // If more than 2, sort by projection along tangent and take min/max
    if (uniq.length > 2) {
      uniq.sort((p, q) => dot(tangentX, tangentY, p.x, p.y) - dot(tangentX, tangentY, q.x, q.y));
      uniq = [uniq[0], uniq[uniq.length - 1]];
    }
    // now clip each endpoint to maxT
    let clipped2 = [];
    if (uniq.length === 1) {
      // single point: check if within maxT bound
      let d = dot(tangentX, tangentY, uniq[0].x, uniq[0].y) - maxT;
      if (d <= 1e-9) clipped2.push(uniq[0]);
    } else {
      clipped2 = clipSegmentToPlane(uniq[0], uniq[1], tangentX, tangentY, maxT, false); // keep dot <= maxT
    }

    if (clipped2.length === 0) {
      // fallback again
      let pA = supportPoint(A.corners, normalX, normalY);
      let pB = supportPoint(B.corners, -normalX, -normalY);
      c.x = (pA.x + pB.x) * 0.5;
      c.y = (pA.y + pB.y) * 0.5;
      c.points = [{ x: c.x, y: c.y, penetration: c.penetration }];
      return c;
    }

    // at this point clipped2 contains 1..2 contact points. compute penetration for each
    // compute reference face projection (faceMax) along the normal
    let projRef = projectOntoAxis(refBox.corners, { x: normalX, y: normalY });
    let faceMax = projRef.max;

    c.points = [];
    let cx = 0, cy = 0;
    for (let p of clipped2) {
      let pDot = dot(normalX, normalY, p.x, p.y);
      let pen = faceMax - pDot;
      if (pen >= -1e-9) { // keep small negative tolerance
        c.points.push({ x: p.x, y: p.y, penetration: pen });
        cx += p.x; cy += p.y;
      }
    }

    if (c.points.length === 0) {
      // fallback one more time
      let pA = supportPoint(A.corners, normalX, normalY);
      let pB = supportPoint(B.corners, -normalX, -normalY);
      c.x = (pA.x + pB.x) * 0.5;
      c.y = (pA.y + pB.y) * 0.5;
      c.points = [{ x: c.x, y: c.y, penetration: c.penetration }];
      return c;
    }

    // set contact centroid (average)
    c.x = cx / c.points.length;
    c.y = cy / c.points.length;
    
  }
  else if (!aRect && !bRect) {
    let dPos = b.pos._subV(a.pos);
    let mag = dPos.mag();
    let invMag = 1 / mag;

    c.penetration = (a.collider.r + b.collider.r) ** 2 - mag ** 2;
    c.nx = dPos.x * invMag;
    c.ny = dPos.y * invMag;
    c.x = a.pos.x + dPos.x * invMag * c.penetration;
    c.y = a.pos.y + dPos.y * invMag * c.penetration;
  }
  else if ((!aRect && bRect) || (aRect && !bRect)) {

  }
  else return;
  return c;
}





/* src/PhysicsObject.js */
/*class PhysicsObject extends Serializable {
  constructor(pos, collider) {
    super();


    this.pos = pos;
    this.lastPos = this.pos._();
    this.dir = 0;
    this.lastDir = this.dir;

    this.collider = collider;

    this.mass = undefined;
    this.invMass = undefined;
    this.moi = undefined;
    this.invMoi = undefined;

    this.vel = new Vec(0, 0);
    this.av = 0;

    this.acc = new Vec(0, 0);
    this.aa = 0;

    this.calculateProps();
  }





  integrate(dt) {
    if (this.invMass == 0) return;
    let dtsq = dt * dt;

    this.vel.from(this.pos).subV(this.lastPos).addV(this.acc.mul(dtsq));
    this.acc.set(0, 0);
    this.lastPos.from(this.pos);
    this.pos.addV(this.vel);
    this.vel.div(dt);


    this.av = this.dir - this.lastDir + this.aa * dtsq;
    this.aa = 0;
    this.lastDir = this.dir;
    this.dir += this.av;
    this.av /= dt;
    
  }
  
  setVel(v, dt) {
    this.lastPos.from(this.pos).subV(v._mul(dt));
  }
  addVel(v, dt) {
    this.lastPos.subV(v._mul(dt))
  }
  addVelFast(x, y, dt) {
    this.lastPos.x -= x * dt;
    this.lastPos.y -= y * dt;
  }
  
  setPos(p) {
    let diff = this.pos._subV(this.lastPos);
    this.pos.from(p);
    this.lastPos.from(this.pos).subV(diff);
  }
  addPos(p) {
    let diff = this.pos._subV(this.lastPos);
    this.pos.addV(p);
    this.lastPos.from(this.pos).subV(diff);
  }


  setAV(av, dt) {
    this.lastDir = this.dir - av * dt;
  }
  addAV(av, dt) {
    this.lastDir -= av * dt;
  }

  setDir(dir) {
    let diff = this.dir - this.lastDir;
    this.dir = dir;
    this.lastDir = this.dir - diff;
  }
  addDir(dir) {
    let diff = this.dir - this.lastDir;
    this.dir += dir;
    this.lastDir = this.dir - diff;
  }

  applyImpulse(impulse, dt) {
    this.lastPos.subV(impulse._mul(dt * this.invMass));
  }
  applyImpulseFast(x, y, dt) {
    this.lastPos.x -= x * dt * this.invMass;
    this.lastPos.y -= y * dt * this.invMass;
  }

  applyImpulseAtPoint(impulse, pos, dt) {
    this.applyImpulseAtPointFast(impulse.x, impulse.y, pos.x, pos.y, dt)
  }
  applyImpulseAtPointFast(ix, iy, px, py, dt) {
    this.lastPos.x -= ix * dt * this.invMass;
    this.lastPos.y -= iy * dt * this.invMass;

    let x = px - this.pos.x;
    let y = py - this.pos.y;

    //cross product
    let torque = x * iy - y * ix;

    this.lastDir -= torque * this.invMoi * dt;
  }



  calculateProps() {
    this.collider.calculateProps();

    this.collider.estimateParentMass(this);

    this.invMass = 1 / this.mass;
    this.invMoi = 1 / this.moi;
  }


  from(data) {
    super.from(data);

    this.pos = new Vec().from(data.pos);
    this.lastPos = this.pos._();
    this.dir = data.dir;
    this.lastDir = this.dir;

    this.collider = cloneData(data.collider);

    this.mass = data.mass;
    this.invMass = data.invMass;
    this.moi = data.moi;

    return this;
  }
}*/



class PhysicsObject extends Serializable {
  constructor(pos, collider, props = {}) {
    super();

    this.pos = pos;
    this.vel = props.vel || new Vec(0, 0);
    this.lastAcc = new Vec(0, 0);

    this.dir = props.dir || 0;
    this.av = props.av || 0;
    this.lastAA = 0;

    this.force = new Vec(0, 0);
    this.torque = 0;

    this.collider = collider;

    this.mass = props.mass;
    
    
    this.moi = props.moi;

    this.invMass = undefined;
    this.invMoi = undefined;

    if (this.mass == undefined) this.collider.estimateParentMass(this);
    if (this.moi == undefined) this.collider.estimateParentMOI(this);

    this.calculateProps();
  }





  integrate(dt) {
    if (this.invMass == 0) return;


    let newVelHalf = this.lastAcc._mul(dt * 0.5).addV(this.vel);
    this.pos = newVelHalf._mul(dt).addV(this.pos);
    this.lastAcc.from(this.force).mul(this.invMass);
    this.vel.from(this.lastAcc).mul(dt * 0.5).addV(newVelHalf);
    this.force.set(0, 0);


    let newAVHalf = this.lastAA * dt * 0.5 + this.av;
    this.dir += newAVHalf * dt;
    this.lastAA = this.torque * this.invMoi;
    this.av = this.lastAA * dt * 0.5 + newAVHalf;    
    this.torque = 0;
  }

  applyImpulse(impulse) {
    this.applyImpulseFast(impulse.x, impulse.y)
  }
  applyImpulseFast(x, y) {
    this.vel.x += x * this.invMass;
    this.vel.y += y * this.invMass;
  }

  applyImpulseAtPoint(impulse, pos) {
    this.applyImpulseAtPointFast(impulse.x, impulse.y, pos.x, pos.y)
  }
  applyImpulseAtPointFast(ix, iy, px, py) {
    this.vel.x += ix * this.invMass;
    this.vel.y += iy * this.invMass;

    let x = px - this.pos.x;
    let y = py - this.pos.y;

    // Cross product for torque (scalar in 2D)
    let torque = x * iy - y * ix;

    this.av += torque * this.invMoi;
  }



  calculateProps() {
    this.collider.calculateProps();

    this.invMass = this.mass ? 1 / this.mass : 0;
    this.invMoi = this.moi ? 1 / this.moi : 0;
  }


  from(data) {
    super.from(data);

    this.pos = new Vec().from(data.pos);
    this.vel = new Vec().from(data.vel);
    this.lastAcc = new Vec(0, 0);

    this.dir = data.dir;
    this.av = data.av;
    this.lastAA = 0;

    this.force = new Vec(0, 0);
    this.torque = 0;

    this.collider = cloneData(data.collider);

    this.mass = data.mass;
    this.invMass = data.invMass;
    this.moi = data.moi;

    return this;
  }
}





/* src/PhysicsGrid.js */
class PhysicsGrid {
  constructor() {
    this.growFactor = 0.3;

    this.min = new Vec(0, 0);
    this.max = new Vec(1, 1);
    this.size = new Vec(1, 1);
    this.gridSize = new Vec(1, 1);
    this.cellSize = 1;
    this.invCellSize = 1;

    this.grid = [[]];

    this.obs = [];
    this.queued = [];
  }

  addOb(ob) {
    this.obs.push(ob);
    this.queued.push(ob);
  }


  restructure(min, max, cellSize, iterations) {
    let hasChanged = false;
    
    let growFactor2 = 1 + this.growFactor;
    let growX = this.size.x * this.growFactor;
    let growY = this.size.y * this.growFactor;

    if (min.x < this.min.x) {
      this.min.x -= growX;
      hasChanged = true;      
    }
    if (min.y < this.min.y) {
      this.min.y -= growY;
      hasChanged = true;
    }
    if (max.x > this.max.x) {
      this.max.x += growX;
      hasChanged = true;
    }
    if (max.y > this.max.y) {
      this.max.y += growX;
      hasChanged = true;
    }
    if (cellSize > this.cellSize) {
      this.cellSize *= growFactor2;
      hasChanged = true;
    }


    if (iterations < 10) {
      let shrinkFactor = 1 - 1 / growFactor2;
      let shrinkFactor2 = 1 - shrinkFactor
      let shrinkX = Math.max(this.size.x * shrinkFactor, this.cellSize);
      let shrinkY = Math.max(this.size.y * shrinkFactor, this.cellSize);

      if (this.min.x + shrinkX < min.x) {
        this.min.x += shrinkX;
        hasChanged = true;
      }
      if (this.min.y + shrinkY < min.y) {
        this.min.y += shrinkY;
        hasChanged = true;
      }
      if (this.max.x - shrinkX > max.x) {
        this.max.x -= shrinkX;
        hasChanged = true;
      }
      if (this.max.y - shrinkY > max.y) {
        this.max.y -= shrinkY;
        hasChanged = true;
      }
      if (cellSize < this.cellSize * shrinkFactor2) {
        this.cellSize *= shrinkFactor2;
        hasChanged = true;        
      }
    }
    
    
    
    if (!hasChanged) return false;




    this.size.from(this.max).subV(this.min);
    let diff = this.size._();
    this.gridSize = this.size.div(this.cellSize).ceil();
    this.size = this.gridSize._mul(this.cellSize);
    diff.subV(this.size).mul(-0.5);
    this.min.subV(diff);
    this.max.addV(diff);
    
    this.invCellSize = 1 / this.cellSize;
    this.grid = new Array(this.gridSize.x * this.gridSize.y).fill(undefined).map(()=>[]);
    
    return true;
  }

  getGridIndex(ob) {
    return Math.floor((ob.pos.y - this.min.y) * this.invCellSize) * this.gridSize.x + Math.floor((ob.pos.x - this.min.x) * this.invCellSize);
  }

  partition() {
    let max = new Vec(-Infinity, -Infinity);
    let min = new Vec(Infinity, Infinity);
    let cellSize = 0;

    for (let i = 0; i < this.obs.length; i++) {
      let e = this.obs[i];

      max.x = Math.max(max.x, e.pos.x);
      max.y = Math.max(max.y, e.pos.y);
      min.x = Math.min(min.x, e.pos.x);
      min.y = Math.min(min.y, e.pos.y);

      cellSize = Math.max(cellSize, e.collider.r * 2);
    }

    if (max.x == -Infinity) max.x = 10;
    if (max.y == -Infinity) max.y = 10;
    if (min.x == Infinity) min.x = 0;
    if (min.y == Infinity) min.y = 0;
    if (min.x == max.x) max.x += 10;
    if (min.y == max.y) max.y += 10;
    if (cellSize == 0) cellSize = 1;

    let iterations = 0;
    while (this.restructure(min, max, cellSize, iterations)) {iterations++}

    
    
    if (iterations != 0) {
      this.queued.length = 0;
      for (let i = 0; i < this.obs.length; i++) {
        this.grid[this.getGridIndex(this.obs[i])].push(this.obs[i]);
      }
    } else {
      for (let i = 0; i < this.grid.length; i++) {
        let cell = this.grid[i];
        
        for (let j = 0; j < cell.length; j++) {
          if (this.getGridIndex(cell[j]) == i) continue;

          this.queued.push(cell[j]);
          cell.splice(j, 1);
          j--;
        }
      }
    } 
    
    
    for (let i = 0; i < this.queued.length; i++) {
      this.grid[this.getGridIndex(this.queued[i])].push(this.queued[i]);
    }
    this.queued.length = 0;
  }

  getCollisions(onlyStatic = false) {    
    let collisions = [];

    for (let cellIndex = 0; cellIndex < this.grid.length; cellIndex++) {
      let cell = this.grid[cellIndex];
      
      
      for (let i = 0; i < cell.length; i++) {
        let a = cell[i];
        if (onlyStatic && a.mass != 0) continue;



        for (let j = i + 1; j < cell.length; j++) {
          let collision = createCollision(a, cell[j]);
          if (collision) collisions.push(collision);
        }

        let goRight = cellIndex % this.gridSize.x < this.gridSize.x - 1;
        let goDown = Math.floor(cellIndex / this.gridSize.x) < this.gridSize.y - 1;

        if (goRight) {
          let cell2 = this.grid[cellIndex + 1];
          for (let j = 0; j < cell2.length; j++) {
            let collision = createCollision(a, cell2[j]);
            if (collision) collisions.push(collision);
          }
        }
        if (goDown) {
          let cell2 = this.grid[cellIndex + this.gridSize.x];
          for (let j = 0; j < cell2.length; j++) {
            let collision = createCollision(a, cell2[j]);
            if (collision) collisions.push(collision);
          }
        }

        if (goRight && goDown) {
          let cell2 = this.grid[cellIndex + this.gridSize.x + 1];
          for (let j = 0; j < cell2.length; j++) {
            let collision = createCollision(a, cell2[j]);
            if (collision) collisions.push(collision);
          }
        }
      }
    }

    return collisions;
  }
}





/* src/index.js */
/*


*/

class NDEPhysics {
  constructor() {
    this.iterations = 3;
    this.restitution = 0.2;
    this.dampingLinear = 0.02;
    this.dampingRotational = 0.02;

    this.obs = []

    this.gridMoving = new PhysicsGrid();
    this.gridStatic = new PhysicsGrid();
  }

  addOb(ob) {
    if (ob.mass != 0) this.gridMoving.addOb(ob);
    this.gridStatic.addOb(ob);

    this.obs.push(ob);
  }

  update(_dt) {
    let dt = _dt / this.iterations;
    for (let iteration = 0; iteration < this.iterations; iteration++) {
      this.integrate(dt);
      
      this.gridMoving.partition();
      this.gridStatic.partition();

      let collisions = this.gridMoving.getCollisions(false);
      let collisionsStatic = this.gridStatic.getCollisions(true);

      for (let i = 0; i < collisionsStatic.length; i++) {
        this.resolveCollision(collisionsStatic[i]);
      }
      for (let i = 0; i < collisions.length; i++) {
        this.resolveCollision(collisions[i]);
      }      
    }
  }

  resolveCollision(coll, dt) {
    let a = coll.a;
    let b = coll.b;

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

    // --- Friction coefficient (can be dynamic/static) ---
    let friction = 0.5; // You can make this dynamic or per-object later

    // --- Compute friction impulse scalar ---
    let raCrossT = rAx * ty - rAy * tx;
    let rbCrossT = rBx * ty - rBy * tx;
    let invMassTangent = a.invMass + b.invMass + raCrossT * raCrossT * a.invMoi + rbCrossT * rbCrossT * b.invMoi;

    let jt = -velAlongTangent / invMassTangent;

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





