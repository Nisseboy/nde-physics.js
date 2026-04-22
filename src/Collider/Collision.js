

// === Lightweight Collision object + pool ===
class Collision {
  constructor() {
    this.a = null; this.b = null;
    this.x = 0; this.y = 0;
    this.nx = 0; this.ny = 0;
    this.penetration = 0;
  }
  reset(a,b) {
    this.a = a; this.b = b;
    this.x = this.y = this.nx = this.ny = this.penetration = 0;
    return this;
  }
}
const CollisionPool = (function(){
  const pool = [];
  return {
    alloc(a,b){
      return (pool.pop() || new Collision()).reset(a,b);
    },
    free(c){ pool.push(c); }
  };
})();


// === Helpers tuned for speed (inline-friendly) ===
// dot product inline in code where needed
// sqrt fallback for magnitude
const EPS = 1e-9;


// === OBB extractor: returns typed arrays for axes [ax,ay,bx,by] and corners Float32Array[8] (x0,y0,x1,y1,...) ===
function getOBBFast(collider) {
  // assume entity.pos = {x,y}, entity.dir is angle in radians, entity.collider.size = {x,y}
  const cx = collider.transform.pos.x, cy = collider.transform.pos.y;
  const hw = collider.size.x * 0.5;
  const hh = collider.size.y * 0.5;
  const c = Math.cos(collider.transform.dir), s = Math.sin(collider.transform.dir);

  // axes: local x, local y
  const axes = [ c, s, -s, c ];

  collider._cornercache[0] = cx + c*hw - s*hh; collider._cornercache[1] = cy + s*hw + c*hh;
  collider._cornercache[2] = cx - c*hw - s*hh; collider._cornercache[3] = cy - s*hw + c*hh;
  collider._cornercache[4] = cx - c*hw + s*hh; collider._cornercache[5] = cy - s*hw - c*hh;
  collider._cornercache[6] = cx + c*hw + s*hh; collider._cornercache[7] = cy + s*hw - c*hh;

  return { axes, corners: collider._cornercache };
}

// Project typed-corners onto axis (ax,ay). corners is Float32Array length 8.
function projectOntoAxisFast(corners, ax, ay) {
  // inline min/max
  let min = Infinity, max = -Infinity;
  let x, y, p;
  for (let i = 0; i < 8; i += 2) {
    x = corners[i]; y = corners[i+1];
    p = x*ax + y*ay;
    if (p < min) min = p;
    if (p > max) max = p;
  }
  return { min, max };
}

// overlap 1D
function overlap1DFast(aMin, aMax, bMin, bMax) {
  return Math.min(aMax, bMax) - Math.max(aMin, bMin);
}

// support point on typed corners along (nx,ny) — returns index of corner with best projection
function supportIndex(corners, nx, ny) {
  let bestI = 0;
  let bestP = corners[0]*nx + corners[1]*ny;
  for (let i = 2; i < 8; i += 2) {
    const p = corners[i]*nx + corners[i+1]*ny;
    if (p > bestP) { bestP = p; bestI = i; }
  }
  return bestI; // index in corners (0..6 step2)
}

// normalize vector (inlined where needed)
function normalizeInline(x, y) {
  const L = Math.sqrt(x*x + y*y) || EPS;
  return [x / L, y / L, L];
}


// find best face index for reference box based on normal (nx,ny)
// returns starting corner index (0,2,4,6)
function findBestFaceIndexFast(corners, nx, ny) {
  let bestIdx = 0;
  let bestDot = -Infinity;
  for (let i = 0; i < 8; i += 2) {
    const x0 = corners[i], y0 = corners[i+1];
    const x1 = corners[(i+2)%8], y1 = corners[(i+3)%8];
    const ex = x1 - x0, ey = y1 - y0;
    // edge normal = (ey, -ex) ; normalize for dot comparisons
    const L = Math.sqrt(ey*ey + ex*ex) || EPS;
    const nxE = ey / L, nyE = -ex / L;
    const d = nxE*nx + nyE*ny;
    if (d > bestDot) { bestDot = d; bestIdx = i; }
  }
  return bestIdx;
}


// Clip segment p0->p1 against plane dot(n,p) >= offset  or <= offset (controlled by keepGreater).
// p0,p1 are {x,y} style plain objects here for clarity — but we avoid creating them in hot path.
function clipSegmentToPlaneSimple(p0x,p0y, p1x,p1y, nx,ny, offset, keepGreater) {
  const out = [];
  const d0 = p0x*nx + p0y*ny - offset;
  const d1 = p1x*nx + p1y*ny - offset;
  const inside0 = keepGreater ? (d0 >= 0) : (d0 <= 0);
  const inside1 = keepGreater ? (d1 >= 0) : (d1 <= 0);
  if (inside0) out.push({x:p0x,y:p0y});
  if (inside1) out.push({x:p1x,y:p1y});
  if (d0 * d1 < 0) {
    const t = d0 / (d0 - d1);
    out.push({ x: p0x + (p1x - p0x)*t, y: p0y + (p1y - p0y)*t });
  }
  return out;
}


// === Main optimized collision creation (rect-rect + circle-circle implemented) ===
function createCollision(a, b) {
  // early broad-phase cheap circle-sphere approx using bounding radii if provided
  // assuming entities may have collider.r for circle or collider.size for rect
  const dx = a.transform.pos.x - b.transform.pos.x;
  const dy = a.transform.pos.y - b.transform.pos.y;
  const sqd = dx*dx + dy*dy;
  const ar = a.r;
  const br = b.r;
  const rsum = ar + br;
  if (sqd > rsum*rsum) return null; // cheap reject

  const c = CollisionPool.alloc(a,b);

  let res;
  let f = PhysicsManager.collisionPairs.get(a.type)?.get(b.type);
  if (f) res = f(a, b, c);
  else {
    f = PhysicsManager.collisionPairs.get(b.type)?.get(a.type);
    res = f(b, a, c);
  }
  
  if (res) return res;

  CollisionPool.free(c);
  return null;
}

