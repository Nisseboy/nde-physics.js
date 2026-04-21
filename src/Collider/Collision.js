

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

  const aRect = a instanceof ColliderRect;
  const bRect = b instanceof ColliderRect;

  // --- rect-rect (OBB vs OBB) ---
  if (aRect && bRect) {
    const A = getOBBFast(a);
    const B = getOBBFast(b);

    // axes are [ax,ay,bx,by] – but in getOBBFast it's [c,s,-s,c]; mapping below:
    const axisList = [
      [A.axes[0], A.axes[1], 0], // source 0 = A
      [A.axes[2], A.axes[3], 0],
      [B.axes[0], B.axes[1], 1],
      [B.axes[2], B.axes[3], 1]
    ];

    let smallestOverlap = Infinity;
    let smallestAxisX = 0, smallestAxisY = 0, axisSource = 0;

    // SAT loop (unrolled-ish)
    for (let i = 0; i < 4; i++) {
      const ax = axisList[i][0], ay = axisList[i][1];
      const projA = projectOntoAxisFast(A.corners, ax, ay);
      const projB = projectOntoAxisFast(B.corners, ax, ay);
      const overlap = overlap1DFast(projA.min, projA.max, projB.min, projB.max);
      if (overlap <= 0) { CollisionPool.free(c); return null; } // separating axis -> no collision
      if (overlap < smallestOverlap) {
        smallestOverlap = overlap;
        smallestAxisX = ax; smallestAxisY = ay;
        axisSource = axisList[i][2];
      }
    }

    // Build normal consistent from A->B
    let dirx = b.transform.pos.x - a.transform.pos.x, diry = b.transform.pos.y - a.transform.pos.y;
    let dotDir = dirx*smallestAxisX + diry*smallestAxisY;
    let normalX = dotDir < 0 ? -smallestAxisX : smallestAxisX;
    let normalY = dotDir < 0 ? -smallestAxisY : smallestAxisY;

    // choose reference and incident boxes
    // axisSource === 0 => axis from A => reference = A, incident = B; else reverse
    let refBox = axisSource === 0 ? A : B;
    let incBox = axisSource === 0 ? B : A;

    // If reference is the second box we want to flip for clipping's outward plane orientation.
    // But we still want the collision normal to point A->B when returning to caller.
    if (axisSource === 1) {
      // flip normal for clipping plane selection (we'll adjust c.a/c.b so result matches expected semantics)
      normalX = -normalX; normalY = -normalY;
      c.a = b; c.b = a; // swap so c.n points from A -> B in final result
    } else {
      c.a = a; c.b = b;
    }

    c.penetration = smallestOverlap;
    c.nx = normalX; c.ny = normalY;

    // reference face index and vertices
    const refIdx = findBestFaceIndexFast(refBox.corners, normalX, normalY);
    const refV1x = refBox.corners[refIdx], refV1y = refBox.corners[refIdx+1];
    const refV2x = refBox.corners[(refIdx+2)%8], refV2y = refBox.corners[(refIdx+3)%8];

    // tangent t = normalized(refV2 - refV1)
    const txRaw = (refV2x - refV1x), tyRaw = (refV2y - refV1y);
    const tnorm = Math.sqrt(txRaw*txRaw + tyRaw*tyRaw) || EPS;
    const tX = txRaw / tnorm, tY = tyRaw / tnorm;

    // find incident face on incident box (most opposite to ref normal)
    let bestInc = 0, bestIncDot = Infinity;
    for (let i = 0; i < 8; i += 2) {
      const p0x = incBox.corners[i], p0y = incBox.corners[i+1];
      const p1x = incBox.corners[(i+2)%8], p1y = incBox.corners[(i+3)%8];
      const ex = p1x - p0x, ey = p1y - p0y;
      const L = Math.sqrt(ex*ex + ey*ey) || EPS;
      const nxE = ey / L, nyE = -ex / L; // edge normal
      const d = nxE*normalX + nyE*normalY;
      if (d < bestIncDot) { bestIncDot = d; bestInc = i; }
    }
    const incV1x = incBox.corners[bestInc], incV1y = incBox.corners[bestInc+1];
    const incV2x = incBox.corners[(bestInc+2)%8], incV2y = incBox.corners[(bestInc+3)%8];

    // Clip incident segment against the two side planes of the reference face:
    // keep dot(t, p) >= minT  and dot(t, p) <= maxT
    const refT1 = refV1x*tX + refV1y*tY;
    const refT2 = refV2x*tX + refV2y*tY;
    const minT = Math.min(refT1, refT2);
    const maxT = Math.max(refT1, refT2);

    // First clip: keep dot >= minT
    let clipped = clipSegmentToPlaneSimple(incV1x,incV1y, incV2x,incV2y, tX, tY, minT, true);
    if (clipped.length === 0) {
      // fallback: centroid of support points
      const ai = supportIndex(A.corners, normalX, normalY);
      const bi = supportIndex(B.corners, -normalX, -normalY);
      const px = (A.corners[ai] + B.corners[bi]) * 0.5;
      const py = (A.corners[ai+1] + B.corners[bi+1]) * 0.5;
      c.x = px; c.y = py;
      return c;
    }

    // dedupe points (small) and pick extreme by dot(t,p) for segment
    let uniq = [];
    for (let P of clipped) {
      let dup = false;
      for (let Q of uniq) {
        const dxp = P.x - Q.x, dyp = P.y - Q.y;
        if (dxp*dxp + dyp*dyp < 1e-12) { dup = true; break; }
      }
      if (!dup) uniq.push(P);
    }
    if (uniq.length > 2) {
      uniq.sort((p,q) => (p.x*tX + p.y*tY) - (q.x*tX + q.y*tY));
      uniq = [uniq[0], uniq[uniq.length-1]];
    }

    // Second clip: keep dot <= maxT
    let clipped2 = [];
    if (uniq.length === 1) {
      if ((uniq[0].x*tX + uniq[0].y*tY) - maxT <= 1e-9) clipped2.push(uniq[0]);
    } else {
      clipped2 = clipSegmentToPlaneSimple(uniq[0].x,uniq[0].y, uniq[1].x,uniq[1].y, tX, tY, maxT, false);
    }

    if (clipped2.length === 0) {
      const ai = supportIndex(A.corners, normalX, normalY);
      const bi = supportIndex(B.corners, -normalX, -normalY);
      const px = (A.corners[ai] + B.corners[bi]) * 0.5;
      const py = (A.corners[ai+1] + B.corners[bi+1]) * 0.5;
      c.x = px; c.y = py;
      return c;
    }

    // Now compute penetration for each clipped point relative to the reference face projection
    const projRef = projectOntoAxisFast(refBox.corners, normalX, normalY);
    const faceMax = projRef.max;
    let cx = 0, cy = 0, keepCount = 0;
    for (let P of clipped2) {
      const pDot = P.x*normalX + P.y*normalY;
      const pen = faceMax - pDot;
      if (pen >= -1e-9) { cx += P.x; cy += P.y; keepCount++; }
    }
    if (keepCount === 0) {
      const ai = supportIndex(A.corners, normalX, normalY);
      const bi = supportIndex(B.corners, -normalX, -normalY);
      const px = (A.corners[ai] + B.corners[bi]) * 0.5;
      const py = (A.corners[ai+1] + B.corners[bi+1]) * 0.5;
      c.x = px; c.y = py;
      return c;
    }
    c.x = cx / keepCount; c.y = cy / keepCount;
    return c;
  }

  // --- circle-circle (simple, optimized) ---
  if (!aRect && !bRect) {
    // assume pos objects with x,y and helpers omitted; using raw arithmetic
    const dx2 = b.transform.pos.x - a.transform.pos.x;
    const dy2 = b.transform.pos.y - a.transform.pos.y;
    const mag2 = Math.sqrt(dx2*dx2 + dy2*dy2) || EPS;
    const rsum2 = a.r + b.r;
    const penetration = rsum2 - mag2;
    if (penetration <= 0) { CollisionPool.free(c); return null; }
    c.penetration = penetration;
    c.nx = dx2 / mag2; c.ny = dy2 / mag2;
    // contact point: move from a towards b by a.r along the normal
    c.x = a.transform.pos.x + c.nx * (a.r - penetration*0.5);
    c.y = a.transform.pos.y + c.ny * (a.r - penetration*0.5);
    return c;
  }

  // --- mixed circle-rect (TODO: implement if you need it; SAT from circle to OBB with closest point) ---
  // For now, treat as collision; you should implement closest-point on OBB vs circle (fast).
  CollisionPool.free(c);
  return null;
}

