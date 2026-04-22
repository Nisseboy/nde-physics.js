class ColliderRect extends Collider {
  constructor(props = {}) {
    super();

    this._size = undefined;
    this.size = props.size ?? vecOne.copy();

    this._cornercache = new Float32Array(8);
  }

  set size(value) {
    this._size = value;
    this.r = Math.sqrt((this.size.x * 0.5) ** 2 + (this.size.y * 0.5) ** 2);
  }
  get size() {
    return this._size;
  }

  render() {
    nde.renderer._(() => {
      nde.renderer.translate(this.transform.pos);
      if (this.transform.dir) nde.renderer.rotate(this.transform.dir);

      nde.renderer.rect(this.size._mul(-0.5), this.size);
    });
  }


  from(data) {
    super.from(data);

    this.size = new Vec().from(data._size);

    return this;
  }
}


PhysicsManager.addCollisionPair("ColliderRect", "ColliderRect", (a, b, c) => {
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
    if (overlap <= 0) { return null; } // separating axis -> no collision
    
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
});

PhysicsManager.addCollisionPair("ColliderCircle", "ColliderRect", (circle, rect, c) => {
  const cBody = circle.transform;
  const rBody = rect.transform;
  
  // 1. Vector from rect center to circle center
  const dx = cBody.pos.x - rBody.pos.x;
  const dy = cBody.pos.y - rBody.pos.y;

  // 2. Rotate that vector into the RECT'S LOCAL SPACE
  // Note: We use -rBody.dir to "un-rotate" the circle's position
  const cos = Math.cos(-rBody.dir);
  const sin = Math.sin(-rBody.dir);
  const localX = dx * cos - dy * sin;
  const localY = dx * sin + dy * cos;

  // 3. Find the closest point on the local AABB
  const hW = rect.size.x * 0.5;
  const hH = rect.size.y * 0.5;
  

  // Clamp local circle center to rect bounds
  const closestX = Math.max(-hW, Math.min(localX, hW));
  const closestY = Math.max(-hH, Math.min(localY, hH));

  // 4. Distance from local circle center to the closest point
  const diffX = localX - closestX;
  const diffY = localY - closestY;
  const distanceSq = diffX * diffX + diffY * diffY;
  const radiusSq = circle.r * circle.r;

  // --- THE EXIT CONDITION ---
  // If the closest point is further than the radius, there is NO collision.
  if (distanceSq > radiusSq) {    
    return null;
  }

  const distance = Math.sqrt(distanceSq);

  // 5. Handle the "Inside" case vs "Outside" case
  if (distance !== 0) {
    // Standard case: Circle center is outside the rect
    c.penetration = circle.r - distance;
    // Local normal points from closest point to local center
    const lnx = diffX / distance;
    const lny = diffY / distance;

    // Rotate local normal back to world space
    const worldCos = Math.cos(rBody.dir);
    const worldSin = Math.sin(rBody.dir);
    c.nx = lnx * worldCos - lny * worldSin;
    c.ny = lnx * worldSin + lny * worldCos;
  } else {
    // Deep case: Circle center is EXACTLY inside the rect
    // Find the shallowest axis to push the circle out
    const overlapX = hW - Math.abs(localX);
    const overlapY = hH - Math.abs(localY);

    const worldCos = Math.cos(rBody.dir);
    const worldSin = Math.sin(rBody.dir);

    if (overlapX < overlapY) {
      c.penetration = overlapX + circle.r;
      const dir = localX > 0 ? 1 : -1;
      c.nx = dir * worldCos;
      c.ny = dir * worldSin;
    } else {
      c.penetration = overlapY + circle.r;
      const dir = localY > 0 ? 1 : -1;
      c.nx = -dir * worldSin;
      c.ny = dir * worldCos;
    }
  }

  // 6. Final Manifold Data
  c.a = rect;
  c.b = circle;

  // Contact point: the point on the Rect surface in world space
  const worldCos = Math.cos(rBody.dir);
  const worldSin = Math.sin(rBody.dir);
  c.x = rBody.pos.x + (closestX * worldCos - closestY * worldSin);
  c.y = rBody.pos.y + (closestX * worldSin + closestY * worldCos);

  return c;
});