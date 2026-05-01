class ColliderGrid extends Collider {
  constructor(props = {}) {
    super();

    this.size = props.size ?? vecOne.copy();

    this.g = new Array(this.size.x * this.size.y);
  }

  set size(value) {
    this._size = value;
    this.r = Math.sqrt((this.size.x * 0.5) ** 2 + (this.size.y * 0.5) ** 2);
  }
  get size() {
    return this._size;
  }

  from(data) {
    super.from(data);

    this.size = new Vec().from(data._size);

    return this;
  }
}

PhysicsManager.addCollisionPair("ColliderGrid", "ColliderCircle", (grid, circle, c) => {
  const dx = circle.transform.pos.x - grid.transform.pos.x;
  const dy = circle.transform.pos.y - grid.transform.pos.y;
  const cos = Math.cos(-grid.transform.dir);
  const sin = Math.sin(-grid.transform.dir);
  const localX = dx * cos - dy * sin;
  const localY = dx * sin + dy * cos;

  const minCX = Math.floor(localX - circle.r);
  const maxCX = Math.floor(localX + circle.r);
  const minCY = Math.floor(localY - circle.r);
  const maxCY = Math.floor(localY + circle.r);

  let bestPen = -Infinity;
  let bestNX = 0, bestNY = 0, bestCX = 0, bestCY = 0;

  for (let cy = minCY; cy <= maxCY; cy++) {
    for (let cx = minCX; cx <= maxCX; cx++) {
      if (cx < 0 || cy < 0 || cx >= grid.size.x || cy >= grid.size.y) continue;
      if (!grid.g[cx + cy * grid.size.x]) continue;

      // Cell center in local space
      const cellCX = cx + 0.5;
      const cellCY = cy + 0.5;

      const relX = localX - cellCX;
      const relY = localY - cellCY;

      const closestX = Math.max(-0.5, Math.min(relX, 0.5));
      const closestY = Math.max(-0.5, Math.min(relY, 0.5));

      const diffX = relX - closestX;
      const diffY = relY - closestY;
      const distSq = diffX * diffX + diffY * diffY;

      if (distSq > circle.r * circle.r) continue;

      const dist = Math.sqrt(distSq);

      let pen, lnx, lny;
      if (dist > 0) {
        pen = circle.r - dist;
        lnx = diffX / dist;
        lny = diffY / dist;
      } else {
        const overlapX = 0.5 - Math.abs(relX);
        const overlapY = 0.5 - Math.abs(relY);
        if (overlapX < overlapY) {
          pen = overlapX + circle.r;
          lnx = relX > 0 ? 1 : -1;
          lny = 0;
        } else {
          pen = overlapY + circle.r;
          lnx = 0;
          lny = relY > 0 ? 1 : -1;
        }
      }

      if (pen > bestPen) {
        bestPen = pen;
        bestNX = lnx; bestNY = lny;
        bestCX = cellCX + closestX;
        bestCY = cellCY + closestY;
      }
    }
  }

  if (bestPen === -Infinity) return null;

  const wCos = Math.cos(grid.transform.dir);
  const wSin = Math.sin(grid.transform.dir);

  c.nx = bestNX * wCos - bestNY * wSin;
  c.ny = bestNX * wSin + bestNY * wCos;
  c.penetration = bestPen;
  c.x = grid.transform.pos.x + (bestCX * wCos - bestCY * wSin);
  c.y = grid.transform.pos.y + (bestCX * wSin + bestCY * wCos);
  c.a = grid;
  c.b = circle;
  return c;
});


PhysicsManager.addCollisionPair("ColliderGrid", "ColliderRect", (grid, rect, c) => {
  const cos = Math.cos(-grid.transform.dir);
  const sin = Math.sin(-grid.transform.dir);

  // Transform rect corners into grid local space to get candidate cells
  const obb = getOBBFast(rect);
  let minLX = Infinity, maxLX = -Infinity;
  let minLY = Infinity, maxLY = -Infinity;
  for (let i = 0; i < 8; i += 2) {
    const wx = obb.corners[i]   - grid.transform.pos.x;
    const wy = obb.corners[i+1] - grid.transform.pos.y;
    const lx = wx * cos - wy * sin;
    const ly = wx * sin + wy * cos;
    if (lx < minLX) minLX = lx;
    if (lx > maxLX) maxLX = lx;
    if (ly < minLY) minLY = ly;
    if (ly > maxLY) maxLY = ly;
  }

  const minCX = Math.floor(minLX);
  const maxCX = Math.floor(maxLX);
  const minCY = Math.floor(minLY);
  const maxCY = Math.floor(maxLY);

  const wCos = Math.cos(grid.transform.dir);
  const wSin = Math.sin(grid.transform.dir);

  // Rect-rect SAT function (reuse existing pair)
  const rectRectFn = PhysicsManager.collisionPairs.get("ColliderRect")?.get("ColliderRect");
  if (!rectRectFn) return null;

  let bestPen = -Infinity;
  let bestNX = 0, bestNY = 0, bestCX = 0, bestCY = 0;

  for (let cy = minCY; cy <= maxCY; cy++) {
    for (let cx = minCX; cx <= maxCX; cx++) {
      if (cx < 0 || cy < 0 || cx >= grid.size.x || cy >= grid.size.y) continue;
      if (!grid.g[cx + cy * grid.size.x]) continue;

      const cellLocalX = cx + 0.5;
      const cellLocalY = cy + 0.5;
      const cellWX = grid.transform.pos.x + (cellLocalX * wCos - cellLocalY * wSin);
      const cellWY = grid.transform.pos.y + (cellLocalX * wSin + cellLocalY * wCos);

      // Fake ColliderRect for this cell
      const cellCollider = {
        r: Math.SQRT2 * 0.5,
        size: { x: 1, y: 1 },
        transform: { pos: { x: cellWX, y: cellWY }, dir: grid.transform.dir },
        _cornercache: new Float32Array(8)
      };

      const result = rectRectFn(cellCollider, rect, {});
      if (!result) continue;

      if (result.penetration > bestPen) {
        bestPen = result.penetration;
        bestNX = result.nx; bestNY = result.ny;
        bestCX = result.x;  bestCY = result.y;
      }
    }
  }

  if (bestPen === -Infinity) return null;

  c.nx = bestNX; c.ny = bestNY;
  c.penetration = bestPen;
  c.x = bestCX; c.y = bestCY;
  c.a = grid; c.b = rect;
  return c;
});