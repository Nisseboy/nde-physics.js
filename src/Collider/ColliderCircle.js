class ColliderCircle extends Collider {
  constructor(props = {}) {
    super();

    this.r = props.r ?? 0.5;
  }
  
  render() {
    nde.renderer._(() => {
      nde.renderer.translate(this.transform.pos);
      if (this.transform.dir) nde.renderer.rotate(this.transform.dir);

      nde.renderer.circle(vecZero, this.r);
    });
  }

}

PhysicsManager.addCollisionPair("ColliderCircle", "ColliderCircle", (a, b, c) => {
  // assume pos objects with x,y and helpers omitted; using raw arithmetic
  const dx2 = b.transform.pos.x - a.transform.pos.x;
  const dy2 = b.transform.pos.y - a.transform.pos.y;
  const mag2 = Math.sqrt(dx2*dx2 + dy2*dy2) || EPS;
  const rsum2 = a.r + b.r;
  const penetration = rsum2 - mag2;
  if (penetration <= 0) { return null; }
  c.penetration = penetration;
  c.nx = dx2 / mag2; c.ny = dy2 / mag2;
  // contact point: move from a towards b by a.r along the normal
  c.x = a.transform.pos.x + c.nx * (a.r - penetration*0.5);
  c.y = a.transform.pos.y + c.ny * (a.r - penetration*0.5);
  return c;
});