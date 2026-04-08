class ConstraintWeld extends ConstraintBase {
  constructor(a, b) {
    super(a, b);
    
    this.offsetPos = undefined;
    this.offsetDir = undefined;

    if (a) this.initOffset();
  }

  initOffset() {
    this.offsetPos = this.a.pos._subV(this.b.pos);
    this.offsetDir = this.a.dir - this.b.dir;
  }

  solve() {
    let x = (this.a.pos.x - this.b.pos.x) - this.offsetPos.x;
    let y = (this.a.pos.y - this.b.pos.y) - this.offsetPos.y;
    let dir = (this.a.dir - this.b.dir) - this.offsetDir;

    this.a.applyImpulseFast(-x, -y);
    this.b.applyImpulseFast(x, y);
    this.a.av -= dir * this.a.invMass;
    this.b.av += dir * this.b.invMass;
  }

  from(data) {
    super.from(data);

    this.offsetPos = new Vec().from(data.offsetPos);
    this.offsetDir = data.offsetDir;

    return this;
  }
}