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