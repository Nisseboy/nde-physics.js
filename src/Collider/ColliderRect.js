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