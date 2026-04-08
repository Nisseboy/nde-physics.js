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