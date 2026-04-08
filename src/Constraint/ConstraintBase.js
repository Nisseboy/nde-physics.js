class ConstraintBase extends Serializable {
  constructor(a, b) {
    super();

    this.a = a;
    this.b = b;

    this.aIndex = undefined;
    this.bIndex = undefined;
  }

  solve() {

  }

  from(data) {
    super.from(data);

    this.aIndex = data.aIndex;
    this.bIndex = data.bIndex;

    return this;
  }
}