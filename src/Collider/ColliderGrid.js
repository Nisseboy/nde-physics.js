class ColliderGrid extends Collider {
  constructor(props = {}) {
    super();

    this._size = undefined;
    this.size = props.size ?? vecOne.copy();
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