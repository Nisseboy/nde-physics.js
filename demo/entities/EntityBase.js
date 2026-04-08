class EntityBase extends PhysicsObject {
  constructor(pos, collider, props = {}) {
    super(pos, collider, props);
  }

  render() {
    renderer._(()=>{
      renderer.translate(this.pos);
      if (this.dir) renderer.rotate(this.dir);

      renderer.rect(this.collider.halfSize._mul(-1), this.collider.size);
    });
  }

  from(data) {
    super.from(data);

    return this;
  }
}