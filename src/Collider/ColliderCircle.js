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