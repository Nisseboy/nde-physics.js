class PhysicsObject extends Serializable {
  constructor(pos, collider, props = {}) {
    super();

    this.pos = pos;
    this.dir = props.dir || 0;

    this.vel = props.vel || new Vec(0, 0);
    this.av = props.av || 0;
    
    this.force = new Vec(0, 0);
    this.torque = 0;

    this.collider = collider;

    

    this.mass = props.mass;
    this.moi = props.moi;
    this.friction = (props.friction != undefined) ? props.friction : Math.sqrt(0.5) //Friction coefficient of this to itself is 0.5

    if (this.mass == undefined) this.collider.estimateParentMass(this);
    if (this.moi == undefined) this.collider.estimateParentMOI(this);

    this.invMass = undefined;
    this.invMoi = undefined;
    this.static = undefined;
    this.calculateProps();



    this.lastAcc = new Vec(0, 0);
    this.lastAA = 0;
    this._cornercache = new Float32Array(8);
  }





  integrate(dt) {
    if (this.invMass == 0) return;


    let newVelHalf = this.lastAcc._mul(dt * 0.5).addV(this.vel);
    this.pos = newVelHalf._mul(dt).addV(this.pos);
    this.lastAcc.from(this.force).mul(this.invMass);
    this.vel.from(this.lastAcc).mul(dt * 0.5).addV(newVelHalf);


    let newAVHalf = this.lastAA * dt * 0.5 + this.av;
    this.dir += newAVHalf * dt;
    this.lastAA = this.torque * this.invMoi;
    this.av = this.lastAA * dt * 0.5 + newAVHalf;    
  }

  applyImpulse(impulse) {
    this.applyImpulseFast(impulse.x, impulse.y)
  }
  applyImpulseFast(x, y) {
    this.vel.x += x * this.invMass;
    this.vel.y += y * this.invMass;
  }

  applyImpulseAtPoint(impulse, pos) {
    this.applyImpulseAtPointFast(impulse.x, impulse.y, pos.x, pos.y)
  }
  applyImpulseAtPointFast(ix, iy, px, py) {
    this.vel.x += ix * this.invMass;
    this.vel.y += iy * this.invMass;

    let x = px - this.pos.x;
    let y = py - this.pos.y;

    // Cross product for torque (scalar in 2D)
    let torque = x * iy - y * ix;

    this.av += torque * this.invMoi;
  }



  calculateProps() {
    this.collider.calculateProps();

    if (this.static) {
      this.mass = 0;
      this.moi = 0;
    }

    this.invMass = this.mass ? 1 / this.mass : 0;
    this.invMoi = this.moi ? 1 / this.moi : 0;
    this.static = this.mass == 0
  }


  from(data) {
    super.from(data);

    this.pos = new Vec().from(data.pos);
    this.dir = data.dir;

    this.vel = new Vec().from(data.vel);
    this.av = data.av;

    this.force = new Vec().from(data.force);
    this.torque = data.torque;

    this.collider = cloneData(data.collider);



    this.mass = data.mass;
    this.moi = data.moi;
    this.friction = data.friction;

    this.invMass = data.invMass;
    this.invMoi = data.invMoi;
    this.static = data.static;



    this.lastAcc = new Vec(0, 0);
    this.lastAA = 0;



    return this;
  }
}