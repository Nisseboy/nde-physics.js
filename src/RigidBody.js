class RigidBody extends Component {
  constructor(props = {}) {
    super();

    this.vel = props.vel || new Vec(0, 0);
    this.av = props.av || 0;
    
    this.force = new Vec(0, 0);
    this.torque = 0;


    this.friction = props.friction ?? Math.sqrt(0.5) //Friction coefficient of this to itself is 0.5
    this.restitution = props.restitution ?? Math.sqrt(0.2) //Restitution coefficient of this to itself is 0.5

    this.mass = props.mass ?? 10;
    this.moi = props.moi ?? 16;
    this.static = props.static ?? false;

    this.lastAcc = new Vec(0, 0);
    this.lastAA = 0;
  }

  set mass(value) {
    this._mass = value;
    this.updateInvs();
  }
  get mass() {
    return this._mass;
  }
  set moi(value) {
    this._moi = value;
    this.updateInvs();
  }
  get moi() {
    return this._moi;
  }
  set static(value) {
    this._static = value;
    this.updateInvs();
  }
  get static() {
    return this._static;
  }


  updateInvs() {
    if (this.static) {
      this.invMass = 0;
      this.invMoi = 0;
      return;
    }
    this.invMoi = 1 / this.moi;
    this.invMass = 1 / this.mass;
  }



  integrate(dt) {
    if (this.invMass != 0) {
      let newVelHalf = this.lastAcc._mul(dt * 0.5).addV(this.vel);
      this.transform.pos = newVelHalf._mul(dt).addV(this.transform.pos);
      this.lastAcc.from(this.force).mul(this.invMass);
      this.vel.from(this.lastAcc).mul(dt * 0.5).addV(newVelHalf);
    }

    if (this.invMoi != 0) {
      let newAVHalf = this.lastAA * dt * 0.5 + this.av;
      this.transform.dir += newAVHalf * dt;
      this.lastAA = this.torque * this.invMoi;
      this.av = this.lastAA * dt * 0.5 + newAVHalf;    
    }
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

    let x = px - this.transform.pos.x;
    let y = py - this.transform.pos.y;

    // Cross product for torque (scalar in 2D)
    let torque = x * iy - y * ix;

    this.av += torque * this.invMoi;
  }


  from(data) {
    super.from(data);

    this.vel = new Vec().from(data.vel);
    this.av = data.av;

    this.force = new Vec().from(data.force);
    this.torque = data.torque;

    this.mass = data._mass;
    this.moi = data._moi;
    this.static = data._static;

    this.friction = data.friction;
    this.restitution = data.restitution;

    this.lastAcc = new Vec().from(data.lastAcc);
    this.lastAA = data.lastAA;

    return this;
  }
}