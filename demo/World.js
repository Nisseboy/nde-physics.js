class World extends Serializable {
  constructor() {
    super();

    this.obs = [];
    this.constraints = [];
  }

  addOb(ob) {
    this.obs.push(ob);
    physics.addOb(ob);
  }
  addConstraint(constraint) {
    this.constraints.push(constraint);
    physics.addConstraint(constraint);

    if (constraint.a == undefined) {
      constraint.a = this.obs[constraint.aIndex];
      constraint.b = this.obs[constraint.bIndex];
    }
    if (constraint.aIndex == undefined) {
      constraint.aIndex = this.obs.indexOf(constraint.a);
      constraint.bIndex = this.obs.indexOf(constraint.b);
    }
  }

  load() {
    physics.clearObs();
    physics.clearConstraints();
    for (let e of this.obs) {physics.addOb(e)}
    for (let e of this.constraints) {physics.addConstraint(e)}
    
  }

  from(data) {
    super.from(data);

    for (let e of data.obs) {this.addOb(cloneData(e))}
    for (let e of data.constraints) {this.addConstraint(cloneData(e))}

    return this;
  }
}