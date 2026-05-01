
let world;
let physics;

class SceneGame extends Scene {
  constructor() {
    super();

    this.cam = new Camera(new Vec(0, 0));
    this.cam.w = 16;
  }
  loadWorld() {    
    let g = new Grid({size: new Vec(16, 9)});
    g.cam = this.cam;
    g.g = [
      1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
      1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,
      1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,
      1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,
      1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,
      1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,
      1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,
      1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,
      1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,
    ];
    this.world = new Ob({}, [
      g,
      new ColliderGrid(),
    ]);

    world = this.world;
    physics = new PhysicsManager();

    physics.iterations = 5;
    world.addComponent(physics);
  

    /*
    this.world.appendChild(
      new Ob({pos: new Vec(0, 0),}, [
        new RigidBody({vel: new Vec(1, 0)}),
        new ColliderRect({size: new Vec(0.2, 0.2)}),
      ]),
      new Ob({pos: new Vec(2, 0)}, [
        new RigidBody({static: false}),
        new ColliderRect({size: new Vec(0.2, 0.2)}),
      ]),
    );*/
    

    /*
    this.world.appendChild(
      new Ob({pos: new Vec(0, 0)}, [
        new RigidBody({vel: new Vec(1, 0)}),
        new ColliderCircle({r: 0.5}),
      ]),
      new Ob({pos: new Vec(2, 0)}, [
        new RigidBody({vel: new Vec(-1, 0)}),
        new ColliderRect({size: new Vec(1, 1)}),
      ]),
    );
    */
    this.add();


    this.world.appendChild(
      new Ob({pos: new Vec(0, 54)}, [
        new RigidBody({static: true}),
        new ColliderRect({size: new Vec(100, 100)}),
      ]),
    );

    this.world.update(1/60);

  }

  add() {
    let ob2 = new Ob({pos: new Vec(Math.random() * 16 - 8, Math.random() * 9 - 4.5), dir: Math.random() * Math.PI * 2}, [
      new RigidBody({mass: 0.2^2, moi: 0.01}),
      new ColliderCircle({r: 0.1}),
    ]);
    this.world.appendChild(ob2);
    
    let ob = new Ob({pos: new Vec(Math.random() * 16 - 8, Math.random() * 9 - 4.5), dir: Math.random() * Math.PI * 2}, [
      new RigidBody({mass: 0.2^2, moi: 0.01}),
      new ColliderRect({size: new Vec(0.2, 0.2)}),
    ]);
    this.world.appendChild(ob);
  }

  start() {

  }

  update(dt) {  
    if (true) {
      let controlled = this.world.children[1];
      let rb = controlled.getComponent(RigidBody);
      let mouseWorld = this.cam.untransformVec(nde.mouse);
      let diff = mouseWorld.subV(controlled.transform.pos);

      rb.vel.from(diff.mul(10));
    }

    if (world.children.length < 200)
      this.add();
    
    //this.world.children[0].getComponent(RigidBody).force.x = 10;
    //this.world.children[1].getComponent(RigidBody).force.x = -10;

    physics.accelerate(new Vec(0, 9.82));
    this.world.children[2].getComponent(RigidBody).force.y -= 9820;

    this.world.update(dt);

    nde.debugStats.collisionMap = physics.collisionMap.size;
  }

  render() {
    let cam = this.cam;


    renderer._(()=>{
      renderer.set("fill", "rgb(100, 100, 50)");
      renderer.rect(vecZero, renderer.size);
    });



    cam._(renderer, () => {
      renderer.set("fill", "rgba(255, 255, 255, 1)")
      this.world.render();
      //physics.renderCollisionPoints();
    });
  }
}