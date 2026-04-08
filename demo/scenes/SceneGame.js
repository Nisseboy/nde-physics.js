let world;

let physics = new NDEPhysics();
physics.iterations = 15;


let collisionPoints = [];

class SceneGame extends Scene {
  constructor() {
    super();

    this.cam = new Camera(new Vec(0, 0));
    this.cam.w = 16;
    this.cam.renderW = nde.w;
  }
  loadWorld(w) {
    this.world = w;

    world = this.world;

    world.obs = [
      new EntityBase(
        new Vec(0, 6.5), 
        new ColliderRect(new Vec(40, 5)), 
        {mass: 0, moi: 0}, //or {static: true} same thing
      ),
      
      /*new EntityBase(
        new Vec(0, 3.2), 
        new ColliderRect(new Vec(1, 1)), 
        {vel: new Vec(0, 0), dir: 0, friction: 0.1},
      ),*/

      /*new EntityBase(
        new Vec(0, 0), 
        new ColliderRect(new Vec(1, 0.33)),
      ),
      new EntityBase(
        new Vec(0, 0), 
        new ColliderRect(new Vec(0.33, 1)),
      ),*/  
    ];
    
    for (let i = 0; i < 300; i++) {
      world.obs.push(
        new EntityBase(
          new Vec(Math.random() * 16 - 8, Math.random() * 8 - 4), 
          new ColliderRect(new Vec(0.2, 0.2)), 
          {dir: Math.random() * Math.PI * 2, vel: new Vec((Math.random()-0.5)*1, (Math.random()-0.5)*1)}
        )
      );
    }
    /*world.addConstraint(
      new ConstraintWeld(world.obs[1], world.obs[2]),
    );*/

    world.load();

    
  }

  start() {
  
  }

  inputdown(key) {
    
  }
  inputup(key) {
    
  }

  update(dt) {    
    for (let i = 1; i < world.obs.length; i++) {
      let e = world.obs[i];
      e.force.y += 10 * e.mass;
    }

    physics.update(dt);    
  }

  render() {
    let cam = this.cam;
    cam.renderW = nde.w;


    renderer._(()=>{
      renderer.set("fill", backgroundCol);
      renderer.rect(vecZero, new Vec(nde.w, nde.w / 16 * 9));
    });



    cam._(renderer, () => {
      renderer.set("fill", "rgb(255, 255, 255)");
      renderer.set("stroke", "rgb(255, 255, 255)");
      for (let i = 0; i < this.world.obs.length; i++) {
        let e = this.world.obs[i];

        e.render();
      }


      /*
      renderer.set("fill", "rgba(0, 0, 0, 0)");
      let g = physics.gridMoving;
      for (let x = g.min.x; x < g.max.x; x += g.cellSize) {
        for (let y = g.min.y; y < g.max.y; y += g.cellSize) {
          renderer.rect(new Vec(x, y), new Vec(g.cellSize, g.cellSize));
        }
      }*/
        

      for (let i = 0; i < collisionPoints.length; i++) {
        let p = collisionPoints[i];
        let pos = new Vec(p.x, p.y);

        renderer.set("fill", "rgb(255, 0, 0)");
        renderer.set("stroke", "rgb(255, 0, 0)");
        renderer.circle(pos, 0.1);  
        
        renderer.line(pos, pos._addV(new Vec(p.nx, p.ny)));

        renderer.set("fill", "rgb(0, 255, 0)");
        renderer.circle(p.a.pos, 0.1);          
      }
    });
  }
}