class PhysicsLayer {
  constructor(props) {
    this.filterF = props.filterF;
    this.size = props.size ?? 1;

    this.colliders = [];
    this.map = {};
  }

  addCollider(collider) {
    let i = this.colliders.indexOf(undefined);
    if (i == -1) i = this.colliders.length;

    this.colliders[i] = collider;

    let index = this.getIndex(collider.transform.pos);
    collider.i = index;

    if (!this.map[index]) this.map[index] = [];
    this.map[index].push(i);    
  }
  removeCollider(collider) {
    let i = this.colliders.indexOf(collider);
    if (i == -1) return;

    let cell = this.map[this.colliders[i].i];
    cell.splice(cell.indexOf(i), 1);

    this.colliders[i] = undefined;
  }

  moveCollider(colliderIndex, i) {
    let collider = this.colliders[colliderIndex];

    if (collider.i == i) return;

    let cell = this.map[collider.i];
    cell.splice(cell.indexOf(colliderIndex), 1);

    if (!this.map[i]) this.map[i] = [];
    this.map[i].push(colliderIndex);
    collider.i = i;

    
  }

  getIndex(pos) {
    return this.getIndexFast(pos.x, pos.y);
  }
  
  getIndexFast(x, y) {
    //return Math.floor(x / this.size) + "_" + Math.floor(y / this.size);

    let h1 = Math.floor(x / this.size) * 0x85ebca6b;
    let h2 = Math.floor(y / this.size) * 0x127842;
    
    h1 ^= h1 >>> 16;
    h2 ^= h2 >>> 16;
    
    return (h1 ^ h2) >>> 0;
  }

  partition() {
    let collider, index;
    for (let i = 0; i < this.colliders.length; i++) {
      collider = this.colliders[i];
      if (collider == undefined) continue;

      index = this.getIndex(collider.transform.pos);
      this.moveCollider(i, index);
    }
  }

  getPotentialCollisions(layer, collisions = []) {
    let size = layer.size;

    if (layer == this) {
      let visited = {};
      let i,j,c1,x,y,coll,cell,cell2,cell3,cell4,cell5,hash;

      for (let I = 0; I < this.colliders.length; I++) {
        coll = this.colliders[I];
        if (coll == undefined) continue;

        hash = coll.i;

        if (visited[hash]) continue;
        visited[hash] = true;

        cell = this.map[hash];
        x = coll.transform.pos.x;
        y = coll.transform.pos.y;
        cell2 = this.map[this.getIndexFast(x + size, y)];
        cell3 = this.map[this.getIndexFast(x, y + size)];
        cell4 = this.map[this.getIndexFast(x + size, y + size)];
        cell5 = this.map[this.getIndexFast(x - size, y + size)];
        
        if (size != 100 && nde.getKeyPressed("Debug Mode")) debugger

        for (i = 0; i < cell.length; i++) {
          c1 = this.colliders[cell[i]];

          for (j = i+1; j < cell.length; j++) {
            collisions.push(c1, this.colliders[cell[j]]);
          }

          for (j = 0; j < cell2?.length; j++) {
            collisions.push(c1, this.colliders[cell2[j]]);
          }
          for (j = 0; j < cell3?.length; j++) {
            collisions.push(c1, this.colliders[cell3[j]]);
          }
          for (j = 0; j < cell4?.length; j++) {
            collisions.push(c1, this.colliders[cell4[j]]);
          }
          for (j = 0; j < cell5?.length; j++) {
            collisions.push(c1, this.colliders[cell5[j]]);
          }
        }
      }
      
      return collisions;
    }


    
    let indexes = [];
    let coll, x, y, i, j, cell2;
    for (i = 0; i < this.colliders.length; i++) {
      coll = this.colliders[i];
      if (coll == undefined) continue;

      x = coll.transform.pos.x;
      y = coll.transform.pos.y;

      indexes[0] = layer.getIndexFast(x - size, y - size);
      indexes[1] = layer.getIndexFast(x - size, y);
      indexes[2] = layer.getIndexFast(x - size, y + size);
      indexes[3] = layer.getIndexFast(x, y - size);
      indexes[4] = layer.getIndexFast(x, y);
      indexes[5] = layer.getIndexFast(x, y + size);
      indexes[6] = layer.getIndexFast(x + size, y - size);
      indexes[7] = layer.getIndexFast(x + size, y);
      indexes[8] = layer.getIndexFast(x + size, y + size);


      for (j = 0; j < 9; j++) {
        cell2 = layer.map[indexes[j]];
        
        if (!cell2) continue;
        for (let k = 0; k < cell2.length; k++) {
          collisions.push(this.colliders[i], layer.colliders[cell2[k]]);
        }
      }
    }

    return collisions;


    if (layer == this) {
      let a,b;
      for (let i = 0; i < collisions.length; i += 2) {
        a = collisions[i];
        b = collisions[i+1];

        if (a != b && a > b) continue;

        collisions[i] = collisions[collisions.length - 2];
        collisions[i + 1] = collisions[collisions.length - 1];
        i -= 2;

        collisions.pop();
        collisions.pop();
      }
    }
    
  }
}