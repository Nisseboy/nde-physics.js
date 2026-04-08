class PhysicsGrid {
  constructor() {
    this.growFactor = 0.3;

    this.min = new Vec(0, 0);
    this.max = new Vec(1, 1);
    this.size = new Vec(1, 1);
    this.gridSize = new Vec(1, 1);
    this.cellSize = 1;
    this.invCellSize = 1;

    this.grid = [[]];

    this.obs = [];
    this.queued = [];
  }

  addOb(ob) {
    this.obs.push(ob);
    this.queued.push(ob);
  }
  removeOb(ob) {
    let index = this.obs.findIndex(ob);
    if (index != -1) {
      this.obs.splice(index, 1);
      let gridIndex = this.getGridIndex(ob);
      let cell = this.grid[gridIndex];
      if (cell) {
        let cellIndex = cell.findIndex(ob);
        if (cellIndex != -1) {
          cell.splice(cellIndex, 1);
        }
      }
    }
  }
  clearObs() {
    this.obs = [];
    this.grid = [[]];
  }


  restructure(min, max, cellSize, iterations) {
    let hasChanged = false;
    
    let growFactor2 = 1 + this.growFactor;
    let growX = this.size.x * this.growFactor;
    let growY = this.size.y * this.growFactor;

    if (min.x < this.min.x) {
      this.min.x -= growX;
      hasChanged = true;      
    }
    if (min.y < this.min.y) {
      this.min.y -= growY;
      hasChanged = true;
    }
    if (max.x > this.max.x) {
      this.max.x += growX;
      hasChanged = true;
    }
    if (max.y > this.max.y) {
      this.max.y += growX;
      hasChanged = true;
    }
    if (cellSize > this.cellSize) {
      this.cellSize *= growFactor2;
      hasChanged = true;
    }


    if (iterations < 10) {
      let shrinkFactor = 1 - 1 / growFactor2;
      let shrinkFactor2 = 1 - shrinkFactor
      let shrinkX = Math.max(this.size.x * shrinkFactor, this.cellSize);
      let shrinkY = Math.max(this.size.y * shrinkFactor, this.cellSize);

      if (this.min.x + shrinkX < min.x) {
        this.min.x += shrinkX;
        hasChanged = true;
      }
      if (this.min.y + shrinkY < min.y) {
        this.min.y += shrinkY;
        hasChanged = true;
      }
      if (this.max.x - shrinkX > max.x) {
        this.max.x -= shrinkX;
        hasChanged = true;
      }
      if (this.max.y - shrinkY > max.y) {
        this.max.y -= shrinkY;
        hasChanged = true;
      }
      if (cellSize < this.cellSize * shrinkFactor2) {
        this.cellSize *= shrinkFactor2;
        hasChanged = true;        
      }
    }
    
    
    
    if (!hasChanged) return false;




    this.size.from(this.max).subV(this.min);
    let diff = this.size._();
    this.gridSize = this.size.div(this.cellSize).ceil();
    this.size = this.gridSize._mul(this.cellSize);
    diff.subV(this.size).mul(-0.5);
    this.min.subV(diff);
    this.max.addV(diff);
    
    this.invCellSize = 1 / this.cellSize;
    this.grid = new Array(this.gridSize.x * this.gridSize.y).fill(undefined).map(()=>[]);
    
    return true;
  }

  getGridIndex(ob) {
    return Math.floor((ob.pos.y - this.min.y) * this.invCellSize) * this.gridSize.x + Math.floor((ob.pos.x - this.min.x) * this.invCellSize);
  }

  partition() {
    let max = new Vec(-Infinity, -Infinity);
    let min = new Vec(Infinity, Infinity);
    let cellSize = 0;

    for (let i = 0; i < this.obs.length; i++) {
      let e = this.obs[i];

      max.x = Math.max(max.x, e.pos.x);
      max.y = Math.max(max.y, e.pos.y);
      min.x = Math.min(min.x, e.pos.x);
      min.y = Math.min(min.y, e.pos.y);

      cellSize = Math.max(cellSize, e.collider.r * 2);
    }

    if (max.x == -Infinity) max.x = 10;
    if (max.y == -Infinity) max.y = 10;
    if (min.x == Infinity) min.x = 0;
    if (min.y == Infinity) min.y = 0;
    if (min.x == max.x) max.x += 10;
    if (min.y == max.y) max.y += 10;
    if (cellSize == 0) cellSize = 1;

    let iterations = 0;
    while (this.restructure(min, max, cellSize, iterations)) {iterations++}

    
    
    if (iterations != 0) {
      this.queued.length = 0;
      for (let i = 0; i < this.obs.length; i++) {
        this.grid[this.getGridIndex(this.obs[i])].push(this.obs[i]);
      }
    } else {
      for (let i = 0; i < this.grid.length; i++) {
        let cell = this.grid[i];
        
        for (let j = 0; j < cell.length; j++) {
          if (this.getGridIndex(cell[j]) == i) continue;

          this.queued.push(cell[j]);
          cell.splice(j, 1);
          j--;
        }
      }
    } 
    
    
    for (let i = 0; i < this.queued.length; i++) {
      this.grid[this.getGridIndex(this.queued[i])].push(this.queued[i]);
    }
    this.queued.length = 0;
  }

  getPotentialCollisions(potentialCollisions = [], onlyStatic = false) {    

    let a, aStatic, i, j, cell, cell2, goRight, goDown;
    for (let cellIndex = 0; cellIndex < this.grid.length; cellIndex++) {
      cell = this.grid[cellIndex];
      
      
      for (i = 0; i < cell.length; i++) {
        a = cell[i];
        aStatic = a.static;


        for (j = i + 1; j < cell.length; j++) {
          if (!onlyStatic || aStatic != cell[j].static) {
            potentialCollisions.push(a, cell[j]);
          }
        }

        goRight = cellIndex % this.gridSize.x < this.gridSize.x - 1;
        goDown = Math.floor(cellIndex / this.gridSize.x) < this.gridSize.y - 1;

        if (goRight) {
          cell2 = this.grid[cellIndex + 1];
          for (j = 0; j < cell2.length; j++) {
            if (!onlyStatic || aStatic != cell2[j].static) {
              potentialCollisions.push(a, cell2[j]);
            }
          }
        }
        if (goDown) {
          cell2 = this.grid[cellIndex + this.gridSize.x];
          for (j = 0; j < cell2.length; j++) {
            if (!onlyStatic || aStatic != cell2[j].static) {
              potentialCollisions.push(a, cell2[j]);
            }
          }
        }

        if (goRight && goDown) {
          cell2 = this.grid[cellIndex + this.gridSize.x + 1];
          for (j = 0; j < cell2.length; j++) {
            if (!onlyStatic || aStatic != cell2[j].static) {
              potentialCollisions.push(a, cell2[j]);
            }
          }
        }
      }
    }

    return potentialCollisions;
  }
}