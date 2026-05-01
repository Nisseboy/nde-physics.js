let allRooms = [];

let noiseOverlay;
function createNoiseOverlay() {
  noiseOverlay = new Img(new Vec(432, 432 * (9/16)));
  let imageData = noiseOverlay.ctx.getImageData(0, 0, noiseOverlay.size.x, noiseOverlay.size.y);
  let pxls = imageData.data;
  for (let i = 0; i < noiseOverlay.size.x * noiseOverlay.size.y * 4;) {
    let c = Math.random() * 255;
    pxls[i++] = c;
    pxls[i++] = c;
    pxls[i++] = c;
    pxls[i++] = (Math.random() > 0.5) ? 10 : 0;
  }
  noiseOverlay.ctx.putImageData(imageData, 0, 0);
}
createNoiseOverlay();

class Grid extends Component {
  constructor(props = {}) {
    super();

    this.size = props.size || vecOne;
    this.g = props.g || new Array(this.size.x * this.size.y).fill(0);
    this.cam = undefined;

    this.tileEntities = {};

    this.mappedMaterials = [];
  }

  start() {
    this.ob.grid = this;

    this.collider = this.getComponent(ColliderGrid);
    if (this.collider) {
      this.ob.transform.pos.from(this.size).mul(0.5);
      this.collider.g = this.g;
    }
  }

  render() {
    let cam = this.cam;
    renderer.set("stroke", "rgba(0, 0, 0, 0)")


    let bounds = new Vec(
      Math.max(Math.floor(cam.pos.x - cam.w / 2), 0),
      Math.max(Math.floor(cam.pos.y - cam.w / 2 * cam.ar), 0),
      Math.min(Math.floor(cam.pos.x + cam.w / 2 + 1), this.size.x),
      Math.min(Math.floor(cam.pos.y + cam.w / 2 * cam.ar + 1), this.size.y),
    );
    let p = new Vec(0, 0); 
    renderer.set("fill", "rgba(255, 255, 255, 1)")
    for (p.x = bounds.x; p.x < bounds.z; p.x++) {
      for (p.y = bounds.y; p.y < bounds.w; p.y++) {
        if (!this.g[p.x + p.y * this.size.x]) continue;
        
        renderer.rect(p, vecOne);
      }
    }

    renderer._(() => {
      let camSize = new Vec(cam.w, cam.w * cam.ar);
      let noiseImg = noiseOverlay;
      let pos = cam.pos._divV(camSize).floor().mulV(camSize);

      renderer.translate(pos);
      renderer.image(noiseImg, camSize._div(-2), camSize);
      renderer.translate(new Vec(camSize.x, 0));
      renderer.image(noiseImg, camSize._div(-2), camSize);
      renderer.translate(new Vec(0, camSize.y));
      renderer.image(noiseImg, camSize._div(-2), camSize);
      renderer.translate(new Vec(-camSize.x, 0));
      renderer.image(noiseImg, camSize._div(-2), camSize);
    });
  }

  random() {
    this.g = new Array(this.size.x * this.size.y)
    let scale = 0.2;
    for (let x = 0; x < this.size.x; x++) {
      for (let y = 0; y < this.size.y; y++) {
        this.g[x + y * this.size.x] = noise.perlin2(x * scale, y * scale) > 0.3 ? 0 : 1;
      }
    }

    this.g = [
      1,1,0,1,1,
      1,0,0,0,1,
      1,0,0,0,1,
      1,0,1,0,1,
      1,1,1,1,1,
    ];
  }

  placeRoom(room, pos) {
    let grid2 = room.getComponent(Grid);
    let g = grid2.g;
    let size = grid2.size;

    let i, thisI, mat;

    for (let x = 0; x < size.x; x++) {
      for (let y = 0; y < size.y; y++) {
        i = x + y * size.x;
        thisI = pos.x + x + (pos.y + y) * this.size.x;

        this.g[thisI] = g[i];

        mat = materials[g[i]]
        if (mat.components) {
          this.placeEntityTile(mat, new Vec(pos.x + x, pos.y + y));
        }
      }
    }

    for (let i = 0; i < room.children.length; i++) {
      let c = room.children[i].copy();
      c.getComponent(Transform).pos.addV(pos);
      c.randomizeId();
      
      this.ob.children[0].appendChild(c);
    }
  }
  placeEntityTile(mat, pos) {
    pos = pos._floor();
    let i = pos.x + pos.y * this.size.x;
    let ob = new Ob({pos: pos.add(0.5), name: mat.fullName}, mat.components.map(e=>e.copy()));
    this.ob.children[0].appendChild(ob);

    this.tileEntities[i] = ob.id;
  }
  removeEntityTile(pos) {
    pos = pos._floor();
    let i = pos.x + pos.y * this.size.x;

    this.g[i] = 0;
    let ob = idLookup[this.tileEntities[i]];
    delete this.tileEntities[i];    
  }
  fenceOff() {
    let id = getMaterialId("fence/chain");
    for (let x = 0; x < this.size.x; x++) {
      for (let y = 0; y < this.size.y; y++) {
        if (x == 0 || y == 0 || x == this.size.x - 1 || y == this.size.y - 1) this.g[x + y * this.size.x] = id;
      }
    }
  }

  updateLights(pos, d = 1) {
    let p, sqd;
    lights.forEach(light => {
      p = light.pos ?? light.transform.pos;
      sqd = (pos.x - p.x) ** 2 + (pos.y - p.y) ** 2;

      if (sqd < Math.max(light.maxR - d, 0) ** 2) light.cached = false;    
    });
  }

  raycast(pos, dirVec, maxDist, tag) {return this.raycastFast(pos.x, pos.y, dirVec.x, dirVec.y, maxDist, tag)}
  raycastFast(posx, posy, dirx, diry, maxDist = 100, tag = "solid") {
    const gridW = this.size.x;
    const gridH = this.size.y;

    // current grid cell
    let mapX = Math.floor(posx);
    let mapY = Math.floor(posy);

    let mat, insideGrid, j;

    /*
    mat = this.g[mapX + mapY * gridW];
    if (materials[mat]?.[tag]) {
      return {
        x: posx,
        y: posy ,
        d: 0,
        isHor: false,
        mat: mat,
      };
    }*/

    // step direction
    const stepX = dirx < 0 ? -1 : 1;
    const stepY = diry < 0 ? -1 : 1;

    // avoid division by zero
    const deltaX = dirx !== 0 ? Math.abs(1 / dirx) : 1e6;
    const deltaY = diry !== 0 ? Math.abs(1 / diry) : 1e6;

    // distance to first grid boundary
    let tMaxX =
      dirx > 0
        ? (mapX + 1 - posx) * deltaX
        : (posx - mapX) * deltaX;

    let tMaxY =
      diry > 0
        ? (mapY + 1 - posy) * deltaY
        : (posy - mapY) * deltaY;

    let t = 0;
    let hitVertical = false;

    for (let i = 0; i < 80; i++) {
      if (tMaxX < tMaxY) {
        t = tMaxX;
        tMaxX += deltaX;
        mapX += stepX;
        hitVertical = true;
      } else {
        t = tMaxY;
        tMaxY += deltaY;
        mapY += stepY;
        hitVertical = false;
      }

      if (t >= maxDist) break;

      insideGrid = mapX >= 0 && mapX < gridW && mapY >= 0 && mapY < gridH;
      if (!insideGrid) {
        if (
          mapX < 0 && dirx < 0 || 
          mapY < 0 && diry < 0 || 
          mapX >= gridW && dirx > 0 || 
          mapY >= gridH && diry > 0
        ) break;
        continue;
      }

      j = mapX + mapY * gridW;
      mat = this.g[j];
      if (materials[mat]?.[tag]) {
        return {
          x: posx + dirx * t,
          y: posy + diry * t,
          i: j,
          d: t,
          isHor: !hitVertical,
          mat: mat,
        };
      }
    }
  }

  createMask(pos, maxR, ar, texture) {
    let ctx = texture.ctx;

    if (pos.x <= 0 || pos.y <= 0 || pos.x >= this.size.x - 1 || pos.y >= this.size.y - 1) {
      ctx.fillStyle = "rgb(255, 255, 255)";
      ctx.fillRect(0, 0, texture.size.x, texture.size.y);
      return;
    }

    ctx.fillStyle = "rgb(255, 255, 255)";
    ctx.strokeStyle = "rgb(255, 255, 255)";
    ctx.lineWidth = 15;
    ctx.beginPath();

    let dirStep = Math.PI * 2 / settings.visionSamples; 
    let scaling = 1 / maxR;
    let invAr = 1 / ar;
    let res, cos, sin, d;
    for (let i = 0; i < settings.visionSamples + 1; i++) {
      cos = Math.cos(i * dirStep);
      sin = Math.sin(i * dirStep)

      res = this.raycastFast(pos.x, pos.y, cos, sin, maxR, "opaque");
      d = res?.d || maxR;
       
      ctx.lineTo(texture.size.x * 0.5 * (1 + cos * d * scaling), texture.size.y * 0.5 * (1 + sin * d * scaling * invAr));
    }

    ctx.fill();
    ctx.stroke();

    return texture;
  }

  getMat(v) {
    if (this.inBounds(v)) return this.g[this.getIndex(v)];
  }
  setMat(v, matIndex) {
    if (this.inBounds(v)) this.g[this.getIndex(v)] = matIndex;
  }
  getIndex(v) {
    if (this.inBounds(v)) return Math.floor(v.x) + Math.floor(v.y) * this.size.x;
  }
  inBounds(v) {
    return (v.x >= 0 && v.x < this.size.x && v.y >= 0 && v.y < this.size.y);
  }

  mapMaterials() {
    this.mappedMaterials.length = 0;
    
    for (let i = 0; i < this.g.length; i++) {
      this.mappedMaterials[this.g[i]] = materials[this.g[i]].fullName;
    }
  }
  unmapMaterials() {
    if (this.mappedMaterials.length == 0) return;

    let mats = this.mappedMaterials.map(e=>materials.findIndex(ee=>ee.fullName==e));
    
    for (let i = 0; i < this.g.length; i++) {
      this.g[i] = mats[this.g[i]];
    }
    
    this.mappedMaterials.length = 0;
  }


  moveEdge(edge, dir) {
    let x = edge % 2 == 0;
    let shift = edge > 1;
    

    if (shift) {
      let vec = new Vec(x ? 1 : 0, x ? 0 : 1).mul(dir);

      function move(ob) {
        for (let c of ob.children) {
          c.transform.pos.addV(vec);
          move(c);
        }
      }
      move(this.ob);
      this.cam.pos.addV(vec);
    }

    let oldSize = this.size.copy();
    let oldGrid = [...this.g];

    if (x) this.size.x += dir;
    else this.size.y += dir;

    this.g.length = this.size.x * this.size.y;
    this.g.fill(0);

    let dx = (x && shift) ? dir : 0;
    let dy = (!x && shift) ? dir : 0;

    for (let x = 0; x < oldSize.x; x++) {
      for (let y = 0; y < oldSize.y; y++) {
        if (x+dx >= this.size.x || y+dy >= this.size.y) continue;

        let kOld = x + y * oldSize.x;
        let k = (x+dx) + (y+dy) * this.size.x;

        this.g[k] = oldGrid[kOld];
      }
    }
  }


  from(data) {
    super.from(data);

    this.size = new Vec().from(data.size);
    this.g = data.g;    
    this.tileEntities = data.tileEntities;

    this.mappedMaterials = data.mappedMaterials;

    return this;
  }

  strip() {
    delete this.ob.grid;

    super.strip();
  }
}


function processRooms() {
  for (let assetName in nde.assets) {
    let split = assetName.split("/");
    if (split.splice(0, 1)[0] != "rooms") continue;
    
    let room = nde.assets[assetName];
    room.name = split.join("/");
    allRooms.push(room);
    room.getComponent(Grid).unmapMaterials();
  }
}