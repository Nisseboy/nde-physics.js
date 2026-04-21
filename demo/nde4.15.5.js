
/*
This is a built version of nde (Nils Delicious Engine) and is all the source files stitched together, https://github.com/nisseboy/nde.js


*/
/* src/Serializable.js */
class Serializable {
  constructor() {
    this.type = this.constructor.name;
  }

  from(data) {
    return this;
  }

  copy() {
    return cloneData(this);
  }
  strip() {}
  serialize() {
    let ob = this.copy();

    ob.strip();

    return JSON.stringify(ob);
  }
}

function cloneData(data, typeOverride = undefined) {
  if (typeof data == "string") data = JSON.parse(data);

  let type = data.type;
  if (!type) type = data.constructor.name;
  if (typeOverride) type = typeOverride;
  
  return new (eval(type))().from(data);
}





/* src/Vec.js */
/*

All of these are valid:
let v = new Vec();
let v = new Vec(0);
let v = new Vec(0, 0);
let v = new Vec(0, 0, 0);
let v = new Vec(0, 0, 0, 0);

generally operations on vectors follow this style:
  * add: adds a scalar to each dimension of the vector
  * addV: adds two vectors together one dimension at a time, X3 = X2 + X1, Y3 = Y2 + Y1, Z3 = Z2 + Z1
  * _add: performs add but instead of mutating the vector it creates a copy of it first so the original vector is unchanged
  * _addV: performs addV but instead of mutating the vector it creates a copy of it first so the original vector is unchanged

*/

class Vec extends Serializable {
  constructor(x, y, z, w) {
    super();
    this.x = x;
    this.y = y;
    this.z = z;
    this.w = w;
  }

  get r() {
    return this.x;
  }
  set r(value) {
    this.x = value;
  }

  get g() {
    return this.y;
  }
  set g(value) {
    this.y = value;
  }

  get b() {
    return this.z;
  }
  set b(value) {
    this.z = value;
  }

  get a() {
    return this.w;
  }
  set a(value) {
    this.w = value;
  }

  /**
   * Creates a copy of this vector
   * 
   * @return {Vec} new vector
   */
  copy() {
    return new Vec(this.x, this.y, this.z, this.w);
  }
  /**
   * Creates an array representation of this vector
   * 
   * @return {Array<number>} array
   */
  toArray() {
    let a = [];
    if (this.x != undefined) a.push(this.x);
    if (this.y != undefined) a.push(this.y);
    if (this.z != undefined) a.push(this.z);
    if (this.w != undefined) a.push(this.w);
    
    return a;
  }
  /**
   * Creates a string representation of this vector
   * 
   * @return {string} formatted string
   */
  toString() {
    let a = "(";
    if (this.x != undefined) a += this.x;
    if (this.y != undefined) a += ", " + this.y;
    if (this.z != undefined) a += ", " + this.z;
    if (this.w != undefined) a += ", " + this.w;
    
    return a + ")";
  }
  /**
   * Checks if other vector is equal to this vector
   * 
   * @param {Vec} other
   * @return {boolean} isEqual
   */
  isEqualTo(other) {
    return this.x == other.x && this.y == other.y && this.z == other.z && this.w == other.w
  }

  /**
   * Checks if this is within either bounding box vec(x, y, width, height)
   * 
   * @param {Vec} boundingBox
   * @return {boolean} isWithin
   */
  isWithin(boundingBox) {
    if (this.x < boundingBox.x || this.x > boundingBox.x + boundingBox.z) return false;
    if (this.y < boundingBox.y || this.y > boundingBox.y + boundingBox.w) return false;
    return true;
  }
  /**
   * Sets each axis of this vector to each axis of v
   * 
   * @param {Vec} v
   * @return {Vec} this
   */
  from(v) {
    super.from(v);
    this.x = v.x != undefined ? parseFloat(v.x) : v.x;
    this.y = v.y != undefined ? parseFloat(v.y) : v.y;
    this.z = v.z != undefined ? parseFloat(v.z) : v.z;
    this.w = v.w != undefined ? parseFloat(v.w) : v.w;
    return this;
  }
  /**
   * Sets each axis of this vector
   * 
   * @param {number} x
   * @param {number} y
   * @param {number} z
   * @param {number} w
   * @return {Vec} this
   */
  set(x = undefined, y = undefined, z = undefined, w = undefined) {
    this.x = x;
    this.y = y;
    this.z = z;
    this.w = w;
    return this;
  }
  /**
   * Sets x,y from angle and length
   * 
   * @param {number} angle
   * @param {number} length
   * @return {Vec} this
   */
  fromAngle(angle, length = 1) {
    this.x = Math.cos(angle) * length;
    this.y = Math.sin(angle) * length;
    return this;
  }

  /**
   * Return the square length of this vector
   * 
   * @return {number} Square length
   */
  sqMag() {
    let m = 0;
    if (this.x) m += this.x ** 2;
    if (this.y) m += this.y ** 2;
    if (this.z) m += this.z ** 2;
    if (this.w) m += this.w ** 2;
    return m;
  }
  /**
   * Return the length of this vector
   * 
   * @return {number} Length
   */
  mag() {
    return Math.sqrt(this.sqMag());
  }

  /**
   * Performs dot poduct between this and v
   * 
   * @param {Vec} v
   * @return {number} dot
   */
  dot(v) {
    let result = 0;

    if (this.x) result += this.x * v.x;
    if (this.y) result += this.y * v.y;
    if (this.z) result += this.z * v.z;
    if (this.w) result += this.w * v.w;

    return result;
  }

  /**
   * Returns angle of this
   * 
   * @return {number} angle
   */
  angle() {
    return Math.atan2(this.y, this.x);
  }

  /**
   * Normalizes this vector
   * 
   * @return {Vec} this
   */
  normalize() {
    let mag = this.mag();
    if (mag != 0) this.div(mag);
    return this;
  }

  
  /**
   * Floors each axis of this vector
   * 
   * @return {Vec} this
   */
  floor() {
    if (this.x) this.x = Math.floor(this.x);
    if (this.y) this.y = Math.floor(this.y);
    if (this.z) this.z = Math.floor(this.z);
    if (this.w) this.w = Math.floor(this.w);
    return this;
  }

  /**
   * Ceils each axis of this vector
   * 
   * @return {Vec} this
   */
  ceil() {
    if (this.x) this.x = Math.ceil(this.x);
    if (this.y) this.y = Math.ceil(this.y);
    if (this.z) this.z = Math.ceil(this.z);
    if (this.w) this.w = Math.ceil(this.w);
    return this;
  }
  
  /**
   * Rounds each axis of this vector
   * 
   * @return {Vec} this
   */
  round() {
    if (this.x) this.x = Math.round(this.x);
    if (this.y) this.y = Math.round(this.y);
    if (this.z) this.z = Math.round(this.z);
    if (this.w) this.w = Math.round(this.w);
    return this;
  }

  /**
   * Adds each axis of this vector by val
   * 
   * @param {number} val Term
   * @return {Vec} this
   */
  add(val) {
    return this.addV(new Vec(val, val, val, val));
  }
  /**
   * Adds each axis of this vector by each axis of v
   * 
   * @param {Vec} v Terms
   * @return {Vec} this
   */
  addV(v) {
    if (this.x != undefined && v.x != undefined) this.x += v.x;
    if (this.y != undefined && v.y != undefined) this.y += v.y;
    if (this.z != undefined && v.z != undefined) this.z += v.z;
    if (this.w != undefined && v.w != undefined) this.w += v.w;

    return this;
  }

  /**
   * Subtracts each axis of this vector by val
   * 
   * @param {number} val Subtrahend
   * @return {Vec} this
   */
  sub(val) {
    return this.subV(new Vec(val, val, val, val));
  }
  /**
   * Subtracts each axis of this vector by each axis of v
   * 
   * @param {Vec} v Subtrahends
   * @return {Vec} this
   */
  subV(v) {
    if (this.x != undefined && v.x != undefined) this.x -= v.x;
    if (this.y != undefined && v.y != undefined) this.y -= v.y;
    if (this.z != undefined && v.z != undefined) this.z -= v.z;
    if (this.w != undefined && v.w != undefined) this.w -= v.w;

    return this;
  }

  /**
   * Multiplies each axis of this vector by val
   * 
   * @param {number} val Factor
   * @return {Vec} this
   */
  mul(val) {
    return this.mulV(new Vec(val, val, val, val));
  }
  /**
   * Multiplies each axis of this vector by each axis of v
   * 
   * @param {Vec} v Factors
   * @return {Vec} this
   */
  mulV(v) {
    if (this.x != undefined && v.x != undefined) this.x *= v.x;
    if (this.y != undefined && v.y != undefined) this.y *= v.y;
    if (this.z != undefined && v.z != undefined) this.z *= v.z;
    if (this.w != undefined && v.w != undefined) this.w *= v.w;

    return this;
  }

  /**
   * Divides each axis of this vector val
   * 
   * @param {number} val Divisor
   * @return {Vec} this
   */
  div(val) {
    val = 1 / val;
    return this.mulV(new Vec(val, val, val, val));
  }
  /**
   * Divides each axis of this vector by each axis of v
   * 
   * @param {Vec} v Divisors
   * @return {Vec} this
   */
  divV(v) {
    if (this.x != undefined && v.x != undefined) this.x /= v.x;
    if (this.y != undefined && v.y != undefined) this.y /= v.y;
    if (this.z != undefined && v.z != undefined) this.z /= v.z;
    if (this.w != undefined && v.w != undefined) this.w /= v.w;

    return this;
  }

  /**
   * Rotates this vector on the Y axis
   * 
   * @param {number} angle Angle in radians to rotate
   * @return {Vec} this
   */
  rotateYAxis(angle) {
    let x = this.x;
    let z = this.z;

    this.x = x * Math.cos(angle) - z * Math.sin(angle);
    this.z = x * Math.sin(angle) + z * Math.cos(angle);

    return this;
  }

  /**
   * Rotates this vector on the Z axis
   * 
   * @param {number} angle Angle in radians to rotate
   * @return {Vec} this
   */
  rotateZAxis(angle) {
    let x = this.x;
    let y = this.y;

    this.x = x * Math.cos(angle) - y * Math.sin(angle);
    this.y = x * Math.sin(angle) + y * Math.cos(angle);

    return this;
  }

  /**
   * Sets this vector to interpolation of this vector and v2
   * 
   * @param {Vec} v2
   * @param {number} i
   * @return {Vec} this
   */
  mix(v2, i) {
    if (this.x != undefined) this.x = this.x + (v2.x - this.x) * i;
    if (this.y != undefined) this.y = this.y + (v2.y - this.y) * i;
    if (this.z != undefined) this.z = this.z + (v2.z - this.z) * i;
    if (this.w != undefined) this.w = this.w + (v2.w - this.w) * i;
    return this;
  }


  _floor() {return this.copy().floor()}
  _ceil() {return this.copy().ceil()}
  _round() {return this.copy().round()}

  _normalize() {return this.copy().normalize()}

  _add(val) {return this.copy().add(val)}
  _addV(v) {return this.copy().addV(v)}
  
  _sub(val) {return this.copy().sub(val)}
  _subV(v) {return this.copy().subV(v)}
  
  _mul(val) {return this.copy().mul(val)}
  _mulV(v) {return this.copy().mulV(v)}
  
  _div(val) {return this.copy().div(val)}
  _divV(v) {return this.copy().divV(v)}

  _rotateYAxis(angle) {return this.copy().rotateYAxis(angle)}
  _rotateZAxis(angle) {return this.copy().rotateZAxis(angle)}

  _mix(v2, i) {return this.copy().mix(v2, i)}

  _() {return this.copy()}
}

let vecZero = new Vec(0, 0);
let vecHalf = new Vec(0.5, 0.5);
let vecOne = new Vec(1, 1); 





/* src/Camera.js */
class Camera extends Serializable {
  constructor(pos, props = {}) {
    super();

    this.pos = pos || new Vec(0, 0);

    this.size = new Vec(1, 1);
    this._w = undefined;
    this.w = 16;
    this._ar = undefined;
    this.ar = props.ar || nde.ar;

    this.dir = 0;
  }

  set w(value) {
    this._w = value;
    this.size.set(this.w, this.w * this.ar);
  }
  get w() {
    return this._w;
  }
  set ar(value) {
    this._ar = value;
    this.size.set(this.w, this.w * this.ar);
  }
  get ar() {
    return this._ar;
  }

  from(data) {
    super.from(data);
    if (data.pos) this.pos = new Vec().from(data.pos);
    if (data.w) this.w = data.w;
    if (data.dir) this.dir = data.dir;
    
    return this;
  }

  /**
   * Transforms vector from world space to screen space
   * 
   * @param {Vec} v World space
   * @return {Vec} Screen space
   */
  transformVec(v) {
    v = v._subV(this.pos);
    v.addV(new Vec(this.w / 2, this.w / 2 * this.ar));
    v.mul(nde.w / this.w);

    return v;
  }
  /**
   * Transforms vector from screen space to world space
   * 
   * @param {Vec} v Screen space
   * @return {Vec} World space
   */
  untransformVec(v) {
    v = v._div(nde.w / this.w);
    v.subV(new Vec(this.w / 2, this.w / 2 * this.ar));
    v.addV(this.pos);

    return v;
  }

  /**
   * Scales number from world space to screen space
   * 
   * @param {number} s World space
   * @return {number} Screen space
   */
  scale(s) {
    return s * (nde.w / this.w);
  }
  /**
   * Scales number from screen space to world space
   * 
   * @param {number} s Screen space
   * @return {number} World space
   */
  unscale(s) {
    return s / (nde.w / this.w);
  }

  /**
   * Scales vector from world space to screen space
   * 
   * @param {Vec} v World space
   * @return {Vec} Screen space
   */
  scaleVec(v) {
    return v._mul(nde.w / this.w);
  }
  /**
   * Scales vector from screen space to world space
   * 
   * @param {Vec} v Screen space
   * @return {Vec} World space
   */
  unscaleVec(v) {
    return v._div(nde.w / this.w);
  }

  /**
   * Scales renderer from world space to screen space
   * 
   * @param {Renderer} r renderer
   */
  scaleRenderer(r = renderer) {
    r.scale(new Vec(r.size.x / this.w, r.size.y / (this.w * this.ar)));
  }
  /**
   * Scales renderer from screen space to world space
   * 
   * @param {Renderer} r renderer
   */
  unscaleRenderer(r = renderer) {
    r.scale(new Vec(this.w / r.size.x, (this.w * this.ar) / r.size.y));
  }

  /**
   * Transforms renderer from world space to screen space
   * 
   * @param {Renderer} r renderer
   */
  transformRenderer(r = renderer) {
    this.scaleRenderer(r);
    r.translate(new Vec(this.w / 2, this.w / 2 * this.ar));

    r.rotate(-this.dir);
    r.translate(this.pos._mul(-1));
  }

  /**
   * Transforms renderer from screen space to world space
   * 
   * @param {Renderer} r renderer
   */
  untransformRenderer(r = renderer) {
    r.translate(this.pos._mul(1));
    r.rotate(this.dir);

    r.translate(new Vec(this.w / -2, this.w / -2 * this.ar));
    this.unscaleRenderer(r);
  }

  
  /**
   * Context where renderer is transformed
   * @param {Renderer} r renderer
   */

  _(r, context) {
    r._(()=>{
      this.transformRenderer(r);
      r.set("lineWidth", this.unscale(1));

      context();
    });
  }
}





/* src/Scene.js */
class Scene {
  constructor() {
    this.hasStarted = false;    

    this.lastIndex = 0;
    this.last = [];
  }


  
  /**
   * 
   */
  useLast(val, _default) {
    let v = this.last[this.lastIndex];
    if (v == undefined) v = (_default == undefined ? 0 : _default);

    this.last[this.lastIndex] = val;

    this.lastIndex++;

    return v;
  }


    
  /**
   * 
   */
  beforeSetup() {}
    
/**
 * 
 */
  afterSetup() {}

  
/**
 * Scene started
 */
  start() {}
  
/**
 * Scene stopped
 */
  stop() {}
 
/**
 * @param {UIEvent} e
 */
  resize(e) {}
 
/**
 * @param {KeyboardEvent} e
 */
  keydown(e) {}
  
/**
 * @param {KeyboardEvent} e
 */
  keyup(e) {}
 
/**
 * @param {MouseEvent} e
 */
  mousemove(e) {}
  
/**
 * @param {MouseEvent} e
 */
  mousedown(e) {}
  
/**
 * @param {MouseEvent} e
 */
  mouseup(e) {}
  

  
/**
 * @param {string} key
 * @param {Event} e
 */
  inputdown(key, e) {}
  
  
/**
 * @param {string} key
 * @param {Event} e
 */
  inputup(key, e) {}
  


/**
 * Mouse scrolled
 * 
 * @param {WheelEvent} e
 */
  wheel(e) {}
 
/**
 * Update scene here, called every frame
 * 
 * @param {number} dt Time in seconds since last frame
 */
  update(dt) {}

 
  /**
   * 
   * 
   * @param {number} dt Time in seconds since last frame
   */
  afterUpdate(dt) {}
    
/**
 * Render scene here, called after update
 */
  render() {}

    
/**
 * 
 */
  afterRender() {}


/**
 * 
 */
  audioContextStarted() {}
}







/* src/EventHandler.js */
class EventHandler {
  constructor() {
    this.events = {};

    this.listeners = [];
  }

  on(eventName, func, isPriority = false) {
    if (!this.events[eventName]) this.events[eventName] = [];
    if (isPriority) {
      this.events[eventName].unshift(func);
    } else {
      this.events[eventName].push(func);
    }
  }
  off(eventName, func) {
    let events = this.events[eventName];
    if (!events) return false;

    let index = events.indexOf(func);
    if (index == -1) return false;

    events.splice(index, 1);
    return true;
  }
  fire(eventName, ...args) {
    let events = this.events[eventName];
    if (events) {
      for (let i = 0; i < events.length; i++) {
        if (events[i](...args) == false) return false;
      }
    } else {
      events = this.events["*"];
      if (events) {
        for (let i = 0; i < events.length; i++) {
          if (events[i](eventName, ...args) == false) return false;
        }
      }
    }
    
    for (let i = 0; i < this.listeners.length; i++) {
      this.listeners[i].fire(eventName, ...args);
    }
      
    return true;
  }
}





/* src/inputManagers/InputManager.js */
class InputManager {
  constructor() {
    this.log = [];
  }

  init() {
    
  }

  fire(eventName, ...args) {
    if (nde.debug) {
      let log = [eventName];
      for (let i = 0; i < args.length; i++) {
        let arg = args[i];
        
        if (arg instanceof Event) {      
          if (arg instanceof MouseEvent) {          
            log.push({
              altKey: arg.altKey,
              ctrlKey: arg.ctrlKey,
              metaKey: arg.metaKey,
              shiftKey: arg.shiftKey,

              button: arg.button,
              buttons: arg.buttons,
              clientX: arg.clientX,
              clientY: arg.clientY,

              deltaY: arg.deltaY,
            });
          } else if (arg instanceof KeyboardEvent) {
            log.push({
              altKey: arg.altKey,
              ctrlKey: arg.ctrlKey,
              metaKey: arg.metaKey,
              shiftKey: arg.shiftKey,

              code: arg.code,
              key: arg.key,
            });
          } else if (arg instanceof UIEvent) {
            
          }
        } else {
          log.push(arg);
        }
      }
      this.log.push(log);
    }
    nde.fire(eventName, ...args);    
  }
}





/* src/inputManagers/InputManagerReplay.js */
class InputManagerReplay extends InputManager {
  constructor(log, playbackDivisions = 1) {
    super();

    this.log = log;
    this.playbackDivisions = playbackDivisions;

    this.frame = 0;
    this.i = 0;
  }

  init() {
    requestAnimationFrame(() => {this.animationFrame()});
  }

  fire(eventName, ...args) {
    
  }

  animationFrame() {
    if (this.frame != -1) requestAnimationFrame(() => {this.animationFrame()});

    this.i++;
    if (this.i % this.playbackDivisions != 0) return;   
    
    while (true) {
      let frame = this.log[this.frame];
      if (!frame) {
        this.frame = -1;
        return;
      }

      nde.fire(...frame);
      this.frame++;

      if (frame[0] == "input_frame") return;
    }
  }
}





/* src/assets/Asset.js */
class Asset {
  constructor() {
    this.loading = false;
    this.path = "";
  }

  load({path, name}) {
    return new Promise((resolve) => {
      fetch(path).then(async res => {
        this.data = await res.text();
        resolve(this);
      }).catch(e => {
        console.error(`"${path}" not found`);

        resolve(this);
      });

      
      if (!nde.assets) nde.assets = {};
      nde.assets[name] = this;
    });
  }

  destroy() {}
}





/* src/assets/EvalAsset.js */
let unloadedEvalAssets = [];

class EvalAsset extends Asset {
  constructor() {
    super();

    this.name = undefined;
  }

  load({path, name}) {
    this.name = name; 

    return new Promise((resolve) => {
      fetch(path).then(async res => {
        let data = await res.text();
          this.data = data;
        if (nde.hasStarted) this.eval();
        else unloadedEvalAssets.push(this);

        resolve(this);
      }).catch(e => {
        console.error(`"${path}" not found`);

        resolve(this);
      });

      
      if (!nde.assets) nde.assets = {};
      nde.assets[name] = undefined;
    });
  }

  eval() {
    let ob = eval(this.data);
    nde.assets[this.name] = ob;
    return ob;
  }
}





/* src/assets/Renderable.js */
class Renderable extends Asset {
  constructor() {
    super();
  }

  getImg() {}
}





/* src/assets/Img.js */
class Img extends Renderable {
  constructor(size = vecOne) {
    super();

    this.size = size.copy();

    this.canvas = document.createElement("canvas");
    this.canvas.width = size.x;
    this.canvas.height = size.y;

    this.ctx = this.canvas.getContext("2d");
    if (this.ctx == null) throw new Error("2d context not supported?");
  }


  load({path, name}) {
    return new Promise((resolve) => {

      let split = name.split("/");
      let lastName = split.splice(-1, 1)[0];
      let segmentNames = lastName.split(",").map(e => split.join("/") + "/" + e);
      let segments = (segmentNames.length > 1) ? [] : undefined;
    
      let image = new Image();
      image.src = path;

      image.onload = e => {
        this.resize(new Vec(image.width, image.height));
        this.ctx.drawImage(image, 0, 0);

        if (segmentNames.length > 1) {
          this.split(segmentNames.length, 1, segments);   
          
          for (let i of segments) {
            i.loading = false;
          }
        }

        resolve(this);
        
      };
      image.onerror = e => {
        console.error(`"${path}" not found`);

        resolve(this);
      };

      if (!nde.tex) nde.tex = {};
      nde.tex[name] = this;

      if (segments) {
        for (let i of segmentNames) {
          let img = new Img();
          img.path = path;
          img.loading = true;

          nde.tex[i] = img;

          segments.push(img);
        }
      }
    });
  }

  split(xSplits, ySplits = 1, arr = []) {
    let i = 0;
    let size = this.size._divV(new Vec(xSplits, ySplits)).floor();

    for (let x = 0; x < xSplits; x++) {
      for (let y = 0; y < ySplits; y++) {
        if (!arr[i]) arr[i] = new Img(size);
        let img = arr[i];
        if (!img.size.isEqualTo(size)) img.resize(size);

        img.ctx.drawImage(this.canvas, x * size.x, y * size.y, size.x, size.y, 0, 0, size.x, size.y);
        
        i++;
      }
    }

    return arr;
  }


  getImg() {return this};

  resize(size) {
    this.size = size.copy();

    this.canvas.width = size.x;
    this.canvas.height = size.y;
  }



  parseColor(...args) {
    if (args.length == 1) {
      if (typeof args[0] == "string") return args[0];
      if (typeof args[0] == "number") return `rgba(${args[0]},${args[0]},${args[0]},1)`;
      if (args[0] instanceof Vec) return `rgba(${args[0].x},${args[0].y},${args[0].z},${args[0][3] == undefined ? 1 : args[0].w})`;
      if (typeof args[0] == "object") return `rgba(${args[0][0]},${args[0][1]},${args[0][2]},${args[0][3] == undefined ? 1 : args[0][3]})`;
    }
    return `rgba(${args[0]},${args[1]},${args[2]},${args[3] == undefined ? 1 : args[3]})`;
  }

  set(property, value) {
    switch (property) {
      case "fill":
        this.ctx.fillStyle = this.parseColor(value);
        break;
      case "stroke":
        this.ctx.strokeStyle = this.parseColor(value);
        break;
      case "lineWidth":
        this.ctx.lineWidth = value;
        break;
      case "textAlign":
        this.ctx.textAlign = value[0];
        this.ctx.textBaseline = value[1];
        break;
      case "font":
        this.ctx.font = value;
        break;
      case "imageSmoothing":
        this.ctx.imageSmoothingEnabled = value;
        break;
      case "filter":
        this.ctx.filter = value;
        break;
    }
  }

  setAll(styles) {
    for (let s in styles) {      
      this.set(s, styles[s]);
    }
  }

  
  translate(pos) {
    this.ctx.translate(pos.x, pos.y);
  }
  rotate(radians) {
    this.ctx.rotate(radians);
  }
  scale(scale) {
    if (scale.x == undefined) scale = new Vec(scale, scale);
    this.ctx.scale(scale.x, scale.y);
  }
  getTransform() {
    return this.ctx.getTransform();
  }
  setTransform(transform) {
    this.ctx.setTransform(transform);
  }
  resetTransform() {
    this.ctx.resetTransform();
  }


  rect(pos, size) {    
    this.ctx.beginPath();    
    this.ctx.rect(pos.x, pos.y, size.x, size.y);
    this.ctx.fill();
    this.ctx.stroke();
  }
  clearRect(pos, size) {
    this.ctx.clearRect(pos.x, pos.y, size.x, size.y);
  }
  ellipse(pos, size) {    
    this.ctx.beginPath();    
    this.ctx.ellipse(pos.x, pos.y, size.x, size.y, 0, 0, Math.PI * 2);
    this.ctx.fill();
    this.ctx.stroke();
  }
  circle(pos, r) {    
    this.ctx.beginPath();    
    this.ctx.ellipse(pos.x, pos.y, r, r, 0, 0, Math.PI * 2);
    this.ctx.fill();
    this.ctx.stroke();
  }
  line(pos1, pos2) {    
    this.ctx.beginPath();
    this.ctx.moveTo(pos1.x, pos1.y);
    this.ctx.lineTo(pos2.x, pos2.y);
    this.ctx.stroke();
  }
  image(img, pos, size) {
    if (!img) {
      console.error("No image supplied to renderer.image()");
    }

    this.ctx.drawImage(img.getImg().canvas, pos.x, pos.y, size.x, size.y);
  }

  text(t, pos) {
    t = t + "";

    let lines = t.split("\n");
    let y = 0;
    let step = 0;

    if (lines.length > 1) step = this.measureText(lines[0]).y;
    
    for (let l of lines) {
      this.ctx.fillText(l, pos.x, pos.y + y);   
      y += step;
    } 
  }
  measureText(text) {
    text = text + "";
    let size = new Vec(0, 0);
    

    let lines = text.split("\n");
    for (let line of lines) {
      let s = this.ctx.measureText(line);
      size.x = Math.max(size.x, s.width);
      size.y += s.fontBoundingBoxAscent + s.fontBoundingBoxDescent;
    }
    
    return size;
  }


  clipRect(pos, size, context) {
    this.ctx.save();
    
    this.ctx.beginPath();
    this.ctx.rect(pos.x, pos.y, size.x, size.y);
    this.ctx.clip();

    context();
    this.ctx.restore();
  }
  
  

  save() {
    this.ctx.save();
  }
  restore() {
    this.ctx.restore();
  }
  _(context) {
    this.save();
    context();
    this.restore();
  }
}





/* src/assets/Aud.js */
class Aud extends Asset {
  constructor(props = {}) {
    super();

    this.duration = undefined;
    this.baseGain = props.baseGain || 1;

    this.queueNodes();

    this.audioBuffer = undefined;
    this.currentSource = undefined;

    this.isPlaying = false;
  }

  load({path, name, gain = 1}) {
    return new Promise((resolve) => {
      this.baseGain = gain;
      
      fetch(path).then(res => {
        res.arrayBuffer().then(arrayBuffer => {
          audioContext.decodeAudioData(arrayBuffer).then(audioBuffer => {                    
            this.audioBuffer = audioBuffer;
            this.duration = audioBuffer.duration;

            resolve(this);
          });
        });
      }).catch(e => {
        console.error(`"${path}" not found`);

        resolve(this);
      });

      
      if (!nde.aud) nde.aud = {};
      nde.aud[name] = new AudPool(this);
    });
  }

  queueNodes() {
    if (audioContext.state != "suspended") {
      this.createNodes();
      return;
    } else {
      
      nde.on("audioContextStarted", () => {
        this.createNodes();
      });
    }
  }

  createNodes() {
    this.panner = audioContext.createPanner();
    this.panner.panningModel = 'HRTF';
    this.panner.distanceModel = 'inverse';
    this.panner.rolloffFactor = 1;
    this.panner.positionX.value = 0;
    this.panner.positionY.value = 1;
    this.panner.positionZ.value = 0;

    this.gainNode = audioContext.createGain();
    this.gainNode.gain.value = this.baseGain;
  }

  copy() {
    const newAud = new Aud();
    newAud.path = this.path;
    newAud.audioBuffer = this.audioBuffer;
    newAud.baseGain = this.baseGain;
    newAud.gain = this.gain;
    return newAud;
  }

  setPosition(x, y, z) {
    if (!this.panner) return;

    this.panner.positionX.value = x;
    this.panner.positionY.value = y;
    this.panner.positionZ.value = z;
  }

  set gain(value) {
    if (!this.gainNode) return;
    this.gainNode.gain.value = this.baseGain * value;    
  }
  get gain() {
    if (!this.gainNode) return 1;
    return this.gainNode.gain.value / this.baseGain;    
  }

  play() {
    if (!this.audioBuffer || !this.panner) return;
    
    const source = audioContext.createBufferSource();
    source.buffer = this.audioBuffer;

    source.connect(this.panner);
    this.panner.connect(this.gainNode);
    this.gainNode.connect(audioContext.destination);

    source.start(0);
    this.currentSource = source;
    this.isPlaying = true;
    source.onended = () => {this.isPlaying = false; this.currentSource = null;}
  }
  stop() {    
    this.isPlaying = false;
    if (this.currentSource) {
      try {
        this.currentSource.stop();
      } catch (e) {
          // Already stopped
      }
      this.currentSource.disconnect();
      this.currentSource = null;
    }
  }
}





/* src/assets/AudPool.js */
class AudPool {
  constructor(aud) {
    this.aud = aud;

    this.auds = [];
  }

  get() {
    for (let i = 0; i < this.auds.length; i++) {
      if (this.auds[i].isPlaying) continue;

      return this.auds[i];
    }
    
    let aud = this.getNew();
    this.auds.push(aud);
    return aud;
  }
  getNew() {
    return this.aud.copy();
  }
}


function moveListener(pos) {
  if (audioContext.listener.positionX != undefined) {
    audioContext.listener.positionX.value = pos.x;
    audioContext.listener.positionY.value = 0;
    audioContext.listener.positionZ.value = pos.y;
  } else {
    audioContext.listener.setPosition(pos.x, 0, pos.y);
  }
}
function playAudio(audPool, pos) {
  let aud = audPool.get();
  aud.setPosition(pos.x, 1, pos.y);
  aud.gain = 1;
  aud.play();  
  return aud;
}





/* src/assets/animation/frames/AnimationFrameBase.js */
class AnimationFrameBase {
  constructor() {
    this.duration = 0;
  }

  step(runningAnimation) {}
}





/* src/assets/animation/frames/AnimationFrame.js */
class AnimationFrame extends AnimationFrameBase {
  constructor(tex) {
    super();
    this.duration = 1;

    this.img = nde.tex[tex];
  }

  step(runningAnimation) {
    runningAnimation.img = this.img;
  }
}





/* src/assets/animation/frames/AnimationFrameEvent.js */
class AnimationFrameEvent extends AnimationFrameBase {
  constructor(eventName, ...eventArgs) {
    super();

    this.eventName = eventName;
    this.eventArgs = eventArgs;
  }

  step(runningAnimation) {
    runningAnimation.fire(this.eventName, ...this.eventArgs);
  }
}





/* src/assets/animation/frames/AnimationFrameLoop.js */
class AnimationFrameLoop extends AnimationFrameBase {
  constructor() {
    super();
  }

  step(runningAnimation) {
    runningAnimation.restart();
  }
}





/* src/assets/animation/Animation.js */
class Animation extends EvalAsset {
  constructor(frames = [], dt = 0.1) {
    super();

    this.frames = frames;
    this.dt = dt;
    this.speed = 1;

    this.duration = 0;
    for (let f of this.frames) this.duration += f.duration;
    this.duration *= dt;
  }

  start(props = {}) {
    return new RunningAnimation(this, props);
  }

  eval() {
    let ob = eval("let frame = AnimationFrame, loop = AnimationFrameLoop, event = AnimationFrameEvent;" + this.data);
    if (!nde.tex) nde.tex = {};
    nde.tex[this.name] = ob;
    return ob;
  }
}





/* src/assets/animation/RunningAnimation.js */
class RunningAnimation extends Renderable {
  constructor(animation, props = {}) {
    super();

    this.frames = animation.frames;
    this.dt = animation.dt;
    this.duration = animation.duration;
    this.speed = animation.speed;

    this.e = new EventHandler();
    if (props.events) this.e.events = props.events;
    if (props.listeners) this.e.listeners = props.listeners;
    
    this.img = undefined;

    this.timer = new TimerTime(100000000, ()=>{this.step()});
    this.lastTimerElapsedTime = 0;
    this.elapsedTime = 0;
    this.executedFrames = 0;
    this.executedTime = 0;
    
    this.step();
  }

  step() {    
    
    this.elapsedTime += (this.timer.elapsedTime - this.lastTimerElapsedTime) * this.speed;
    this.lastTimerElapsedTime = this.timer.elapsedTime;

    while (this.executedTime <= this.elapsedTime) {
      let frame = this.frames[this.executedFrames];
      if (!frame) {
        this.fire("done");
        return;
      }

      this.executedTime += frame.duration * this.dt;
      this.executedFrames++;

      frame.step(this);            
    }
  }

  on(...args) {return this.e.on(...args)}
  off(...args) {return this.e.off(...args)}
  fire(...args) {return this.e.fire(...args)}


  getImg() {    
    return this.img;
  }

  start() {
    this.timer.start();
  }
  stop() {
    this.timer.stop();
  }

  restart() {
    this.timer.reset();
    this.lastTimerElapsedTime = 0;
    this.elapsedTime = 0;
    this.executedFrames = 0;
    this.executedTime = 0;
    this.step();
  }
}





/* src/stateMachines/nodes/StateMachineNodeBase.js */
class StateMachineNodeBase {
  constructor(...children) {
    this.children = children;
  }

  choose(stateMachine) {}
}





/* src/stateMachines/nodes/StateMachineNodeCondition.js */
class StateMachineNodeCondition extends StateMachineNodeBase {
  constructor(conditionF, ...children) {
    super(...children);
    
    this.conditionF = conditionF;
    this.children = children;
  }

  choose(stateMachine) {
    let result = this.conditionF(stateMachine);
    if (result == true) return this.children[0];
    if (result == false) return this.children[1];
    return this.children[2];
  }
}





/* src/stateMachines/nodes/StateMachineNodeResult.js */
class StateMachineNodeResult extends StateMachineNodeBase {
  constructor(result) {
    super();

    this.result = result;
  }
}





/* src/stateMachines/StateMachine.js */
class StateMachine {
  constructor(rootNode) {
    this.rootNode = rootNode;

    this.lastChoice = undefined;
    this.result = undefined;

    this.e = new EventHandler();
  }

  choose() {
    let choice = this.rootNode;

    while (choice && !(choice instanceof StateMachineNodeResult)) {
      choice = choice.choose(this);
    }


    if (choice != this.lastChoice) {    
      this.parseResult(choice.result);
      this.fire("change", this.result);
      this.lastChoice = choice;
    }

    return this.result;
  }

  parseResult(result) {
    this.result = result;
  }


  

  on(...args) {return this.e.on(...args)}
  off(...args) {return this.e.off(...args)}
  fire(...args) {return this.e.fire(...args)}
}





/* src/stateMachines/StateMachineImg.js */
class StateMachineImg extends StateMachine {
  constructor(rootNode) {
    super(rootNode);

    this._speed = 1;
  }

  set speed(value) {
    if (this.result && this.result instanceof RunningAnimation) {
      this.result.speed = value;
    }
    this._speed = value;
  }
  get speed() {
    return this._speed;
  }

  parseResult(result) {
    if (this.result instanceof RunningAnimation) this.result.stop();

    if (result instanceof Animation) {  
      this.result = result.start({listeners: [this.e]});
      this.result.speed *= this.speed;
    } else {
      this.result = result;
    }
    
    return this.result;
  }
}





/* src/ui/UIBase.js */
let defaultStyle = {
  minSize: new Vec(0, 0),
  maxSize: new Vec(Infinity, Infinity),

  growX: false,
  growY: false,

  scroll: {
    x: true,
    y: true,
    alwaysShow: false,
    fill: "rgba(255, 255, 255, 255)",
    width: -1, //-1 to be padding size
  },

  align: new Vec(0, 0), //0: left, 1: center, 2: right,    0: top, 1: middle, 2: bottom

  position: "normal", //normal (from calculated pos, takes up space), relative (from parent element), absolute (from 0, 0)
  pos: new Vec(0, 0), //position offset
  selfPos: new Vec(0, 0), //position offset relative to size

  render: "normal", //normal, hidden, last

  padding: 0,

  direction: "row",
  gap: 0,

  fill: "rgba(0, 0, 0, 0)",
  stroke: "rgba(0, 0, 0, 0)",

  cursor: "auto",
}

class UIBase {
  constructor(props = {}) {
    this.defaultStyle = {};
    this.style = undefined;

    this.parent = undefined;
    this.uiRoot = undefined;

    this.children = props.children || [];
    this.e = new EventHandler();
    if (props.events) this.e.events = props.events;

    this.interactable = false;
    
    this.hovered = false;
    this.trueHovered = false;
    this.trueHoveredBottom = false;
    this.pos = new Vec(0, 0);
    this.size = new Vec(1, 1);
    this.contentSize = new Vec(1, 1);
    this.scroll = new Vec(0, 0);

    this.debugColor = undefined;

    this.fillStyle(props.style);

  }

  on(...args) {return this.e.on(...args)}
  off(...args) {return this.e.off(...args)}
  fire(...args) {return this.e.fire(...args)}


  fillStyle(style) {
    this.style = {};
    
    if (!style) style = {};

    let temp = nestedObjectAssign({}, defaultStyle, this.defaultStyle);

    nestedObjectAssign(this.style, temp, style);

    if (this.style.size) {
      this.style.minSize = this.style.size._();
      this.style.maxSize = this.style.size._();
      delete this.style.size;
    }

    delete this.style.hover;
    this.style.hover = nestedObjectAssign({}, this.style, style.hover);
  }

  calculateSize() {
    for (let c of this.children) c.calculateSize();

    let isRow = this.style.direction == "row";

    this.size.set(0, 0);

    let numChildren = 0;
    for (let c of this.children) {
      if (c.style.position != "normal") continue;

      if (isRow) {
        this.size.x += c.size.x;
        this.size.y = Math.max(this.size.y, c.size.y);
      } else {
        this.size.x = Math.max(this.size.x, c.size.x);
        this.size.y += c.size.y;
      }
      numChildren++;
    }
    
    this.size.add(this.style.padding * 2);

    let gap = this.style.gap * Math.max(numChildren - 1, 0);

    if (isRow) this.size.x += gap;
    else this.size.y += gap;

    this.contentSize.from(this.size);

    this.size.x = Math.min(Math.max(this.size.x, this.style.minSize.x), this.style.maxSize.x);
    this.size.y = Math.min(Math.max(this.size.y, this.style.minSize.y), this.style.maxSize.y);
  }

  growChildren() {
    let isRow = this.style.direction == "row";

    let numChildren = 0;
    for (let c of this.children) {
      if (c.style.position != "normal") continue;
      numChildren++;
    }

    for (let i = 0; i < 2; i++) {
      let isHor = i == 0;

      let remaining = isHor ? this.size.x : this.size.y;
      let growable = [];

      remaining -= this.style.padding * 2;

      if (isHor == isRow) {
        for (let c of this.children) {
          if (c.style.position != "normal") continue;

          if (isHor) {
            remaining -= c.size.x;
            if (c.style.growX) growable.push(c);
          } else {
            remaining -= c.size.y;
            if (c.style.growY) growable.push(c);
          }
        }
        if (growable.length == 0) continue;
  
        remaining -= this.style.gap * Math.max(numChildren - 1, 0);

        while (remaining > 0.0001) {
          let smallestChild = growable[0];

          let smallest = isHor ? smallestChild.size.x : smallestChild.size.y;
          let secondSmallest = Infinity;
          let widthToAdd = remaining;

          for (let c of growable) {
            let w = isHor ? c.size.x : c.size.y;

            if (w < smallest) {
              secondSmallest = smallest;
              smallest = w;
            }
            if (w > smallest) {
              secondSmallest = Math.min(secondSmallest, w);
              widthToAdd = secondSmallest - smallest;
            }
          }

          widthToAdd = Math.min(widthToAdd, remaining / growable.length);

          for (let c of growable) {
            let w = isHor ? c.size.x : c.size.y;

            if (w == smallest) {
              if (isHor) {
                c.size.x += widthToAdd;
              } else {
                c.size.y += widthToAdd;
              }
              remaining -= widthToAdd;
            }
          }         

        }
        for (let c of this.children) {
          if ((isHor && !c.style.growX) || (!isHor && !c.style.growY) || c.style.position != "normal") continue;
  
          if (isHor) {
            c.size.x += remaining / growable.length;
          } else {
            c.size.y += remaining / growable.length;
          } 
        }


      } else {
        for (let c of this.children) {
          if ((isHor && !c.style.growX) || (!isHor && !c.style.growY) || c.style.position != "normal") continue;
  
          if (isHor) {
            c.size.x = remaining;
          } else {
            c.size.y = remaining;
          }             
        }
      }
    }



    for (let c of this.children) {
      c.growChildren();
    }
  }

  positionChildren() {
    let isRow = this.style.direction == "row";


    let remaining = isRow ? this.size.x : this.size.y;
    remaining -= this.style.padding * 2;

    let numChildren = 0;
    for (let c of this.children) {
      if (c.style.position != "normal") continue;

      if (isRow) remaining -= c.size.x;
      else remaining -= c.size.y;

      numChildren++;
    }

    remaining -= this.style.gap * Math.max(numChildren - 1, 0);
    remaining *= (isRow ? this.style.align.x : this.style.align.y) / 2;
    
    let along = this.style.padding + remaining;

    for (let c of this.children) {
      switch (c.style.position) {
        case "normal":
          let remaining;

          if (isRow) {
            remaining = this.size.y - this.style.padding * 2 - c.size.y;
            remaining *= this.style.align.y / 2;

            c.pos.x = this.pos.x + along;
            c.pos.y = this.pos.y + this.style.padding + remaining;

            along += c.size.x + this.style.gap;
          } else {
            remaining = this.size.x - this.style.padding * 2 - c.size.x;
            remaining *= this.style.align.x / 2;

            c.pos.x = this.pos.x + this.style.padding + remaining;
            c.pos.y = this.pos.y + along;

            along += c.size.y + this.style.gap;
          }
          break;

        case "absolute":
          c.pos = this.uiRoot.pos._add(this.uiRoot.style.padding);
          break;

        case "relative":
          c.pos = this.pos._add(this.style.padding);
          break;
      }
      
      c.pos.subV(this.scroll);

      c.pos.addV(c.style.pos);

      c.pos.addV(c.style.selfPos._mulV(c.size));

      c.positionChildren();
    }
  }

  renderDebug() {
    if (this.debugColor) {
      nde.renderer.set("fill", `rgb(${this.debugColor}, 0, 0)`);
      nde.renderer.set("stroke", `rgb(255, 255, 255)`);

      if (this.trueHovered) {
        nde.renderer.set("fill", `rgb(${this.debugColor}, ${this.debugColor}, 0)`);
      }

      if (this.trueHoveredBottom) {
        nde.renderer.set("fill", `rgb(0, 255, 0)`);

        nde.debugStats.uiClass = this.__proto__.constructor.name;
        nde.debugStats.uiPos = this.pos.toString();
        nde.debugStats.uiSize = this.size.toString();
        nde.debugStats.uiContentSize = this.contentSize.toString();
        for (let style in this.style) {
          nde.debugStats[style] = this.style[style];
          if (this.style[style] instanceof Vec) nde.debugStats[style] = this.style[style].toString();
        }
      }
    }
  }

  render() {
    nde.renderer.setAll(this.hovered ? this.style.hover : this.style);

    this.renderDebug();

    nde.renderer.rect(this.pos, this.size);   
  }

  constrainScroll() {
    let maxScroll = this.contentSize._subV(this.size);
    this.scroll.x = Math.max(Math.min(this.scroll.x, maxScroll.x), 0);
    this.scroll.y = Math.max(Math.min(this.scroll.y, maxScroll.y), 0);
  }
  renderScrollbars() {
    let size = this.size._sub(this.style.padding * 2);
    let fraction = size._divV(this.contentSize);
    let scrollFraction = this.scroll._divV(this.contentSize._subV(this.size));

    let width = this.style.scroll.width;
    if (width == -1) width = this.style.padding;

    nde.renderer.set("stroke", "rgba(0, 0, 0, 0)");
    nde.renderer.set("fill", this.style.scroll.fill);
    
    
    if (this.style.scroll.x && fraction.x < 1) {
      let max = 1 - fraction.x;
      nde.renderer.rect(new Vec(this.pos.x + scrollFraction.x * max * this.size.x, this.pos.y + this.size.y - width), new Vec(this.size.x * fraction.x, width));      
    }
    
    if (this.style.scroll.y && fraction.y < 1) {
      let max = 1 - fraction.y;
      nde.renderer.rect(new Vec(this.pos.x + this.size.x - width, this.pos.y + scrollFraction.y * max * this.size.y), new Vec(width, this.size.y * fraction.y));
      
    }
    
  }
}

function nestedObjectAssign(dest, target, source) {  
  Object.assign(dest, target, source);
  if (target == undefined || source == undefined) return dest;

  for (let key in dest) {
    let ob = dest[key];
    
    if (ob instanceof Vec) {
      dest[key] = new Vec().from(ob);
    }
    else if (Array.isArray(ob)) {
      
    }
    else if (ob instanceof Object) {
      dest[key] = nestedObjectAssign({}, target[key], source[key]);
    }
  }

  return dest;
}






/* src/ui/UIRoot.js */
class UIRoot extends UIBase {
  constructor(props) {
    super(props);

    this.defaultStyle = {
      direction: "column",
    };

    this.fillStyle(props.style);

    if (props.pos) this.pos.from(props.pos);

    this.initUI();

    this.deepestScrollable = undefined;
    this.deepestScrollableDepth = 0;
    this.renderLast = [];
  }

  initUI() {
    this.depth = 0;

    this.initPass(this);

    this.calculateSize();
    this.constrainAllScroll();
    this.growChildren();
    this.positionChildren();

    this.fire("init");
  }

  renderUI() {
    nde.renderer._(()=>{
      this.hoverPass();
      this.renderPass();
    });
  }


  initPass(elem, depth = 0) {
    elem.uiRoot = this;
    this.depth = Math.max(this.depth, depth);

    for (let c of elem.children) {
      c.parent = elem;

      this.initPass(c, depth + 1);
    }
  }


  constrainAllScroll(element = this) {
    element.constrainScroll();

    for (let c of element.children) {
      this.constrainAllScroll(c);
    }
  }



  wheel(e) {
    if (!this.deepestScrollable) return;
    
    let delta = e.deltaY * 0.5;

    if (e.shiftKey) this.deepestScrollable.scroll.x += delta;
    else this.deepestScrollable.scroll.y += delta;

    this.deepestScrollable.constrainScroll();

    this.deepestScrollable.positionChildren();

    return false;
  }



  hoverPass() {    
    let mousePoint = new DOMPoint(nde.mouse.x, nde.mouse.y);
    let transformedMousePoint = mousePoint.matrixTransform(nde.renderer.getTransform().inverse());

    document.body.style.cursor = "auto";

    this.renderLast.length = 0;
    this.deepestScrollable = undefined;
    this.hoverPassHelper(this, false, false, transformedMousePoint);

    for (let elem of this.renderLast) {
      this.hoverPassHelper(elem[0], elem[1], elem[2], transformedMousePoint, true);
    }

    if (transformedMousePoint.x >= this.pos.x && transformedMousePoint.x <= this.pos.x + this.size.x && transformedMousePoint.y >= this.pos.y && transformedMousePoint.y <= this.pos.y + this.size.y) {
      nde.hoveredUIRoot = this;      
    }
  }
  hoverPassHelper(element, found, ignoreHover, pt, ignoreRenderLast = false) {
    if (element.style.render == "last" && !ignoreRenderLast) {
      this.renderLast.push([element, found, ignoreHover]);
      return;
    }
    if (element.style.render == "hidden") return;


    element.hovered = false;
    element.trueHovered = false;
    element.trueHoveredBottom = false;

    let clip = (element.style.maxSize.x != Infinity || element.style.maxSize.y != Infinity);
    let scrollable = clip && ((element.style.scroll.x && element.contentSize.x > element.size.x - element.style.padding * 2) || (element.style.scroll.y && element.contentSize.y > element.size.y - element.style.padding * 2));

    if (ignoreHover || (clip && 
      (pt.x < element.pos.x || 
      pt.x > element.pos.x + element.size.x || 
      pt.y < element.pos.y || 
      pt.y > element.pos.y + element.size.y))) 
    {
      found = false;
      ignoreHover = true;
    } else {
      let inBounds = (pt.x >= element.pos.x && 
                    pt.x <= element.pos.x + element.size.x && 
                    pt.y >= element.pos.y && 
                    pt.y <= element.pos.y + element.size.y);
    
      element.trueHovered = inBounds;
      element.trueHoveredBottom = inBounds;
      
      if (inBounds) {
        if (element.interactable) {
          nde.hoveredUIElement = element;
          document.body.style.cursor = element.style.cursor;

          found = true;
        }

        if (scrollable) {
          this.deepestScrollable = element;          
        }
      }
    
    }

    if (found) {
      element.hovered = true;
    }
    if (element.forceHover) {
      element.hovered = true;
      found = true;        
    }
    

    


    for (let c of element.children) {
      this.hoverPassHelper(c, found, ignoreHover, pt);

      if (c.trueHovered) element.trueHoveredBottom = false;
    }
  }

  renderPass() {
    this.renderLast.length = 0;
    this.renderPassHelper(this, 0);

    for (let elem of this.renderLast) {
      this.renderPassHelper(elem[0], elem[1], true);
    }
  }
  renderPassHelper(element, depth, ignoreRenderLast = false) {
    if (element.style.render == "last" && !ignoreRenderLast) {
      this.renderLast.push([element, depth]);
      return;
    }
    if (element.style.render == "hidden") return;

    if (nde.uiDebug) {
      element.debugColor = 255 / (this.depth + 1) * (depth + 1);
    } else element.debugColor = undefined;


    element.render();

    let clip = (element.style.maxSize.x != Infinity || element.style.maxSize.y != Infinity);
    if (clip) {
      nde.renderer.clipRect(element.pos._add(element.style.padding), element.size._sub(element.style.padding * 2), () => {
        for (let c of element.children) {
          this.renderPassHelper(c, depth + 1);
        }
        
      });
      if (this.deepestScrollable == element || element.style.scroll.alwaysShow) element.renderScrollbars();
    } else {
      for (let c of element.children) {
        this.renderPassHelper(c, depth + 1);
      }
    }
  }
}





/* src/ui/UIText.js */
class UIText extends UIBase {
  constructor(props) {
    super(props);
    this.children = [];

    this.defaultStyle.text = {
      fill: "rgb(255, 255, 255)",

      font: "25px monospace",
      textAlign: ["left", "top"],
    };

    this.fillStyle(props.style);

    this.text = props.text;    
  }

  calculateSize() {
    nde.renderer.setAll(this.style.text);

    this.size = nde.renderer.measureText(this.text);    

    if (this.size.x < this.style.minSize.x) this.size.x = this.style.minSize.x;
    if (this.size.y < this.style.minSize.y) this.size.y = this.style.minSize.y;
  }

  render() {
    nde.renderer.setAll(this.hovered ? this.style.hover.text : this.style.text);

    super.renderDebug();

    nde.renderer.text(this.text, this.pos);
  }
}





/* src/ui/UIImage.js */
class UIImage extends UIBase {
  constructor(props) {
    super(props);
    this.children = [];

    this.defaultStyle.image = {
      imageSmoothing: false,
    };

    this.fillStyle(props.style);  

    this.image = props.image;
  }

  render() {
    nde.renderer.setAll(this.hovered ? this.style.hover.image : this.style.image);

    super.renderDebug();

    nde.renderer.image(this.image, this.pos, this.size);
  }
}





/* src/ui/buttons/UIButton.js */
class UIButton extends UIBase {
  constructor(props) {
    super(props);

    this.interactable = true;
  }
}





/* src/ui/buttons/UIButtonText.js */
class UIButtonText extends UIButton {
  constructor(props) {
    super(props);

    this.children = [new UIText({
      text: props.text,
      style: props.textStyle,
    })];
  }
}





/* src/ui/buttons/UIButtonImage.js */
class UIButtonImage extends UIButton {
  constructor(props) {
    super(props);

    this.children = [new UIImage({
      image: props.image,
      style: props.imageStyle,
    })];
  }
}





/* src/ui/ScenePopup.js */
class ScenePopup extends Scene {
  constructor() {
    super();

    this.cam = new Camera(new Vec(800, 450));
    this.cam.w = 1600;

    this.img = undefined;

    this.ui = new UIRoot({
      children: [
        new UIBase({
          style: {
            pos: new Vec(800, 450),
            selfPos: new Vec(-0.5, -0.5),
          },
        }),
      ]
    });   
  }


  inputdown(key) {
    if (nde.getKeyEqual(key,"Pause")) {
      nde.resolvePopup();
    }
  }

  captureScreen() {
    this.img = new Img(new Vec(nde.w, nde.w * nde.ar));
    nde.fire("render");
    this.img.ctx.imageSmoothingEnabled = false;
    this.img.image(nde.renderer, vecZero, this.img.size);
  }

  render() {
    let cam = this.cam;

    cam._(renderer, ()=>{
      renderer.image(this.img, vecZero, new Vec(cam.w, cam.w * cam.ar));
      this.ui.renderUI();
    });
  }
}





/* src/ui/settings/UISettingBase.js */
class UISettingBase extends UIBase {
  constructor(props) {
    super(props);
    this.interactable = true;

    this.value = props.value;
    this.focused = false;

    this.name = props.name;
    this.displayName = props.displayName;
  }


  initChildren() {}


  setValue(newValue) {
    this.value = newValue;
  }
  setFocus(newFocus) {
    this.focused = newFocus;

    if (!newFocus) {
      this.fireChange(false);
    }
  }

  fireInput() {
    this.fire("input", this.value);
  }
  fireChange(wasSubmitted = true) {    
    this.fire("change", this.value, wasSubmitted);
  }
}





/* src/ui/settings/UISettingCollection.js */
class UISettingCollection extends UISettingBase {
  constructor(props) {
    super(props);
    this.interactable = false;

    this.defaultStyle = {
      direction: "column",

      row: {growX: true},
      label: {},
    };
    this.fillStyle(props.style);  

    this.value = props.value || {};

    for (let i = 0; i < this.children.length; i++) {
      let c = this.children[i];

      let rowChildren = [c];
      
      if (props.hasLabels && c.displayName) {
        rowChildren.unshift(new UIBase({
          style: {growX: true},
        }));

        rowChildren.unshift(new UIText({
          text: c.displayName,

          style: this.style.label,
        }));
      }

      this.children[i] = new UISettingCollectionRow({
        style: this.style.row,

        children: rowChildren,
      });

      let name = c.name;
      if (name && c.value != undefined) {
        if (this.value[name] != undefined) c.setValue(this.value[name]);
        else this.value[name] = c.value;
  
        c.on("input", value => {
          this.value[name] = value;
          this.fire("input", this.value);
        });
        c.on("change", value => {
          this.value[name] = value;
          this.fire("change", this.value);
        });
      }
    }
  }

  render() {
    super.render();
    
    for (let i in this.elements) {
      let elem = this.elements[i];      
      nde.renderer.setAll(this.style.text);
      nde.renderer.text(this.template[i].name || i, new Vec(this.pos.x, elem.pos.y));

      elem.render();
    }
  }
}


class UISettingCollectionRow extends UIBase {
  render() {
    if (this.children.length == 3) this.children[0].hovered = this.children[2].hovered;
    
    super.render();
  }
}






/* src/ui/settings/UISettingChoice.js */
class UISettingChoice extends UISettingBase {
  constructor(props) {
    super(props);
    this.interactable = false;

    this.defaultStyle = {
      direction: "column",
      gap: 10,
    };
    this.fillStyle(props.style); 
    this.style.fill = "rgba(0, 0, 0, 0)";
    this.style.padding = 0;
    
    this.choices = props.choices || ["undefined"];
    this.value = props.value || this.choices[0];

    

    for (let choice of this.choices) {
      let elem = new UIButton({
        style: {...props.style,
          direction: "row",
          align: new Vec(0, 1),
        },

        children: [
          new UIText({
            style: props.style,
            text: choice,
          }),
          new UIBase({
            style: {
              minSize: this.style.minSize._sub((props.style.padding || 0) * 2),
            },
          }),
        ],

        events: {mouseup: [() => {
          this.value = choice;

          this.updateColors();
          
          this.fire("input", this.value);
          this.fire("change", this.value);
        }]},
      });

      this.children.push(elem);
    }

    this.updateColors();
  }

  updateColors() {
    for (let c of this.children) {
      let name = c.children[0].text;
      let col = (name == this.value) ? "rgb(255, 255, 255)" : "rgba(0, 0, 0, 0)";

      c.children[1].style.fill = col;
      c.children[1].style.hover.fill = col;
    }
  }

  setValue(newValue) {
    super.setValue(newValue);

    this.updateColors();
  }
}





/* src/ui/settings/UISettingDropdown.js */
class UISettingDropdown extends UISettingBase {
  constructor(props) {
    super(props);
    this.interactable = false;

    this.defaultStyle = {
      gap: 0,
    };
    this.fillStyle(props.style); 
    
    this.choices = props.choices || ["undefined"];
    this.value = props.value || this.choices[0];

    let oldMaxSize = this.style.maxSize;
    this.style.maxSize = new Vec(Infinity, Infinity);

    this.children = [
      new UIButton({
        style: {...this.style,
          growX: true,
          growY: true,
        },
        children: [
          new UIText({
            style: {...this.style},
            text: this.value,
          }),
          new UIText({
            style: {...this.style,
              minSize: this.style.minSize._sub((this.style.padding || 0) * 2),
            },

            text: "\u23F7",
          }),
        ],

        events: {mouseup: [() => {
          this.switchOpen();
        }]}
      }),
      new UIBase({
        style: {
          position: "relative",
          render: "hidden",
        },

        children: [
          new UIBase({
            style: {
              scroll: {...this.style.scroll,
                width: this.style.padding / 2,
              },
              direction: "column",
              gap: this.style.gap,
              maxSize: oldMaxSize,
            },
          }),
        ],
      }),
    ];
    this.container = this.children[1].children[0];

    let size = new Vec(0, 0);
    for (let choice of this.choices) {
      let elem = new UIButton({
        style: {...this.style,
          direction: "row",
          align: new Vec(0, 1),
          growX: true,
        },

        children: [
          new UIText({
            style: this.style,
            text: choice,
          }),
          new UIBase({
            style: {
              minSize: this.style.minSize._sub((props.style.padding || 0) * 2),
            },
          }),
        ],

        events: {mouseup: [() => {
          this.setValue(choice);

          this.switchOpen();
          
          this.fire("input", this.value);
          this.fire("change", this.value);
        }]},
      });

      this.container.children.push(elem);

      for (let c of elem.children) c.calculateSize();
      elem.calculateSize();

      size.x = Math.max(size.x, elem.size.x);
      size.y = Math.max(size.y, elem.size.y);
    }
    this.style.minSize.x = Math.max(this.style.minSize.x, size.x);
    this.style.minSize.y = Math.max(this.style.minSize.y, size.y);

    
    this.style.fill = "rgba(0, 0, 0, 0)";
    this.style.padding = 0;

    this.updateColors();
  }

  switchOpen() {
    if (this.children[0].style.render == "hidden") {
      this.children[0].style.render = "normal";
      this.children[1].style.render = "hidden";
    } else {
      this.children[0].style.render = "hidden";
      this.children[1].style.render = "last";
    }
  }
  updateColors() {
    let dropdown = this.children[1].children[0];
    for (let c of dropdown.children) {
      let name = c.children[0].text;
      let col = (name == this.value) ? "rgb(255, 255, 255)" : "rgba(0, 0, 0, 0)";

      c.children[1].style.fill = col;
      c.children[1].style.hover.fill = col;
    }
  }

  setValue(newValue) {
    super.setValue(newValue);

    this.updateColors();
    this.children[0].children[0].text = this.value;
    if (this.uiRoot) this.uiRoot.initUI();
  }
}







/* src/ui/settings/UISettingCheckbox.js */
class UISettingCheckbox extends UISettingBase {
  constructor(props) {
    super(props);

    this.defaultStyle.checkbox = {
      growX: true,
      growY: true,
        
      checked: {
        fill: "rgba(255, 255, 255, 1)",
        stroke: "rgba(255, 255, 255, 1)",
      },
    };
    this.fillStyle(props.style);

    this.initChildren();
    

    this.funcA = e=>this.mouseupGlobal(e);
    this.on("mouseup", e=>{this.mouseupLocal(e)});
    this.on("mousedown", e=>{
      this.forceHover = true;

      nde.on("mouseup", this.funcA);
    });

    

    this.setValue(props.value);
  }

  
  initChildren() {
    this.children = [new UIBase({
      
    })];
  }

  setValue(newValue) {
    super.setValue(newValue);

    let style = this.style.checkbox;
    if (this.value) style = {...style, ...this.style.checkbox.checked};

    let hoverStyle = this.style.hover.checkbox;
    if (this.value) hoverStyle = {...hoverStyle, ...this.style.hover.checkbox.checked};

    style.hover = hoverStyle;
    
    this.children[0].fillStyle(style);
  }

  mouseupLocal(e) {
    if (!this.forceHover) return;

    this.setValue(!this.value);
    this.fireInput();
    this.fireChange();
  }

  mouseupGlobal(e) {
    this.forceHover = false;

    nde.off("mouseup", this.funcA);
  }
}





/* src/ui/settings/UISettingRange.js */
class UISettingRange extends UISettingBase {
  constructor(props) {
    super(props);

    this.defaultStyle = {
      gap: 10,

      slider: {
        padding: 5,
        minSize: new Vec(300, 30),

        fill: "rgb(0, 0, 0)",
  
        active: {
          fill: "rgb(255, 255, 255)",
        },
      },

      number: {
        fill: "rgb(0, 0, 0)",
      },
    };
    this.fillStyle(props.style);
    this.style.fill = "rgba(0, 0, 0, 0)";
    this.style.hover.fill = "rgba(0, 0, 0, 0)";
    

    this.min = props.min;
    this.max = props.max;
    this.step = props.step || 1;


    this.initChildren();

    this.rangeSizeTotal = new Vec(0, 0);
    this.setValue(this.value);
    

    this.rendererTransform = undefined;

    this.funcA = e=>this.mousemove(e);
    this.funcB = e=>this.mouseup(e);

    this.on("mousedown", e=>{
      this.forceHover = true;

      this.mousemove(e);

      nde.on("mousemove", this.funcA);
      nde.on("mouseup", this.funcB);
    });
  }


  initChildren() {
    let numberSize = new Vec(0, 0);
    nde.renderer._(()=>{
      nde.renderer.setAll(this.style.number.text);
      numberSize.y = Math.max(nde.renderer.measureText(this.max).y, this.style.minSize.y);
      numberSize.x = Math.max(nde.renderer.measureText(this.max).x, this.style.minSize.x);
    });

    let sliderSize = new Vec(this.style.slider.minSize.x - numberSize.x - this.style.gap, this.style.slider.minSize.y);
    
    this.slider = new UIBase({
      style: {...this.style.slider.active,
        hover: this.style.hover.slider.active,
      },
    });
    this.range = new UIBase({
      style: {...this.style.slider,
        hover: this.style.hover.slider,
        minSize: sliderSize,
      },

      children: [this.slider],
    });
    this.number = new UISettingText({
      style: {...this.style.number,
        hover: this.style.hover.number,
        text: this.style.text,
        size: numberSize,
        editor: {
          numberOnly: true,
        },
      },

      value: 0,

      events: {change: [value => {
        this.setValue(value);
        this.fireInput();
        this.fireChange();
      }]},
    });

    this.children = [this.range, this.number];
  }



  calculateSize() {
    super.calculateSize();

    this.rangeSizeTotal.from(this.range.size).sub(this.range.style.padding * 2);

    this.sizeSlider();
  }

  mousemove(e) {
    let mousePoint = new DOMPoint(nde.mouse.x, nde.mouse.y);
    let transformedMousePoint = mousePoint.matrixTransform(this.rendererTransform.inverse());
    
    let progress = Math.min(Math.max((transformedMousePoint.x - this.pos.x - this.style.padding) / this.rangeSizeTotal.x, 0), 1); 

    this.setValue(progress * (this.max - this.min) + this.min);
    this.fireInput();
  }

  mouseup(e) {
    this.forceHover = false;

    nde.off("mousemove", this.funcA);
    nde.off("mouseup", this.funcB);

    this.fireChange();
  }

  setValue(newValue) {
    super.setValue(Math.round(newValue / this.step) * this.step);

    this.sizeSlider();
    
    this.number.setValue(this.value);
    
  }

  sizeSlider() {
    this.slider.size.x = (this.value - this.min) / (this.max - this.min) * this.rangeSizeTotal.x;
    this.slider.size.y = this.rangeSizeTotal.y;
  }

  render() {    
    super.render();
    
    this.rendererTransform = nde.renderer.getTransform();
  }
}





/* src/ui/settings/UISettingText.js */
let activeSettingText = undefined;

class UISettingText extends UISettingBase {
  constructor(props) {
    super(props);

    this.defaultStyle = {
      cursor: "text",

      editor: {
        blinkTime: 1,
        clickTime: 0.5,
        multiLine: false,
        numberOnly: false,
        autoScroll: false,
        readOnly: false,
      },

      text: {
        fill: "rgb(255, 255, 255)",

        font: "25px monospace",
        textAlign: ["left", "top"],
      },
    };
    this.fillStyle(props.style);

    
    //this.style.maxSize.from(this.style.minSize);
    
    this.children = [
      new UIText({
        style: {...this.style,
          minSize: vecZero,
        },
        text: "",
      }),
    ];

    this.setValue("" + this.value);
    

    this.focused = false;

    this.cursor = new Vec(0, 0);
    this.cursor2 = undefined;
    this.cursorTimer = new TimerTime(1000000000);
    this.cursorTimer.loop = true;
    this.maxCursorX = 0;

    this.clicksInRow = 0;
    this.lastCharPos = new Vec(0, 0);


    this.rendererTransform = undefined;
    this.mousedownGlobalFunc = e => {this.mousedownGlobal(e)}
    this.mousemoveGlobalFunc = e => {this.mousemoveGlobal(e)}
    this.mouseupGlobalFunc = e => {this.mouseupGlobal(e)}
    this.keydownGlobalFunc = e => {return this.keydownGlobal(e)}
    this.on("mousedown", e=>{
      if (!this.focused) {
        this.setFocus(true);
      }
      this.focused = true;

      let charPos = this.getCharPos(this.getMousePos());

      if (this.cursorTimer.elapsedTime >= this.style.editor.clickTime || !this.lastCharPos.isEqualTo(charPos)) {
        this.clicksInRow = 0;
      } else {
        this.clicksInRow++;
      }

      this.lastCharPos = charPos;
      this.cursor.from(charPos);
      this.cursor2 = undefined;
      this.cursorTimer.elapsedTime = 0;
      this.maxCursorX = 0;

      if (this.clicksInRow == 1) {
        this.cursor2 = this.cursor._();
        this.fillCursorLeft(this.cursor);
        this.fillCursorRight(this.cursor2);
      }

      if (this.clicksInRow == 2) {
        this.cursor.x = 0;
        this.cursor2 = new Vec(this.getLines()[this.cursor.y].length, this.cursor.y);
      }

      if (this.clicksInRow == 3) {
        let lines = this.getLines();
        this.cursor.set(0, 0);
        this.cursor2 = new Vec(lines[lines.length - 1].length, lines.length - 1);
        
      }

      if (activeSettingText && activeSettingText != this) activeSettingText.setFocus(false);
      activeSettingText = this;
      this.forceHover = true;
      nde.on("mousemove", this.mousemoveGlobalFunc, true);
      nde.on("mouseup", this.mouseupGlobalFunc, true);
    });
  }
  
  setValue(newValue) {
    super.setValue(newValue);
    this.value = "" + this.value;
    
    this.recalculateSize();
    
    if (this.cursor) this.constrainCursor(this.cursor);
    if (this.cursor2) this.constrainCursor(this.cursor2);
  }

  setFocus(newFocus) {
    super.setFocus(newFocus);

    if (this.focused) {
      nde.on("mousedown", this.mousedownGlobalFunc, true);
      nde.on("keydown", this.keydownGlobalFunc, true);
    } else {
      nde.off("mousedown", this.mousedownGlobalFunc);
      nde.off("keydown", this.keydownGlobalFunc);
    }
  }

  recalculateSize() {
    let wasAtBottom = this.contentSize.y - this.size.y <= this.scroll.y;

    this.children[0].text = this.value;
    this.children[0].calculateSize();
    this.calculateSize();

    if (this.style.editor.autoScroll && wasAtBottom) {
      this.scroll.y = this.contentSize.y - this.size.y;
    }
  }

  mousedownGlobal(e) {
    this.setFocus(false);
  }
  mousemoveGlobal(e) {        
    if (this.clicksInRow != 0) return;

    let charPos = this.getCharPos(this.getMousePos());
    this.cursor2 = charPos;

    if (this.cursor.isEqualTo(this.cursor2)) {
      this.cursor2 = undefined;
    }
  }
  mouseupGlobal(e) {
    this.forceHover = false;
    nde.off("mousemove", this.mousemoveGlobalFunc);
    nde.off("mouseup", this.mouseupGlobalFunc);
  }

  keydownGlobal(e) {
    if (["Control", "Shift", "Alt", "AltGraph"].includes(e.key)) return false;
    this.cursorTimer.elapsedTime = this.style.editor.blinkTime;

    let ctrl = e.ctrlKey;
    let shift = e.shiftKey;

    if (e.key == "Escape") {
      this.setFocus(false);
      return false;
    }
    if (e.key.startsWith("Arrow")) {
      let lines = this.getLines();

      let cursor = this.cursor;
      let unselected = false;
      if (shift) {
        if (this.cursor2 == undefined) this.cursor2 = this.cursor._();

        cursor = this.cursor2;
      } else {
        let old = this.cursor;
        if (this.cursor2 != undefined) this.cursor = this.cursor2;
        this.cursor2 = undefined;
        cursor = this.cursor;

        unselected = true;
        if (old.isEqualTo(this.cursor)) unselected = false;
      }


      this.maxCursorX = Math.max(cursor.x, this.maxCursorX);

      if (e.key == "ArrowUp") {
        if (cursor.y == 0){
          cursor.x = 0;
          this.maxCursorX = 0;
        } else {
          cursor.y--;
          cursor.x = Math.min(this.maxCursorX, lines[cursor.y].length);
        }
      }
      if (e.key == "ArrowDown") {
        if (cursor.y == lines.length - 1){
          cursor.x = lines[cursor.y].length;
        } else {
          cursor.y++;
          cursor.x = Math.min(this.maxCursorX, lines[cursor.y].length);
        }
      }
      if (e.key == "ArrowLeft") {
        this.maxCursorX = 0;

        if (ctrl) {
          let x = this.fillCursorLeft(cursor);
          if (x == 0) {
            this.moveCursorLeft(cursor);
            this.fillCursorLeft(cursor);
          }
        } else {
          if (unselected) return false;
          this.moveCursorLeft(cursor);
        }
      }
      if (e.key == "ArrowRight") {
        this.maxCursorX = 0;

        if (ctrl) {
          let x = this.fillCursorRight(cursor);
          if (x == 0) {
            this.moveCursorRight(cursor);
            this.fillCursorRight(cursor);
          }
        } else {
          if (unselected) return false;
          this.moveCursorRight(cursor);
        }
      }

      this.moveScreenToCursor(cursor);
    } else {
      this.maxCursorX = 0;
    }

    if (!this.style.editor.readOnly && e.key == "Backspace") {
      if (this.cursor2 != undefined) this.removeSelected();
      else {
        if (ctrl && this.cursor.x != 0) {
          this.cursor2 = this.cursor._();
          this.fillCursorLeft(this.cursor);
          this.removeSelected();
        } else {
          this.removeAtCursor(this.cursor);
        }
      }

      this.recalculateSize()
    
      this.moveScreenToCursor(this.cursor);
    }
    if (!this.style.editor.readOnly && e.key == "Delete") {
      if (this.cursor2 != undefined) this.removeSelected();
      else {
        if (ctrl) {
          this.cursor2 = this.cursor._();
          this.fillCursorRight(this.cursor2);
          this.removeSelected();
        } else {
          if (this.moveCursorRight(this.cursor))
            this.removeAtCursor(this.cursor);
        }
      }

      this.recalculateSize()

      this.moveScreenToCursor(this.cursor);
    }
    if (ctrl) {
      let key = e.key.toLowerCase();
      let lines = this.getLines();

      if (e.key == "a") {
        this.cursor.set(0, 0);
        this.cursor2 = new Vec(lines[lines.length - 1].length, lines.length - 1);
      }

      if (["c", "x"].includes(key)) {
        let oldCursorPos = undefined;

        if (this.cursor2 == undefined) {
          oldCursorPos = this.cursor._();

          this.cursor.x = 0;
          this.cursor2 = new Vec(lines[this.cursor.y].length, this.cursor.y);
        }

        let string = "";
        if (key == "c") {
          string = this.getChars(this.cursor, this.cursor2);

          if (oldCursorPos) {
            this.cursor = oldCursorPos;
            this.cursor2 = undefined;
            string += "\n";
          }
        }
        if (!this.style.editor.readOnly && key == "x") {
          string = this.removeSelected();

          if (oldCursorPos) {
            this.cursor2 = undefined;
            string += "\n";

            this.moveCursorRight(this.cursor);
            this.removeAtCursor(this.cursor);
          }
        }

        navigator.clipboard.writeText(string);
        
      }
      if (!this.style.editor.readOnly && key == "v") {
        navigator.clipboard.readText().then(string => {
          if (this.cursor2 != undefined) {
            this.removeSelected();
          }
          this.addAtCursor(this.cursor, string);
          this.moveScreenToCursor(this.cursor);
          
          this.recalculateSize()
        });
      }



      if (["a", "c", "v", "x"].includes(key)) return;
    }


    let newText = "";
    
    if (e.key == "Enter") {
      if (this.style.editor.multiLine) {
        newText = "\n";
      } else {
        this.fireChange(true);
        this.setFocus(false);
      }
    }
    if (e.key.length == 1) newText = e.key;

    if (this.style.editor.numberOnly && !["0", "1", "2", "3", "4", "5", "6", "7", "8", "9", ".", "*", "/", "+", "-", "(", ")"].includes(newText)) return false;


    if (!this.style.editor.readOnly) {
      if (newText) {
        if (this.cursor2 != undefined) this.removeSelected();
        this.addAtCursor(this.cursor, newText);
        
        this.recalculateSize()
        this.moveScreenToCursor(this.cursor);
      }


      this.fireInput();
    }
    
    return false;
  }

  getMousePos() {
    let mousePoint = new DOMPoint(nde.mouse.x, nde.mouse.y);
    let transformedMousePoint = mousePoint.matrixTransform(this.rendererTransform.inverse());
    return new Vec(transformedMousePoint.x, transformedMousePoint.y).subV(this.pos).sub(this.style.padding).addV(this.scroll);
  }
  getCharPos(pos) {
    nde.renderer.setAll(this.hovered ? this.style.hover.text : this.style.text);
    let size = nde.renderer.measureText("a");
    
    let p = pos._divV(size);

    let split = this.getLines();
    p.y = Math.max(Math.min(Math.floor(p.y), split.length - 1), 0);
    p.x = Math.max(Math.min(Math.round(p.x), split[p.y].length), 0);
    
    return p;
  }
  getCharActualPos(charPos) {
    let size = nde.renderer.measureText("a");

    return charPos._mulV(size).addV(this.pos).add(this.style.padding).subV(this.scroll);
  }

  getMinPos(pos1, pos2) {
    let top = pos1;
    let bottom = pos2;
    if (bottom.y < top.y || (bottom.y == top.y && bottom.x < top.x)) {
      let t = top;
      top = bottom;
      bottom = t;
    }

    return top;
  }

  moveCursorRight(cursor) {
    let lines = this.getLines();

    if (cursor.x == lines[cursor.y].length) {
      if (cursor.y == lines.length - 1) return false;

      cursor.y++;
      cursor.x = 0;
    } else {
      cursor.x++;
    }
    return true;
  }
  moveCursorLeft(cursor) {
    let lines = this.getLines();

    if (cursor.x == 0) {
      if (cursor.y == 0) return false;

      cursor.y--;
      cursor.x = lines[cursor.y].length;
    } else {
      cursor.x--;
    }
    return true;
  }

  getLines() {
    return this.value.split("\n");
  }
  getCharSpans(pos1, pos2) {
    let split = this.getLines();

    let top = pos1;
    let bottom = pos2;
    if (bottom.y < top.y || (bottom.y == top.y && bottom.x < top.x)) {
      let t = top;
      top = bottom;
      bottom = t;
    }

    let spans = [];
    for (let y = top.y; y <= bottom.y; y++) {
      let span = new Vec(0, y, 0);

      if (y == top.y) span.x = top.x; 
      else span.x = 0;

      if (y == bottom.y) span.z = Math.min(bottom.x, split[y].length) - span.x;
      else span.z = split[y].length - span.x;

      spans.push(span);
    }

    return spans;
  }
  getChars(pos1, pos2) {
    let lines = this.getLines();
    let spans = this.getCharSpans(pos1, pos2);

    let strings = [];
    for (let span of spans) {
      strings.push(lines[span.y].substr(span.x, span.x + span.z));
    }
    return strings.join("\n");
  }
  getCharIndex(pos) {
    let lines = this.getLines();

    let index = pos.x;
    for (let y = 0; y < pos.y; y++) {
      index += lines[y].length + 1;
    }

    return index;
  }
  
  isCursorEmpty(cursor) {
    let line = this.getLines()[cursor.y];

    return ((cursor.x == 0 || !isAlphaNumeric(line[cursor.x - 1])) && (cursor.x == line.length || !isAlphaNumeric(line[cursor.x])))
  }
  fillCursorLeft(cursor) {
    let line = this.getLines()[cursor.y];
    let isEmpty = this.isCursorEmpty(cursor);

    for (let x = 0; x < 1000; x++) {      
      if (cursor.x == 0) return x;

      let charIsEmpty = !isAlphaNumeric(line[cursor.x - 1]);
      if (isEmpty != charIsEmpty) return x;

      cursor.x--;
    }
    return 1000;
  }
  fillCursorRight(cursor) {
    let line = this.getLines()[cursor.y];
    let isEmpty = this.isCursorEmpty(cursor);

    for (let x = 0; x < 1000; x++) {      
      if (cursor.x == line.length) return x;    

      let charIsEmpty = !isAlphaNumeric(line[cursor.x]);
      if (isEmpty != charIsEmpty) return x;

      cursor.x++;
    }
    return 1000;
  }

  removeAtCursor(cursor) {
    let lines = this.getLines();
    let line = lines[cursor.y];

    let char = "";

    if (cursor.x == 0) {
      if (cursor.y == 0) return char;

      cursor.x = lines[cursor.y - 1].length;
      lines[cursor.y - 1] += line;
      lines.splice(cursor.y, 1);
      cursor.y--;

      char = "\n";
    } else {
      lines[cursor.y] = line.slice(0, cursor.x - 1) + line.slice(cursor.x, line.length);
      cursor.x--;
      char = line.slice(cursor.x - 1, cursor.x);
    }

    this.value = lines.join("\n");
    
    return char;
  }
  removeSelected() {
    let i1 = this.getCharIndex(this.cursor);
    let i2 = this.getCharIndex(this.cursor2);

    let top = Math.min(i1, i2);
    let bottom = Math.max(i1, i2);

    let string = this.value.slice(top, bottom);
    this.value = this.value.slice(0, top) + this.value.slice(bottom, this.value.length);

    this.cursor = this.getMinPos(this.cursor, this.cursor2);
    this.cursor2 = undefined;

    return string;
  }

  addAtCursor(cursor, text) {
    let lines = this.getLines();
    let line = lines[cursor.y];

    lines[cursor.y] = line.slice(0, cursor.x) + text + line.slice(cursor.x, line.length);
    this.value = lines.join("\n");

    for (let i = 0; i < text.length; i++) {
      this.moveCursorRight(cursor);
    }

    
    return;
  }


  moveScreenToCursor(cursor) {
    let pos = this.getCharActualPos(cursor).subV(this.pos).addV(this.scroll);
    let size = nde.renderer.measureText("i").y;
    
    this.scroll.x = Math.max(Math.min(this.scroll.x, pos.x - this.style.padding * 2), pos.x - this.size.x + this.style.padding * 2);
    this.scroll.y = Math.max(Math.min(this.scroll.y, pos.y - this.style.padding * 2), pos.y - this.size.y + this.style.padding * 2 + size);
    
    this.constrainScroll();
    this.positionChildren();
  }
  constrainCursor(cursor) {
    let lines = this.getLines();
    if (cursor.y >= lines.length) cursor.y = lines.length - 1;
    let line = lines[this.cursor.y];
    if (cursor.x > line.length) cursor.x = line.length;
  }

  render() {        
    this.rendererTransform = nde.renderer.getTransform();

    super.render();    
    nde.renderer.setAll(this.hovered ? this.style.hover.text : this.style.text);

    let cursor = this.cursor2 || this.cursor;
    let cursorPos = this.getCharActualPos(cursor);
    let cursorSize = nde.renderer.measureText("i");
    cursorSize.x = cursorSize.x * 0.15;
    cursorPos.x -= cursorSize.x / 2;

    nde.renderer.clipRect(this.pos._add(this.style.padding), this.size._sub(this.style.padding * 2), () => {
      this.children[0].text = this.value;

      if (this.focused) {        
        if (!this.style.editor.readOnly && (this.cursorTimer.elapsedTime / this.style.editor.blinkTime) % 1 < 0.5) {
          nde.renderer.rect(cursorPos, cursorSize);
        }

        if (this.cursor2 != undefined) {
          nde.renderer.set("filter", "opacity(20%)");
          let spans = this.getCharSpans(this.cursor, this.cursor2);
          let size = nde.renderer.measureText("a");
          for (let i = 0; i < spans.length; i++) {
            let span = spans[i];
            if (i < spans.length - 1) span.z++;
            nde.renderer.rect(this.getCharActualPos(span), new Vec(size.x * span.z, size.y));
          }
        }
      }
    });
  }
}



//https://stackoverflow.com/questions/4434076/best-way-to-alphanumeric-check-in-javascript
function isAlphaNumeric(char) {
  let code = char.charCodeAt(0);
  return ((code > 47 && code < 58) || // numeric (0-9)
      (code > 64 && code < 91) || // upper alpha (A-Z)
      (code > 96 && code < 123)) // lower alpha (a-z)
  
};





/* src/ui/settings/UISettingRGB.js */
class UISettingRGB extends UISettingCollection {
  
}





/* src/timers/TimerBase.js */
class TimerBase {
  constructor(callback = (timer) => {}) {
    this.elapsedFrames = 0;
    this.elapsedTime = 0;
    this.progress = 0;

    this.playing = false;
    this.loop = false;

    this.callback = callback;

    this.start();
  }

  calculateProgress() {}

  tick(dt) {
    this.elapsedFrames++; 
    this.elapsedTime += dt;
    
    this.calculateProgress();

    this.callback(dt);

    if (this.progress == 1 && this.playing) {
      this.stop();

      if (!this.loop) return

      if (this.lengthFrames) this.elapsedFrames -= this.lengthFrames; else this.elapsedFrames = 0;
      if (this.lengthTime) this.elapsedTime -= this.lengthTime; else this.elapsedTime = 0;
      this.progress = this.calculateProgress();
      this.start();
    }
  }
  
  stop() {
    let index = nde.timers.indexOf(this);
    if (index != -1) nde.timers.splice(index, 1);
    this.playing = false;
  }
  start() {
    nde.timers.push(this);
    this.playing = true;
  }

  reset() {
    this.elapsedFrames = 0;
    this.elapsedTime = 0;
    this.progress = 0;

    this.stop();
    this.start();
  }
}





/* src/timers/TimerFrames.js */
class TimerFrames extends TimerBase {
  constructor(frames, callback) {
    super(callback);

    this.lengthFrames = frames;
  }

  calculateProgress() {
    this.progress = Math.min(this.elapsedFrames / this.lengthFrames, 1);
    return this.progress;
  }
}





/* src/timers/TimerTime.js */
class TimerTime extends TimerBase {
  constructor(seconds, callback) {
    super(callback);

    this.lengthTime = seconds;
  }

  calculateProgress() {
    this.progress = Math.min(this.elapsedTime / this.lengthTime, 1);
    return this.progress;
  }
}





/* src/transitions/TransitionBase.js */
class TransitionBase {
  constructor(newScene, timer) {
    this.oldImg = new Img(new Vec(nde.w, nde.w * nde.ar));
    this.newImg = new Img(this.oldImg.size);

    this.timer = timer;

    nde.fire("render");
    this.oldImg.ctx.imageSmoothingEnabled = false;
    this.oldImg.image(nde.renderer, vecZero, this.oldImg.size);

    nde.setScene(newScene);
    newScene.update(1/60);
    nde.fire("render");
    this.newImg.ctx.imageSmoothingEnabled = false;
    this.newImg.image(nde.renderer, vecZero, this.newImg.size);
  }

  render() {
    nde.renderer.set("fill", 0);
    nde.renderer.rect(new Vec(0, 0), nde.mainImg.size);

    if (this.timer.progress == 1) nde.transition = undefined;
  }
}





/* src/transitions/TransitionFade.js */
class TransitionFade extends TransitionBase {
  constructor(newScene, timer) {
    super(newScene, timer);
  }

  render() {
    super.render();
    
    nde.renderer._(()=>{
      nde.renderer.set("filter", `opacity(${(1-this.timer.progress) * 100}%)`);
      nde.renderer.image(this.oldImg, new Vec(0, 0), this.oldImg.size);
      nde.renderer.set("filter", `opacity(${this.timer.progress * 100}%)`);
      nde.renderer.image(this.newImg, new Vec(0, 0), this.newImg.size);
    });
  }
}





/* src/transitions/TransitionSlide.js */
class TransitionSlide extends TransitionBase {
  constructor(newScene, timer) {
    super(newScene, timer);
  }

  render() {
    super.render();

    let a = this.oldImg.size.x * this.timer.progress;
    let b = this.oldImg.size.x * (1 - this.timer.progress);

    let ctx = nde.renderer.ctx;
    
    nde.renderer._(()=>{
      ctx.beginPath();
      ctx.rect(0, 0, a, this.oldImg.size.y);
      ctx.clip();
      nde.renderer.image(this.newImg, new Vec(0, 0), this.oldImg.size);
    });
    nde.renderer._(()=>{
      ctx.beginPath();
      ctx.rect(a, 0, b, this.oldImg.size.y);
      ctx.clip();
      nde.renderer.image(this.oldImg, new Vec(0, 0), this.oldImg.size);
    });
  }
}





/* src/transitions/TransitionNoise.js */
class TransitionNoise extends TransitionBase {
  constructor(newScene, timer, sliding = false, w = 432) {
    super(newScene, timer);

    this.sliding = sliding;

    this.noiseTexture = new Img(new Vec(w, w / 16 * 9));
    this.outImage = new Img(nde.renderer.size);
    this.outImage.ctx.imageSmoothingEnabled = false;
  }

  render() {
    super.render();
    let lastRandom = 0;
    const random = () => {
      lastRandom = (1103515245  * lastRandom + 12345) % (2 ** 31);
      return lastRandom / 10000000 % 1;
    };


    let data = this.noiseTexture.ctx.getImageData(0, 0, this.noiseTexture.size.x, this.noiseTexture.size.y);
    let i, threshold;
    for (let x = 0; x < this.noiseTexture.size.x; x++) {
      for (let y = 0; y < this.noiseTexture.size.y; y++) {
        i = (x + y * this.noiseTexture.size.x) * 4;
        threshold = this.timer.progress;

        if (this.sliding) {
          threshold = (this.timer.progress * 1.3 - x/this.noiseTexture.size.x) * 2;
        }

        if (random() < threshold) {
          data.data[i  ] = 255;
          data.data[i+1] = 255;
          data.data[i+2] = 255;
          data.data[i+3] = 255;
        } else {
          data.data[i  ] = 0;
          data.data[i+1] = 0;
          data.data[i+2] = 0;
          data.data[i+3] = 0;
        }
      }
    }

    this.noiseTexture.ctx.putImageData(data, 0, 0);

    nde.renderer.image(this.oldImg, vecZero, nde.renderer.size);

    this.outImage.image(this.noiseTexture, vecZero, nde.renderer.size);
    this.outImage.ctx.globalCompositeOperation = 'source-in';
    this.outImage.image(this.newImg, vecZero, nde.renderer.size);
    this.outImage.ctx.globalCompositeOperation = 'source-over';

    nde.renderer.image(this.outImage, vecZero, nde.renderer.size);
  }
}





/* src/ECS/components/Component.js */
class Component extends Serializable {
  constructor(props = {}) {
    super();

    this.hasStarted = false;
    this.hasInit = false;

    this.ob = undefined;
    this.transform = undefined;

    this.lastActive = false;
    this.active = props.active == undefined ? true : props.active;
    this.visible = props.visible == undefined ? true : props.visible;

    this.clientOnly = props.clientOnly || false;
  }
  

  init() {}
  start() {}
  update(dt) {}
  render() {}
  
  enable() {}
  disable() {}
  remove() {}


  on(...args) {return this.ob.on(...args)}
  off(...args) {return this.ob.off(...args)}
  fire(...args) {return this.ob.fire(...args)}

  getComponent(...args) {return this.ob.getComponent(...args); }
  getComponents(...args) {return this.ob.getComponent(...args); }
  find(...args) {return this.ob.getComponent(...args); }
  findAll(...args) {return this.ob.getComponent(...args); }
  findId(...args) {return this.ob.getComponent(...args); }
  addComponent(...args) {return this.ob.addComponent(...args); }
  removeComponent(...args) {return this.ob.addComponent(...args); }

  from(data) {
    super.from(data);

    if (data.active != undefined) this.active = data.active;
    if (data.visible != undefined) this.visible = data.visible;

    if (data.clientOnly != undefined) this.clientOnly = data.clientOnly;

    if (data.hasInit != undefined) this.hasInit = data.hasInit;

    return this;
  }
  strip() {
    delete this.ob;
    delete this.transform;
    delete this.hasStarted;
  }
}





/* src/ECS/components/Transform.js */
class Transform extends Component {
  constructor(props = {}) {
    super();

    this.pos = props.pos || new Vec(0, 0);
    this.dir = props.dir || 0;
    this.size = props.size || new Vec(1, 1);
  }

  from(data) {
    super.from(data);

    this.pos = new Vec().from(data.pos);
    this.dir = data.dir;
    this.size = new Vec().from(data.size);

    return this;
  }
}





/* src/ECS/components/Sprite.js */
class Sprite extends Component {
  constructor(texOrTexture) {
    super();

    this._tex = texOrTexture;

    this.texture = undefined;    

    //For animations/state machines
    this.animation = undefined;
    this._speed = 1;
    this.stateMachineImg = undefined;
  }

  set tex(value) {
    this._tex = value;
    
    this.texture = undefined;
    this.animation = undefined;
    this.stateMachineImg = undefined;
  }
  get tex() {
    return this._tex;
  }
  set speed(value) {
    if (this.stateMachineImg) {
      this.stateMachineImg.speed = value;

    } else if (this.texture instanceof RunningAnimation) {
      this.texture.speed = value;      
      
    }

    this._speed = value;
  }
  get speed() {
    return this._speed;
  }

  start() {
    this.ob.sprite = this;
  }

  updateTexture() {
    if (!this.tex) return;

    if (!this.texture) {
      this.tex = nde.getTex(this.tex);
      let texture = nde.tex[this.tex];

      if (texture instanceof Animation) {
        this.animation = texture;
        this.texture = this.animation.start({listeners: [this.e]});
        this.texture.speed *= this.speed;        

      } else if (texture instanceof RunningAnimation) {
        this.texture = texture;
        this.texture.speed *= this.speed;        
        this.texture.e.listeners.push(this.ob.e);

      } else if (texture instanceof StateMachineImg) {
        this.stateMachineImg = texture;
        this.stateMachineImg.speed = this.speed;
        this.stateMachineImg.e.listeners.push(this.ob.e);

      } else {
        this.texture = texture;
      }
    }

    if (this.stateMachineImg) {
      this.texture = this.stateMachineImg.choose();
    }
  }

  render() {
    if (!this.tex) return;

    this.updateTexture();
    
    nde.renderer._(() => {
      nde.renderer.translate(this.transform.pos);
      if (this.transform.dir) nde.renderer.rotate(this.transform.dir);

      nde.renderer.image(this.texture, this.transform.size._mul(-0.5), this.transform.size);
    });
  }

  from(data) {
    super.from(data);

    this.tex = data._tex;
    this.speed = data._speed;

    return this;
  }
  strip() {
    delete this.texture;
    delete this.animation;
    delete this.stateMachineImg;
    delete this.ob.sprite;
    super.strip();
  }
}





/* src/ECS/components/TextRenderer.js */
class TextRenderer extends Component {
  constructor(text = "", style = {}) {
    super();

    this.text = text;
    this.style = {
      fill: style.fill || "rgb(255,255,255)",
      textAlign: style.textAlign || ["center", "middle"],
      font: style.font || "1px monospace",
    };
  }

  render() {
    nde.renderer._(() => {
      nde.renderer.setAll(this.style);
      nde.renderer.text(this.text, this.transform.pos);
    });
  }

  from(data) {
    super.from(data);

    this.text = data.text;
    this.style = data.style;

    return this;
  }
}





/* src/ECS/components/AudioSource.js */
class AudioSource extends Component {
  constructor(props = {}) {
    super();

    this._gain = undefined;
    this.playing = [];

    this.gain = props.gain != undefined ? props.gain : 1;
  }

  set gain(value) {
    this._gain = value;
    for (let i = 0; i < this.playing.length; i++) {
      this.playing[i].gain = value;
    }
  }
  get gain() {
    return this._gain;
  }


  start() {
    this.ob.audioSource = this;

    this.lastPos = this.transform.pos.copy();
  }


  update() {
    let hasMoved = !this.transform.pos.isEqualTo(this.lastPos);
    this.lastPos.from(this.transform.pos);

    for (let i = 0; i < this.playing.length; i++) {
      let aud = this.playing[i];
      if (!aud.isPlaying) {
        this.playing.splice(i, 1);
        i--;
        continue;
      }

      if (hasMoved) {
        aud.setPosition(this.transform.pos.x, 1, this.transform.pos.y);
      }
    }

  }


  play(audPool) {
    let aud = playAudio(audPool, this.transform.pos);
    aud.gain = this.gain;
    this.playing.push(aud);
  }

  from(data) {
    super.from(data);

    this.gain = data._gain;

    return this;
  }

  strip() {
    delete this.ob.audioSource;

    super.strip();
  }
}





/* src/ECS/Ob.js */
let DEFAULTTRANSFORMCOMPONENT = Transform;

class Ob extends Serializable {
  constructor(props = {}, components = [], children = []) {
    super();

    this.name = props.name || "";
    this.children = [];
    this.id = props.id;
    if (this.id == undefined) this.randomizeId();

    this.lastActive = false;
    this.active = props.active == undefined ? true : props.active;
    this.visible = props.visible == undefined ? true : props.visible;

    this.components = components;
    this.transform = this.getComponent(DEFAULTTRANSFORMCOMPONENT);
    if (!this.transform) {
      this.transform = new DEFAULTTRANSFORMCOMPONENT({
        pos: props.pos,
        dir: props.dir,
        size: props.size,
      });
      this.components.unshift(this.transform);
    }

    for (let c of this.components) {
      c.ob = this;
      c.transform = this.transform;
    }


    this.parent = undefined;
    this.appendChild(...children);


    this.e = new EventHandler();
  }


  update(dt) {
    if (!this.active) {
      if (this.lastActive) {
        for (let i = 0; i < this.components.length; i++) {
          let c = this.components[i];
          if (c.active) {
            c.enable();
            c.lastActive = false;
          }
        }

        this.lastActive = false;
      }
      return;
    }

    if (!this.lastActive) {
      for (let i = 0; i < this.components.length; i++) {
        let c = this.components[i];
        if (c.active) {
          c.enable();
          c.lastActive = true;
        }
      }
    }

    for (let i = 0; i < this.components.length; i++) {
      let c = this.components[i];

      if (!c.hasInit) {
        c.init();
        c.hasInit = true;
      }
      if (!c.hasStarted) {
        c.start();
        c.hasStarted = true;
      }

      if (!c.active) {
        if (c.lastActive) c.disable();
        c.lastActive = false;
        continue;
      }
      if (!c.lastActive) {
        c.enable();
        c.lastActive = true;
      }

      c.update(dt);
    }

    for (let i = 0; i < this.children.length; i++) {
      this.children[i].update(dt);
    }

    this.lastActive = this.active;
  }
  render() {
    for (let i = 0; i < this.components.length; i++) {
      if (this.components[i].visible) this.components[i].render();
    }

    for (let i = 0; i < this.children.length; i++) {
      if (this.children[i].visible) this.children[i].render();
    }
  }
  
  
  on(...args) {return this.e.on(...args)}
  off(...args) {return this.e.off(...args)}
  fire(...args) {return this.e.fire(...args)}


  addComponent(...components) {
    for (let i = 0; i < components.length; i++) {
      let component = components[i];

      if (component.ob) {
        component.ob.removeComponent(component);
      }

      this.components.push(component);
      component.ob = this;
      component.transform = this.transform;
    }
  }
  removeComponent(component) {
    let index = this.components.indexOf(component);
    if (index == -1) return false;

    this.components.splice(index, 1);
    component.ob = undefined;
    component.transform = undefined;
    return true;
  }

  getComponent(type) {
    return this.components.find(e=>{return e instanceof type});
  }
  getComponents(type, limit = 9999, arr = []) {
    let comp = this.components.find(e=>{return e instanceof type});
    if (comp) arr.push(comp);

    for (let i = 0; i < this.children.length; i++) {
      if (arr.length == limit) return arr;
      
      this.children[i].getComponents(type, limit, arr);
    }

    return arr;
  }

  find(fn) {
    let arr = this.findAll(fn, 1);
    if (arr) return arr[0];
  }
  findAll(fn, limit = 9999, arr = []) {
    if (fn(this)) arr.push(this);

    for (let i = 0; i < this.children.length; i++) {
      if (arr.length == limit) return arr;
      
      this.children[i].findAll(fn, limit, arr);
    }

    return arr;
  }

  findId(id) {
    if (this.id == id) return this;

    for (let i = 0; i < this.children.length; i++) {
      let res = this.children[i].findId(id);
      
      if (res) return res;
    }
  }
  randomizeId() {
    this.id = Math.floor(Math.random() * 1000000);

    for (let i = 0; i < this.children.length; i++) this.children[i].randomizeId();

    return this.id;
  }
  createLookupTable(table = {}) {
    table[this.id] = this;

    for (let i = 0; i < this.children.length; i++) {
      this.children[i].createLookupTable(table);
    }
    
    return table;
  }

  appendChild(...obs) {
    for (let i = 0; i < obs.length; i++) {
      let ob = obs[i];
      
      if (ob.parent) {
        ob.parent.removeChild(ob);
      }

      this.children.push(ob);
      ob.parent = this;
    }
  }
  removeChild(ob) {
    let index = this.children.indexOf(ob);
    if (index == -1) return false;

    this.children.splice(index, 1);
    ob.parent = undefined;
    return true;
  }
  replaceChild(original, ...obs) {
    let index = this.children.indexOf(original);
    if (index == -1) return false;

    for (let i = 0; i < obs.length; i++) {
      let ob = obs[i];
      if (ob.parent) ob.parent.removeChild(ob);
      ob.parent = this;
    }

    this.children.splice(index, 1, ...obs);
    original.parent = undefined;
    return true;
  }
  setParent(ob) {
    if (this.parent) this.parent.removeChild(this);
    ob.appendChild(this);
  }


  
  remove() {
    if (this.parent) this.parent.removeChild(this);

    for (let i = 0; i < this.components.length; i++) {
      let c = this.components[i];
      if (c.lastActive) c.disable();
      c.remove();
    }

    for (let i = 0; i < this.children.length; i++) {
      this.children[i].remove();
    }
  }
  replace(...obs) {
    if (!this.parent) return false;

    this.parent.replaceChild(this, ...obs);
    return true;
  }

  copy(randomizeId = false) {
    let ob2 = super.copy();
    if (randomizeId) ob2.randomizeId();
    return ob2;
  }

  from(data) {
    super.from(data);

    this.name = data.name;
    this.id = data.id;

    this._active = false;
    if (data.active) this.active = true;
    if (data.visible != undefined) this.visible = data.visible;

    this.components = [];
    for (let c of data.components) this.components.push(cloneData(c));
    this.transform = this.getComponent(DEFAULTTRANSFORMCOMPONENT);

    for (let i = 0; i < this.components.length; i++) {
      let c = this.components[i];
      c.ob = this;
      c.transform = this.transform;
    }

  
    for (let i = 0; i < data.children.length; i++) {
      let c2 = cloneData(data.children[i]);
      this.appendChild(c2);
    }


    return this;
  }
  
  stripComponents() {
    for (let i = 0; i < this.components.length; i++) {
      this.components[i].strip();
    }

    for (let i = 0; i < this.children.length; i++) {
      this.children[i].stripComponents();
    }
  }
  stripObs() {
    delete this.transform;
    delete this.parent;
    delete this.e;

    for (let i = 0; i < this.children.length; i++) {
      this.children[i].stripObs();
    }
  }
  strip() {
    this.stripComponents();
    this.stripObs();
    
  }
  stripClientComponents() {
    for (let i = 0; i < this.components.length; i++) {
      if (this.components[i].clientOnly) {
        this.removeComponent(this.components[i]);
        i--;
      }
    }

    for (let i = 0; i < this.children.length; i++) {
      this.children[i].stripClientComponents();
    }
  }
}





/* src/index.js */
/*
scene: the current active scene, is set by setScene, see engine/scenes/scene.js for base class

w: how many pixels are across the screen, is updated automatically on resize

targetFPS: if this is not undefined, the game will try to run at that fps and the deltaTime will be fixed so it will be deterministic

controls: map of control names along with key codes, set by you and used in getKeyPressed
pressed: array of all the currently pressed key codes, accessed with getKeyPressed

mouse: a vector of the mouse position
hoveredUIElement: the currently hovered ui element, but you can't see which one it is sucker

timers: array of all currently active timers, to add a timer call new TimerWhatever, see engine/timers/timerWhatever.js for details

transition: the currently active transition, set this to a new TransitionWhatever which usually takes (newScene, timerWhatever) and it will transition, see engine/transitions/transitionBase.js for base class

debug: if debugStats are shown
debugStats: map thats cleared each frame where you can put debug info

renderer: which renderer to use, pass all drawing code into this pls, see engine/renderers/rendererBase.js for base class



Events:

beforeSetup
afterSetup

keydown
keyup

mousemove
mousedown
mouseup
wheel

resize

update
afterUpdate

render
afterRender

beforeScene
afterScene



use these to handle key input:
getKeyCode: Get keycode from control name
getKeyEqual: Get if keycode is same as controlName

getKeyPressed: Get if key corresponding to control name is pressed
getKeyDown: Get if key corresponding to control name was pressed this frame
getKeyUp: Get if key corresponding to control name was released this frame

.scrolled: how far has scrolled from last frame



important classes:
SceneBase: base class of all scenes, new SceneGame()
RendererBase: base class of all renderers, new RendererCanvas()
TimerBase: base class of all timers, new TimerTime(seconds, callback)
TransitionBase: base class of all transitions, new TransitionSlide(newScene, timer)

Vec: vector, see ndv.js

Img: image, see img.js

Camera: an object that can transform points and apply transformations to renderer, new Camera(pos), see camera.js



Recommended project structure:

Project:
  nde.js
  scenes:
    sceneGame.js
    sceneMainMenu.js
    ...
  ui:
    UIElementCustom.js
    ...
  index.js
  index.html
  style.css
  ...

*/  

let audioContext = new (window.AudioContext || window.webkitAudioContext)();
let assetFolder = "assets";

class NDE {
  constructor(mainElem) {
    this.scene = undefined;
    this.w = undefined;
    this.ar = 9 / 16;
    this.targetFPS = undefined;
    this.transition = undefined;
    this.renderer = new Img(vecOne);
    this.hasStarted = false;

    this.e = new EventHandler();

    this.assetLoaders = {
      "ob": EvalAsset,
      "component": EvalAsset,
      "anim": Animation,
      "png": Img,
      "mp3": Aud,
    };

    this.mouse = new Vec(0, 0);
    this.mainElemBoundingBox = new Vec(0, 0, 1, 1);

    this.controls = {};
    this.pressed = {};
    this.pressedFrame = [];
    this.releasedFrame = [];
    this.scrolled = 0;

    this.timers = [];

    this.debug = false;
    this.lastDebugKey = "";
    this.debugStats = {};

    this.hoveredUIElement = undefined;
    this.hoveredUIRoot = undefined;
    this.uiDebug = false;

    this.resolvePopupFunc = undefined;
    this.scenePopup = undefined;

    this.mainElem = mainElem;
    this.mainImg = new Img(new Vec(1, 1));
    this.unloadedAssets = [];

    this.lastFrameTime = 0;
    this.lastGameDt = 1;
    this.latestDts = [];

    this.setScene(new Scene());

    this.inputManager = undefined;
    this.initInputManager();
    this.setupHandlers();

    let i = 0;
    let lastLength = Infinity;
    let interval = setInterval(e => {
      if (this.unloadedAssets.length == 0) {        
        clearInterval(interval);
        this.hasStarted = true;
        for (let a of unloadedEvalAssets) {
          a.eval();
        }
        this.inputManager.init();
        

        this.fire("beforeSetup");

        this.mainElem.appendChild(this.mainImg.canvas);
        this.inputManager.fire("input_resize");

        this.renderer.set("font", "16px monospace");
        this.renderer.set("textAlign", ["left", "top"]);
        this.renderer.set("imageSmoothing", false);
        
        this.fire("afterSetup");

        requestAnimationFrame(time => {this.animationFrame(time)});
      }
      if (i >= 100) {
        if (this.unloadedAssets.length < lastLength) {
          i = 0;
          lastLength = this.unloadedAssets.length;
        } else {
          clearInterval(interval); 
          console.error("assets could not be loaded: " + this.unloadedAssets.map(e => e.path))
        }
      }
      
      i++;
    }, 50);

  }

  initInputManager() {
    let log = localStorage.getItem("replay-log");
    if (log) {
      let parsed = JSON.parse(log);
      this.inputManager = new InputManagerReplay(parsed.log, parsed.playbackDivisions);
      localStorage.removeItem("replay-log");
    } else {
      this.inputManager = new InputManager();
    }
  }
  replay(playbackDivisions = 1) {
    localStorage.setItem("replay-log", JSON.stringify({log: this.inputManager.log, playbackDivisions: playbackDivisions}));
    window.location.reload();
  }

  setupHandlers() {
    window.addEventListener("resize", e => {
      this.inputManager.fire("input_resize");
    });
    document.addEventListener("keydown", e => {
      this.tryStartAudio();

      this.inputManager.fire("input_keydown", e);
    });
    document.addEventListener("keyup", e => {
      this.inputManager.fire("input_keyup", e);
    });
    document.addEventListener("mousemove", e => {
      this.inputManager.fire("input_mousemove", e, (e.clientX - this.mainElemBoundingBox.x) / this.mainImg.size.x * this.w, (e.clientY - this.mainElemBoundingBox.y) / this.mainImg.size.x * this.w);
    });
    document.addEventListener("mousedown", e => {
      this.tryStartAudio();

      this.inputManager.fire("input_mousedown", e);
    });
    document.addEventListener("mouseup", e => {
      this.inputManager.fire("input_mouseup", e);
    });
    document.addEventListener("wheel", e => {
      this.inputManager.fire("input_wheel", e);
    });

    
    window.oncontextmenu = (e) => {
      e.preventDefault(); 
      e.stopPropagation(); 
      return false;
    };


    this.on("input_keydown", e => {
      if (this.hoveredUIElement) {
        if (this.hoveredUIElement.fire("keydown", e) == false) return;
        if (this.hoveredUIElement.fire("inputdown", e.key.toLowerCase(), e) == false) return;
      }
    
      if (!this.pressed[e.key.toLowerCase()]) {
        if (this.debug) this.lastDebugKey = e.key;
        
        if (!this.fire("keydown", e)) return;
        if (!this.fire("inputdown", e.key.toLowerCase(), e)) return;

        this.pressed[e.key.toLowerCase()] = true;
        this.pressedFrame.push(e.key.toLowerCase());
      }
    });
    this.on("input_keyup", e => {
      delete this.pressed[e.key.toLowerCase()];
      this.releasedFrame.push(e.key.toLowerCase());

      if (this.hoveredUIElement) {
        this.hoveredUIElement.fire("keyup", e);
        this.hoveredUIElement.fire("inputup", e.key.toLowerCase(), e);
      }

      this.fire("keyup", e);
      this.fire("inputup", e.key.toLowerCase(), e);
    });
    this.on("input_mousemove", (e, x, y) => {      
      this.mouse.x = x;
      this.mouse.y = y;

      if (this.hoveredUIElement) {
        this.hoveredUIElement.fire("mousemove", e);
      }
      
      this.fire("mousemove", e);
    });
    this.on("input_mousedown", e => {
      if (this.hoveredUIElement) {
        if (this.hoveredUIElement.fire("mousedown", e) == false) return;
        this.hoveredUIElement.fire("inputdown", "mouse" + e.button, e);
        return;
      }
    

      if (!this.pressed["mouse" + e.button]) {
        if (this.debug) this.lastDebugKey = "mouse" + e.button;

        if (!this.fire("mousedown", e)) return;
        if (!this.fire("inputdown", "mouse" + e.button, e)) return;

        this.pressed["mouse" + e.button] = true;
        this.pressedFrame.push("mouse" + e.button);
      }

    });
    this.on("input_mouseup", e => {
      delete this.pressed["mouse" + e.button];
      this.releasedFrame.push("mouse" + e.button);

      if (this.hoveredUIElement) {
        this.hoveredUIElement.fire("mouseup", e);
        this.hoveredUIElement.fire("inputup", "mouse" + e.button, e);
      }
      
      this.fire("mouseup", e);
      this.fire("inputup", "mouse" + e.button, e);
    });
    this.on("input_wheel", e => {
      if (this.hoveredUIElement) {
        if (this.hoveredUIElement.fire("wheel", e) == false) return;
        if (this.hoveredUIRoot?.wheel(e) == false) return;
      }

      this.scrolled += e.deltaY;
      this.fire("wheel", e);      
    });

    this.on("input_resize", e => {
      this.resize(e);  
    });
    this.on("input_frame", time => {
      if (time == undefined) time = performance.now();
      
      let dt = Math.min(time - this.lastFrameTime, 200);
      
      if (this.targetFPS != undefined) {
        if ((time + 0.1) - this.lastFrameTime < 1000 / this.targetFPS) return; 
      }
    
      this.lastFrameTime = time;
    
      this.updateGame(dt);
    });
  }

  setScene(newScene) {
    if (this.e.events["beforeScene"]) {
      for (let ee of this.e.events["beforeScene"]) {
        let res = ee(newScene); 
        if (res == false) break;
        if (res) {
          newScene = res;
          break;
        }
      };
    }


    if (this.scene) this.scene.stop();
    this.scene = newScene;
    this.scene.start();
    this.scene.hasStarted = true;

    if (this.e.events["afterScene"]) {
      for (let ee of this.e.events["afterScene"]) {
        let res = ee(newScene); 
        if (res == false) break;
      };
    }
  }


  on(...args) {return this.e.on(...args)}
  off(...args) {return this.e.off(...args)}
  fire(eventName, ...args) {
    if (this.e.fire(eventName, ...args) == false) return false;
    if (this.scene[eventName] && this.scene[eventName](...args) == false) return false;
    return true;
  }

  resize(e) {
    this.w = Math.min(window.innerWidth, window.innerHeight / this.ar);
    this.mainImg.resize(new Vec(this.w, this.w * this.ar));

    let box = this.mainElem.getBoundingClientRect();
    this.mainElemBoundingBox.set(box.x, box.y, box.width, box.height);
  

    let result = undefined;
    if  (this.e.events["resize"]) {
      for (let ee of this.e.events["resize"]) {
        let res = ee(e); if (res) result = res
      };
    }

    this.w = result || this.w;
    
    this.renderer.resize(new Vec(this.w, this.w * this.ar));
    
    this.scene.resize(e);
  }

  /**
   * Gets keycode of a control
   * 
   * @param {string} controlName
   * @return {string} keyCode
   */
  getKeyCodes(controlName) {
    return this.controls[controlName].toLowerCase().split(",");
  }
  /**
   * Gets if keycode is equal to control keycode
   * 
   * @param {string} keyCode
   * @param {string} controlName
   * @return {boolean} equal
   */
  getKeyEqual(keyCode, controlName) {
    return this.getKeyCodes(controlName).includes(keyCode.toLowerCase());
  }
  /**
   * Gets if a key is pressed
   * 
   * @param {string} controlName
   * @return {boolean} pressed
   */
  getKeyPressed(controlName) {
    let keyCodes = this.getKeyCodes(controlName);

    for (let i = 0; i < keyCodes.length; i++) {
      if (this.pressed[keyCodes[i]]) return true;
    }
    return false;
  }
  /**
   * Gets if a key got pressed this frame
   * 
   * @param {string} controlName
   * @return {boolean} pressed
   */
  getKeyDown(controlName) {
    let keyCodes = this.getKeyCodes(controlName);

    for (let i = 0; i < keyCodes.length; i++) {
      if (this.pressedFrame.includes(keyCodes[i])) return true;
    }
    return false;
  }
  /**
   * Gets if a key got released this frame
   * 
   * @param {string} controlName
   * @return {boolean} pressed
   */
  getKeyUp(controlName) {
    let keyCodes = this.getKeyCodes(controlName);

    for (let i = 0; i < keyCodes.length; i++) {
      if (this.releasedFrame.includes(keyCodes[i])) return true;
    }
    return false;
  }



  animationFrame(time) {
    requestAnimationFrame(time => {this.animationFrame(time)});
    this.inputManager.fire("input_frame", time);
  }
  updateGame(dt) {
    let t1 = performance.now();

    let gameDt = (this.targetFPS == undefined) ? dt * 0.001 : 1 / this.targetFPS;
    let last = this.lastGameDt;
    this.lastGameDt = gameDt;

    gameDt = Math.min(gameDt, last * 10);
    this.debugStats = {};
  
    this.renderer._(()=>{      
      this.renderer.clearRect(vecZero, this.renderer.size);

      for (let i = 0; i < this.timers.length; i++) this.timers[i].tick(gameDt);
      
      this.scene.lastIndex = 0; 
      this.fire("update", gameDt);
      this.fire("afterUpdate", gameDt);

      let t2 = performance.now();
    
      this.hoveredUIElement = undefined;
      this.hoveredUIRoot = undefined;
      this.fire("render");
      if (this.transition) this.transition.render();
      this.fire("afterRender");


      this.pressedFrame.length = 0;
      this.releasedFrame.length = 0;
      this.scrolled = 0;


      this.latestDts.push(dt);
      if (this.latestDts.length > 10) this.latestDts.shift();
      let averageDt = this.latestDts.reduce((partialSum, a) => partialSum + a, 0) / this.latestDts.length;

      let debugStats2 = {};
      
      debugStats2["updateTime"] = Math.round((t2 - t1) * 10) / 10;
      debugStats2["renderTime"] = Math.round((performance.now() - t2) * 10) / 10;
      debugStats2["fps"] = Math.round(1000 / averageDt);
    
      if (this.targetFPS != undefined) {
        debugStats2["target frameTime"] = 1000 / this.targetFPS;
        debugStats2["target fps"] = this.targetFPS;
      }

      debugStats2["key"] = this.lastDebugKey;


    
      this.renderer.set("fill", 255);
      this.renderer.set("stroke", 0);
      let textSize = 0.012 * this.w;
      this.renderer.set("font", `${textSize}px monospace`);
      this.renderer.set("textAlign", ["left", "top"]);
      if (this.debug) {
        let n = 0;
        for (let i in debugStats2) {
          this.renderer.text(`${i}: ${JSON.stringify(debugStats2[i])}`, new Vec(0, n * textSize));
          n++;
        }
        for (let i in this.debugStats) {
          this.renderer.text(`${i}: ${JSON.stringify(this.debugStats[i])}`, new Vec(0, n * textSize));
          n++;
        }
      }
    });
    
  
    
    this.flushRenderer();
  }
  flushRenderer() {
    this.mainImg.ctx.imageSmoothingEnabled = false;
    this.mainImg.image(this.renderer, vecZero, this.mainImg.size);
  }


  openPopup(ui = new UIBase({})) {
    return new Promise(resolve => {
      this.resolvePopupFunc = resolve;

      if (!this.scenePopup) this.scenePopup = new ScenePopup();
      this.scenePopup.lastScene = this.scene;
      this.scenePopup.captureScreen();
      this.scenePopup.ui.children[0].children[0] = ui;
      this.scenePopup.ui.initUI();
      this.setScene(this.scenePopup);
    });
  }
  resolvePopup(...args) {
    if (!this.resolvePopupFunc) return;
    
    this.resolvePopupFunc(...args);
    this.resolvePopupFunc = undefined;
    this.setScene(this.scenePopup.lastScene);
  }

  loadAsset(assetDescriptor) {
    if (typeof assetDescriptor == "string") assetDescriptor = {path: assetDescriptor};

    let path = assetDescriptor.path;
    let split1 = path.split(".");
    let fileType = split1.splice(-1, 1)[0];
    path = split1.join(".");

    if (!assetDescriptor.name) assetDescriptor.name = path;
    
    let assetLoader = this.assetLoaders[fileType];
    if (!assetLoader) {
      console.error("Unsupported filetype: " + assetDescriptor.path);
      return;
    }
    assetDescriptor.path = assetFolder + "/" + assetDescriptor.path;

    let asset = new assetLoader();
    this.unloadedAssets.push(asset);
    asset.loading = true;
    asset.path = assetDescriptor.path;
    asset.load(assetDescriptor).then(() => {
      asset.loading = false;
      this.unloadedAssets.splice(this.unloadedAssets.indexOf(asset), 1);
    });
    
    return asset;
  }
  getTex(texOrTexture) {
    if (typeof texOrTexture == "string") return texOrTexture;
   
    let index = Object.keys(nde.tex).find(key => nde.tex[key] == texOrTexture);
    if (index != undefined) return index;
    
    index = Math.floor(Math.random() * 100000) + "";
    nde.tex[index] = texOrTexture;
    
    return index;
  }
  tryStartAudio() {
    if (audioContext.state != "running") {
      audioContext.resume();
    
      
      setTimeout(() => {
        this.fire("audioContextStarted");        
      }, 0);
    }
  }
}



var getDeltaAngle = function () {
  var TAU = 2 * Math.PI;
  var mod = function (a, n) { return ( a % n + n ) % n; } // modulo
  var equivalent = function (a) { return mod(a + Math.PI, TAU) - Math.PI } // [-π, +π]
  return function (current, target) {
    return equivalent(target - current);
  }
}();
let deg2rad = Math.PI / 180;
let rad2deg = 180 / Math.PI;





