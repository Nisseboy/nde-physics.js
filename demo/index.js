let nde = new NDE(document.getElementsByTagName("main")[0]); //nde needs to be defined globally and be the single instance of NDE
nde.debug = true;
nde.uiDebug = false;
//nde.targetFPS = 60;

let renderer = nde.renderer;


let scenes = {};
nde.controls = {
  "Move Up": "w",
  "Move Down": "s",
  "Move Left": "a",
  "Move Right": "d",

  "Pause": "Escape",
  "Debug Mode": "l",
  "UI Debug Mode": "k",
};


nde.registerEvent("keydown", e => {
  if (nde.getKeyEqual(e.key,"Debug Mode")) nde.debug = !nde.debug;
  if (nde.getKeyEqual(e.key,"UI Debug Mode")) nde.uiDebug = !nde.uiDebug;
});

nde.registerEvent("afterSetup", () => {
  scenes = {
    game: new SceneGame(), 
  };

  scenes.game.loadWorld(new World());
  nde.setScene(scenes.game);
});

nde.registerEvent("update", dt => {
  renderer.set("font", "16px monospace");
  renderer.set("imageSmoothing", false);
});

nde.registerEvent("resize", e => {
  return nde.w// * settings.renderResolution / 100;
  //return 432; //new width
});




//https://gist.github.com/yomotsu/165ba9ee0dc991cb6db5
var getDeltaAngle = function () {
  var TAU = 2 * Math.PI;
  var mod = function (a, n) { return ( a % n + n ) % n; } // modulo
  var equivalent = function (a) { return mod(a + Math.PI, TAU) - Math.PI } // [-π, +π]
  return function (current, target) {
    return equivalent(target - current);
  }
}();