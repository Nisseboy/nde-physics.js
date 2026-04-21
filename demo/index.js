let nde = new NDE(document.getElementsByTagName("main")[0]); //nde needs to be defined globally and be the single instance of NDE
nde.debug = true;
nde.uiDebug = false;
//nde.targetFPS = 60;

let renderer = nde.renderer;
for (let asset of assetPaths) {
  nde.loadAsset(asset);
}

let scenes = {};
nde.controls = {
  "Debug Mode": "l",
  "UI Debug Mode": "k",
};


nde.on("keydown", e => {
  if (nde.getKeyEqual(e.key,"Debug Mode")) nde.debug = !nde.debug;
  if (nde.getKeyEqual(e.key,"UI Debug Mode")) nde.uiDebug = !nde.uiDebug;
});

nde.on("afterSetup", () => {
  initStyles();
  
  for (let path of scenePaths) {
    let name = path.split("Scene")[1];
    name = name[0].toLowerCase() + name.slice(1);
    scenes[name] = new (eval(path))();
  }


  scenes.game.loadWorld(new Ob());
  nde.setScene(scenes.game);
});

nde.on("update", dt => {
  renderer.set("font", "16px monospace");
  renderer.set("imageSmoothing", false);
});


//For nde-Editor
function getContext() {
  return {
    nde,
    scenes,
  }
}