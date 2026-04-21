
let backgroundCol = "rgb(19, 19, 19)";
let buttonCol = "rgb(25, 25, 25)";
let buttonHoveredCol = "rgb(40, 40, 40)";

let uicam;
let buttonStyle;
let rangeStyle;
let rootStyle;

function initStyles() {
  let w = 1600;
  uicam = new Camera(new Vec(w*0.5, w*nde.ar*0.5));
  uicam.w = w;

  buttonStyle = {
    minSize: new Vec(30, 30),
    padding: 5,


    fill: buttonCol,
    text: {font: "25px monospace"},
    
    
    scroll: {
      fill: "rgb(60, 60, 60)",
    },

    slider: {
      padding: 5,
      minSize: new Vec(300, 30),

      fill: buttonCol,
    },

    number: {
      fill: buttonCol,
    },

    hover: {
      fill: buttonHoveredCol,

      
      slider: {
        fill: buttonHoveredCol,
        active: {
          fill: "rgb(255, 0, 0)",
          stroke: "rgb(255, 0, 0)",
        }
      },
      number: {
        fill: buttonHoveredCol,
      }
    }
  };

  rangeStyle = {...buttonStyle,
    padding: 0,
  };


  rootStyle = {
    size: new Vec(w, w*uicam.ar),
    scroll: {...buttonStyle.scroll,
      width: buttonStyle.padding,
    },
  };
}

function createDefaultUIRoot(children) {
  return new UIRoot({
    pos: new Vec(0, 0),

    style: rootStyle,
    
    children: [
      new UIBase({
        style: {
          direction: "column",
          padding: 50,
          gap: 10,
        },

        children: children,
      })],
  });   
}