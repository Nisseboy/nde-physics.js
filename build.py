import sys

files = [
  "src/Collider/ColliderBase.js",
  "src/Collider/ColliderCircle.js",
  "src/Collider/ColliderRect.js",
  "src/Collider/Collision.js",

  "src/Constraint/ConstraintBase.js",
  "src/Constraint/ConstraintWeld.js",

  "src/PhysicsObject.js",

  "src/PhysicsGrid.js",

  "src/index.js",
] 
outputFile = "release/nde-physics" + sys.argv[1] + ".js"
fileHeader = '''
/*
This is a built version of nde-physics and is all the source files stitched together, go to the github for source


*/
'''


output = fileHeader
for f in files:
  with open(f) as phille: output += "/* " + f + " */" + "\n" + phille.read() + "\n\n\n\n\n\n"

with open(outputFile, "w+") as f: f.write(output)