const assert = require("assert");
const fs = require("fs");
const vm = require("vm");

const toolPath =
  "/home/nvidia/PC_WJ/巡检路线标注工具/route_map_tool.html";
const html = fs.readFileSync(toolPath, "utf8");

function extractFunction(name) {
  const start = html.indexOf(`function ${name}(`);
  assert.notStrictEqual(start, -1, `${name}() not found`);

  const bodyStart = html.indexOf("{", start);
  let depth = 0;
  for (let index = bodyStart; index < html.length; index += 1) {
    const char = html[index];
    if (char === "{") depth += 1;
    if (char === "}") depth -= 1;
    if (depth === 0) return html.slice(start, index + 1);
  }
  throw new Error(`${name}() body not closed`);
}

function loadParseYaml() {
  const context = {
    state: {
      map: {
        resolution: 0.05,
        origin: [-2.89, -6.37, 0],
      },
    },
    rebuildMapBitmap() {},
    renderAll() {},
  };
  vm.createContext(context);
  vm.runInContext(extractFunction("parseYaml"), context);
  return context;
}

function testBlockOrigin() {
  const context = loadParseYaml();
  context.parseYaml(`image: my_map.pgm
mode: trinary
resolution: 0.05
origin:
- -5.89
- -13.3
- 0
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
`);

  assert.deepStrictEqual(Array.from(context.state.map.origin), [-5.89, -13.3, 0]);
  assert.strictEqual(context.state.map.resolution, 0.05);
  assert.strictEqual(context.state.map.image, "my_map.pgm");
}

function testInlineOrigin() {
  const context = loadParseYaml();
  context.parseYaml("resolution: 0.05\norigin: [-5.89, -13.3, 0]\n");

  assert.deepStrictEqual(Array.from(context.state.map.origin), [-5.89, -13.3, 0]);
}

testBlockOrigin();
testInlineOrigin();
