const assert = require("assert");
const fs = require("fs");
const vm = require("vm");

const toolPath =
  "tools/route_map_tool/route_map_tool.html";
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

function loadRouteJsonContext() {
  const inputs = {
    routeVersion: { value: "3" },
    startName: { value: "" },
    startX: { value: "0" },
    startY: { value: "0" },
    startYaw: { value: "0" },
    publishInitialPose: { checked: true },
    covX: { value: "0.25" },
    covY: { value: "0.25" },
    covYaw: { value: "0.0685" },
    routeId: { value: "route_patrol_001" },
    activeRouteId: { value: "route_patrol_001", dataset: {} },
    routeName: { value: "本地巡逻路线" },
    returnToStart: { checked: true },
    loopEnabled: { checked: false },
    loopWait: { value: "600" },
    maxCycles: { value: "0" },
    goalTimeout: { value: "120" },
    maxRetries: { value: "1" },
    failurePolicy: { value: "abort_and_return_home" },
  };
  const context = {
    state: {
      map: {
        yamlName: "my_map.yaml",
        pgmName: "my_map.pgm",
        image: "my_map.pgm",
        resolution: 0.025,
        origin: [-7.07, -13.3, 0],
        width: 395,
        height: 675,
        imageSha256: "a".repeat(64),
      },
      zones: [],
      activeZoneId: null,
      targets: [],
      selectedTargetId: null,
      yawTarget: null,
      nextTargetNo: 1,
    },
    els: inputs,
    Number,
    Math,
  };
  context.renderAll = function renderAll() {};
  vm.createContext(context);
  vm.runInContext(extractFunction("round3"), context);
  vm.runInContext(extractFunction("numberValue"), context);
  vm.runInContext(extractFunction("normalizeKeepoutZones"), context);
  vm.runInContext(extractFunction("loadRouteJson"), context);
  vm.runInContext(extractFunction("buildRouteJson"), context);
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

function testRemovedOldInputs() {
  assert(!html.includes("Nav2 " + "参数"));
  assert(!html.includes("禁行区 " + "JSON"));
  assert(!html.includes("download" + "ZonesBtn"));
  assert(!html.includes("function parse" + "Nav2Params"));
  assert(!html.includes("function load" + "ZonesJson"));
  assert(!html.includes("function build" + "ZonesJson"));
}

function testRouteKeepoutZonesRoundTrip() {
  const context = loadRouteJsonContext();
  context.state.zones = [
    {
      id: "keepout_001",
      name: "禁行区1",
      type: "hard_keepout",
      enabled: true,
      polygon: [{ x: 1.2345, y: -2.3456 }],
    },
  ];

  assert.deepStrictEqual(JSON.parse(JSON.stringify(context.buildRouteJson().keepout_zones)), [
    {
      id: "keepout_001",
      name: "禁行区1",
      type: "hard_keepout",
      enabled: true,
      polygon: [{ x: 1.235, y: -2.346 }],
    },
  ]);
  assert.deepStrictEqual(JSON.parse(JSON.stringify(context.buildRouteJson().map)), {
    yaml: "my_map.yaml",
    image: "my_map.pgm",
    resolution: 0.025,
    origin: [-7.07, -13.3, 0],
    width: 395,
    height: 675,
    image_sha256: "a".repeat(64),
  });

  context.loadRouteJson({
    version: 3,
    frame_id: "map",
    map: {
      yaml: "my_map.yaml",
      image: "my_map.pgm",
      resolution: 0.025,
      origin: [-7.07, -13.3, 0],
      width: 395,
      height: 675,
      image_sha256: "a".repeat(64),
    },
    keepout_zones: [
      {
        id: "keepout_002",
        name: "禁区2",
        type: "hard_keepout",
        enabled: false,
        polygon: [{ x: "1.2", y: "-2.2" }],
      },
    ],
    targets: [],
    routes: [{ id: "route_patrol_001", target_ids: [] }],
  });
  assert.deepStrictEqual(JSON.parse(JSON.stringify(context.state.zones)), [
    {
      id: "keepout_002",
      name: "禁区2",
      type: "hard_keepout",
      enabled: false,
      polygon: [{ x: 1.2, y: -2.2 }],
    },
  ]);
  assert.strictEqual(context.state.activeZoneId, "keepout_002");
}

function testOldRouteWithoutKeepoutZones() {
  const context = loadRouteJsonContext();
  context.loadRouteJson({
    version: 2,
    frame_id: "map",
    targets: [],
    routes: [{ id: "route_patrol_001", target_ids: [] }],
  });
  assert.deepStrictEqual(JSON.parse(JSON.stringify(context.state.zones)), []);
  assert.strictEqual(context.state.activeZoneId, null);
}

testBlockOrigin();
testInlineOrigin();
testRemovedOldInputs();
testRouteKeepoutZonesRoundTrip();
testOldRouteWithoutKeepoutZones();
