// World size
var delta = [0, 0];
var stage = [ window.screenX, window.screenY, window.innerWidth, window.innerHeight ];

// World bounding boxes
var walls = [];
var wall_thickness = 200;
var wallsSet = false;


// Box2D world parameters
var world;
var velocitySolverIterations = 3;
var positionSolverIterations = 2;
var gravity = { x: 0, y: 9.81 };
var timeScale = 1;
var physicsTimestep = 0.02;


// Mouse stuff
var isMouseDown = false;
var mouseJoint;
var mouse = { x: 0, y: 0 };
var mouseOnClick = [];


// Physics bodies
var elements = [];
var bodies = [];
var properties = [];


InitPhysics();

  
function InitPhysics() {
  GetWindowDimensions();
 	 
  document.addEventListener("mousedown", () => isMouseDown = true, false);
  document.addEventListener("mouseup", () => isMouseDown = false, false);
  
  document.addEventListener("mousemove", (event) => {
  	mouse.x = event.clientX;
  	mouse.y = event.clientY;
  }, false);

  // Initialize a new Box2D world the size of the screen
  var worldAABB = new b2AABB();
  worldAABB.minVertex.Set(-200, -200);
  worldAABB.maxVertex.Set(window.innerWidth + 200, window.innerHeight + 200);
  
  var contentKeywords = [
    'gravity',
    'timestep',
    'timescale'
  ];
  
  var content = GetMetaContent('physics', contentKeywords);
  
  for (let i = 0; i < contentKeywords.length; i++) {
    if (content[i] == null) continue;

   	if (contentKeywords[i] == 'gravity') {
      gravity.y = parseFloat(content[i]);
    }
    
    if (contentKeywords[i] == 'timescale') {
      timeScale = parseFloat(content[i]);
    }
    
    if (contentKeywords[i] == 'timestep') {
      physicsTimestep = parseFloat(content[i]);
    }
  }
  
  world = new b2World(worldAABB, new b2Vec2(0, 0), true);
  
  SetBounds();
  
  
  // Get all HTML physics elements and any additional values
  var values = GetPhysicsElements();
  elements = values[0];
  
  var densities = values[1];
  var restitutions = values[2];
  var frictions = values[3];
  var statics = values[4];
  
  // Lay-out all physics elements first
  for (var i = 0; i < elements.length; i++) {
  	properties[i] = GetElementProperties(elements[i]);
  }
  
  // Create element's physics box and assign additional properties
  for (var i = 0; i < elements.length; i++) {
    var element = elements[i];
    
    element.style.position = "absolute";
    element.style.left = properties[i][0] + "px";
    element.style.top = properties[i][1] + "px";
    element.style.width = properties[i][2] + "px";
    
    element.addEventListener("mousedown", OnMouseDown, false);
    element.addEventListener("mouseup", (event) => event.preventDefault(), false);
    element.addEventListener("click", OnClick, false);

    bodies[i] = CreateBox(
      world,
      properties[i][0] + (properties[i][2] >> 1),
      properties[i][1] + (properties[i][3] >> 1),
      properties[i][2] / 2,
      properties[i][3] / 2,
      statics[i],
      densities[i],
      restitutions[i],
      frictions[i]
    );
   
    // Clean CSS position dependencies to give full position control to physics system
    while (element.offsetParent) {
      element = element.offsetParent;
      element.style.position = "static";
    }
  }
  
  // Invoke FixedUpdate by the timestep converted to milliseconds
  setInterval(FixedUpdate, physicsTimestep * 1000);
}

function OnMouseDown(event) {
  event.preventDefault();

  mouseOnClick[0] = event.clientX;
  mouseOnClick[1] = event.clientY;
}

function OnClick(event) {
  var range = 5;

  if (mouseOnClick[0] > event.clientX + range || (mouseOnClick[0] < event.clientX - range && mouseOnClick[1] > event.clientY + range) || mouseOnClick[1] < event.clientY - range) {
    event.preventDefault();
  }
  
  if (event.target == document.getElementById("q")) {
    document.getElementById("q").focus();
  }
}

// Compute and update all physics bodies + visuals
function FixedUpdate() {
  // Update walls if the window has resized
  if (GetWindowDimensions()) SetBounds();
	
  world.m_gravity.x = 0;
  // NORMALLY you don't have to premultiply gravity, but either the physics is janky or I'm not using fixed timestep correctly
  world.m_gravity.y = gravity.y * 50;

  UpdateMouse();
  
  // Step the physics simulation forward
  world.Step(physicsTimestep * timeScale, velocitySolverIterations, positionSolverIterations);
	
  // Update element visuals with corresponding body position/rotations
  for (i = 0; i < elements.length; i++) {
    var body = bodies[i];
    var element = elements[i];
 
    element.style.left = body.m_position0.x - (properties[i][2] >> 1) + "px";
    element.style.top = body.m_position0.y - (properties[i][3] >> 1) + "px";
    var style = "rotate(" + body.m_rotation0 * 57.2957795 + "deg)";

    element.style.transform = style;
    element.style.WebkitTransform = style + " translateZ(0)";
    element.style.MozTransform = style;
    element.style.OTransform = style;
    element.style.msTransform = style;
  }
}


function CreateBox(world, x, y, width, height, static, density, restitution, friction) {
  if (typeof static == "undefined") static = true;

  var boxSd = new b2BoxDef();

  if (!static) {
    // Kinda like mass
    boxSd.density = density;
    // Kinda like bounciness
    boxSd.restitution = restitution;
    boxSd.friction = friction;
  }
  
  boxSd.extents.Set(width, height);

  var boxBd = new b2BodyDef();
  boxBd.AddShape(boxSd);
  boxBd.position.Set(x, y);

  return world.CreateBody(boxBd);
}


function UpdateMouse() {
  // Mouse press
  if (isMouseDown && !mouseJoint) {
    var body = GetBodyAtMouse();

    if (body) {
      var md = new b2MouseJointDef();
      md.body1 = world.m_groundBody;
      md.body2 = body;
      md.target.Set(mouse.x, mouse.y);
      md.maxForce = 30000.0 * body.m_mass;
      mouseJoint = world.CreateJoint(md);
      body.WakeUp();
    }
  }

  // Mouse release
  if (!isMouseDown) {
    if (mouseJoint) {
      world.DestroyJoint(mouseJoint);
      mouseJoint = null;
    }
  }

  // Mouse move
  if (mouseJoint) {
    var p2 = new b2Vec2(mouse.x, mouse.y);
    mouseJoint.SetTarget(p2);
  }
}


function GetBodyAtMouse() {
  // Make a small box.
  var mousePVec = new b2Vec2();
  mousePVec.Set(mouse.x, mouse.y);

  var aabb = new b2AABB();
  aabb.minVertex.Set(mouse.x - 1, mouse.y - 1);
  aabb.maxVertex.Set(mouse.x + 1, mouse.y + 1);

  // Query the world for overlapping bodies.
  var k_maxCount = 10;
  var shapes = [];
  var count = world.Query(aabb, shapes, k_maxCount);
  var body = null;
  
  // Get the first non-static overlapping body
  for (var i = 0; i < count; i++) {
    if (shapes[i].m_body.IsStatic() == false) {
      if (shapes[i].TestPoint(mousePVec)) {
        body = shapes[i].m_body;
        break;
      }
    }
  }

  return body;
}


function SetBounds() {
  if (wallsSet) {
    world.DestroyBody(walls[0]);
    world.DestroyBody(walls[1]);
    world.DestroyBody(walls[2]);
    world.DestroyBody(walls[3]);

    walls[0] = null;
    walls[1] = null;
    walls[2] = null;
    walls[3] = null;
  }

  walls[0] = CreateBox(
    world,
    stage[2] / 2,
    -wall_thickness,
    stage[2],
    wall_thickness,
  );
  
  walls[1] = CreateBox(
    world,
    stage[2] / 2,
    stage[3] + wall_thickness,
    stage[2],
    wall_thickness,
  );
  
  walls[2] = CreateBox(
    world,
    -wall_thickness,
    stage[3] / 2,
    wall_thickness,
    stage[3],
  );
  
  walls[3] = CreateBox(
    world,
    stage[2] + wall_thickness,
    stage[3] / 2,
    wall_thickness,
    stage[3],
  );

  wallsSetted = true;
}


function GetPhysicsElements() {
  var physicsElements = [];
  var densities = [];
  var restitutions = [];
  var frictions = [];
  var statics = [];
  
  var els = document.getElementsByTagName("*");

  for (i = 0, j = 0; i < els.length; i++) {
    // Check for physics attribute in element
    if (els[i].hasAttribute("physics")) {
      var attributeValues = els[i].getAttribute("physics").split(',');
      
      physicsElements[j++] = els[i];
        
      densities[j-1] = 1.0;
      restitutions[j-1] = 0.3;
      frictions[j-1] = 1.0;
      statics[j-1] = false;
      
      // Parse additional values
      for (k = 0; k < attributeValues.length; k++) {
        var val = parseFloat(GetParamValues(attributeValues[k], "density="));
        
        if (val >= 0) {
          densities[j-1] = val;
        }
        
        val = parseFloat(GetParamValues(attributeValues[k], "restitution="));
        
        if (val >= 0) {
          restitutions[j-1] = val;
        }
        
        val = parseFloat(GetParamValues(attributeValues[k], "friction="));
        
        if (val >= 0) {
          frictions[j-1] = val;
        }
        
        val = GetParamValues(attributeValues[k], "static=");
        
        if (val == "true") {
          statics[j-1] = true;
        } else if (val == "false") {
          statics[j-1] = false;
        }
      }
    }
  }
  
  console.log(`Found ${physicsElements.length} physics elements`);
  
  var values = [
    physicsElements,
    densities,
    restitutions,
    frictions,
    statics
  ];
  
  return values;
}

// Parses the values of an entire parameter (ex: myParam = 100, returns the '100' part)
function GetParamValues(param, paramName) {
  param = param.replaceAll(' ', '');
  paramName = paramName.replaceAll(' ', '');
  
  if (param.substring(0, paramName.length) == paramName) {
    var parsed = param.substring(paramName.length, param.length).trim();
    return parsed;
  }
  
  return param;
}


function GetElementProperties(element) {
  var x = 0;
  var y = 0;
  var width = element.offsetWidth;
  var height = element.offsetHeight;

  do {
    x += element.offsetLeft;
    y += element.offsetTop;
  } while ((element = element.offsetParent));

  return [x, y, width, height];
}

function GetMetaContent(metaName, attributeNames) {
  const metas = document.getElementsByTagName('meta');

  var content = [];

  for (let i = 0; i < metas.length; i++) {
    if (metas[i].getAttribute('name') === metaName) {
      for (let j = 0; j < attributeNames.length; j++) {
      	content[j] = metas[i].getAttribute(attributeNames[j]);
      }
    }
  }

  return content;
}


function GetWindowDimensions() {
  var changed = false;

  if (stage[0] != window.screenX) {
    delta[0] = (window.screenX - stage[0]) * 50;
    stage[0] = window.screenX;
    changed = true;
  }

  if (stage[1] != window.screenY) {
    delta[1] = (window.screenY - stage[1]) * 50;
    stage[1] = window.screenY;
    changed = true;
  }

  if (stage[2] != window.innerWidth) {
    stage[2] = window.innerWidth;
    changed = true;
  }

  if (stage[3] != window.innerHeight) {
    stage[3] = window.innerHeight;
    changed = true;
  }

  return changed;
}