// fix cam z position control with dat-gui
// fix remove boid function (actually implement it !)
// remove more boids at a time
// detect orientation of each boid



var flock;

var fov = 0;
var cameraZ = 0;

function setup() {
  createCanvas(windowWidth, windowHeight,WEBGL);

  flock = new Flock();
  // Add an initial set of boids into the system
  for (var i = 0; i < 10; i++) {
    var b = new Boid(1,1,250);
    flock.addBoid(b);
  }

  sp = new simulationParameters();
  cc = new camControls();
  gui = new dat.GUI();
  initGui();


  var v1 = createVector(1, 0, 0);
  var v2 = createVector(0, 1, 0);
 
  var angle = p5.Vector.angleBetween(v1, v2);
  console.log(angle);
}

function draw() {
  background(0);
  ambientLight(255, 255, 255);

  translate(0,0,cc.z_position);

  rotateY(cc.x_rotation);
  rotateX(cc.y_rotation);

  specularMaterial(250);
  push();
  translate(-width/2,-height/2,0);
  box(5,5,5);
  translate(0,0,-800);
  box(5,5,5);
  translate(width,0,0);
  box(5,5,5);
  translate(0,0,800);
  box(5,5,5);
  pop();
  push();
  translate(-width/2,0,0);
  box(5,5,5);
  translate(0,0,-800);
  box(5,5,5);
  translate(width,0,0);
  box(5,5,5);
  translate(0,0,800);
  box(5,5,5);
  pop();
  push();
  translate(-width/2,height/2,0);
  box(5,5,5);
  translate(0,0,-800);
  box(5,5,5);
  translate(width,0,0);
  box(5,5,5);
  translate(0,0,800);
  box(5,5,5);
  pop();

  push();
  normalMaterial();
  flock.run();
  pop();
}




// The Nature of Code
// Daniel Shiffman
// http://natureofcode.com
// adapted for 3D rendering.
// Flock object
// Does very little, simply manages the array of all the boids

function Flock() {
  // An array for all the boids
  this.boids = []; // Initialize the array
}

Flock.prototype.run = function() {
  for (var i = 0; i < this.boids.length; i++) {
    this.boids[i].run(this.boids);  // Passing the entire list of boids to each boid individually
  }
}

Flock.prototype.addBoid = function(b) {
  this.boids.push(b);
}

// The Nature of Code
// Daniel Shiffman
// http://natureofcode.com
// adapted for 3D
// Boid class
// Methods for Separation, Cohesion, Alignment added

function Boid(x,y,z) {
  this.acceleration = createVector(0,0,0);
  this.velocity = createVector(random(-1,1),random(-1,1),random(-1,1));
  this.position = createVector(x,y,z);
  this.r = 3.0;
  this.maxspeed = 3;    // Maximum speed
  this.maxforce = 0.05; // Maximum steering force
 
  this.globsteer;
}

Boid.prototype.run = function(boids) {
  this.flock(boids);
  this.update();
  this.borders();
  this.render(boids);
}

Boid.prototype.applyForce = function(force) {
  // We could add mass here if we want A = F / M
  this.acceleration.add(force);
}

// We accumulate a new acceleration each time based on three rules
Boid.prototype.flock = function(boids) {
  var sep = this.separate(boids);   // Separation
  var ali = this.align(boids);      // Alignment
  var coh = this.cohesion(boids);   // Cohesion
  // Arbitrarily weight these forces
  sep.mult(sp.separation);
  ali.mult(sp.alignement);
  coh.mult(sp.cohesion);

  //this.globsteer = sep.mult(sp.separation)+  ali.mult(sp.alignement)+  coh.mult(sp.cohesion);
  //console.log(this.globsteer);
  // Add the force vectors to acceleration
  this.applyForce(sep);
  this.applyForce(ali);
  this.applyForce(coh);
}

// Method to update location
Boid.prototype.update = function() {
  // Update velocity
  this.velocity.add(this.acceleration);
  // Limit speed
  this.velocity.limit(this.maxspeed);
  this.position.add(this.velocity);
  // Reset accelertion to 0 each cycle
  this.acceleration.mult(0);
}

// A method that calculates and applies a steering force towards a target
// STEER = DESIRED MINUS VELOCITY
Boid.prototype.seek = function(target) {
  var desired = p5.Vector.sub(target,this.position);  // A vector pointing from the location to the target
  // Normalize desired and scale to maximum speed
  desired.normalize();
  desired.mult(this.maxspeed);
  // Steering = Desired minus Velocity
  var steer = p5.Vector.sub(desired,this.velocity);
  steer.limit(this.maxforce);  // Limit to maximum steering force
  return steer;
}

Boid.prototype.render = function(boids) {
  // Draw a triangle rotated in the direction of velocity

  fill(127);
  stroke(200);
  push();
  

  translate(this.position.x,this.position.y,this.position.z);

  rotateY(atan2(-this.velocity.z,this.velocity.x));
  rotateZ(asin(this.velocity.y/this.velocity.mag()));

  cone(this.r*3, this.r*5);
  pop();
}

// Wraparound
Boid.prototype.borders = function() {
  if (this.position.x < -this.r-width/2)  this.position.x = width/2 +this.r;
  if (this.position.y < -this.r-height/2)  this.position.y = height/2+this.r;
  if (this.position.x > width/2 +this.r) this.position.x = -width/2-this.r;
  if (this.position.y > height/2+this.r) this.position.y = -height/2-this.r;
  if (this.position.z > 0+this.r) this.position.z = -this.r -800;
  if (this.position.z < -800+this.r) this.position.z = -this.r +0;

 
}

// Separation
// Method checks for nearby boids and steers away
Boid.prototype.separate = function(boids) {
  var desiredseparation = 25.0;
  var steer = createVector(0,0,0);
  var count = 0;
  // For every boid in the system, check if it's too close
  for (var i = 0; i < boids.length; i++) {
    var d = p5.Vector.dist(this.position,boids[i].position);
    // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
    if ((d > 0) && (d < desiredseparation)) {
      // Calculate vector pointing away from neighbor
      var diff = p5.Vector.sub(this.position,boids[i].position);
      diff.normalize();
      diff.div(d);        // Weight by distance
      steer.add(diff);
      count++;            // Keep track of how many
    }
  }
  // Average -- divide by how many
  if (count > 0) {
    steer.div(count);
  }

  // As long as the vector is greater than 0
  if (steer.mag() > 0) {
    // Implement Reynolds: Steering = Desired - Velocity
    steer.normalize();
    steer.mult(this.maxspeed);
    steer.sub(this.velocity);
    steer.limit(this.maxforce);
  }
  return steer;
}

// Alignment
// For every nearby boid in the system, calculate the average velocity
Boid.prototype.align = function(boids) {
  var neighbordist = 50;
  var sum = createVector(0,0,0);
  var count = 0;
  for (var i = 0; i < boids.length; i++) {
    var d = p5.Vector.dist(this.position,boids[i].position);
    if ((d > 0) && (d < neighbordist)) {
      sum.add(boids[i].velocity);
      count++;
    }
  }
  if (count > 0) {
    sum.div(count);
    sum.normalize();
    sum.mult(this.maxspeed);
    var steer = p5.Vector.sub(sum,this.velocity);
    steer.limit(this.maxforce);
    return steer;
  } else {
    return createVector(0,0,0);
  }
}

// Cohesion
// For the average location (i.e. center) of all nearby boids, calculate steering vector towards that location
Boid.prototype.cohesion = function(boids) {
  var neighbordist = 50;
  var sum = createVector(0,0,0);   // Start with empty vector to accumulate all locations
  var count = 0;
  for (var i = 0; i < boids.length; i++) {
    var d = p5.Vector.dist(this.position,boids[i].position);
    if ((d > 0) && (d < neighbordist)) {
      sum.add(boids[i].position); // Add location
      count++;
    }
  }
  if (count > 0) {
    sum.div(count);
    return this.seek(sum);  // Steer towards the location
  } else {
    return createVector(0,0,0);
  }
}


////////////////////////////////////////////////////////////////////////
// GUI
var initGui = function() {
  var f2 = gui.addFolder('Simulation parameters');
  f2.add(sp, 'separation' ,-10,10);
  f2.add(sp, 'alignement' ,-10,10);
  f2.add(sp, 'cohesion' ,-10,10);  
  f2.add(sp, 'add_boid');
  f2.add(sp, 'remove_boid');  

  var f = gui.addFolder('Camera controls');
  f.add(cc, 'z_position', -800,800);
  f.add(cc, 'x_rotation', -PI/2,PI/2);
  f.add(cc, 'y_rotation', -PI/2,PI/2);
  f.add(cc, 'reset_camera');
}

var simulationParameters = function(){
    this.separation = 1.5;
    this.alignement = 1.0;
    this.cohesion = 1.0;

    this.add_boid = function (){flock.addBoid(new Boid(random(windowWidth),random(windowHeight)))}
    this.remove_boid = function (){flock.removeBoid();}
}

var camControls = function(){
  this.z_position = 0;
  this.x_rotation = 0;
  this.y_rotation = 0;

  this.reset_camera = function (){
     this.z_position = 0;
     this.x_rotation = 0;
     this.y_rotation = 0;
  }
}


function makePerspective(fieldOfViewInRadians, aspect, near, far) {
  var f = Math.tan(Math.PI * 0.5 - 0.5 * fieldOfViewInRadians);
  var rangeInv = 1.0 / (near - far);
  return [
    f / aspect, 0, 0, 0,
    0, f, 0, 0,
    0, 0, (near + far) * rangeInv, -1,
    0, 0, near * far * rangeInv * 2, 0
  ];
};

function makeTranslation(tx, ty, tz) {
  return [
     1,  0,  0,  0,
     0,  1,  0,  0,
     0,  0,  1,  0,
    tx, ty, tz,  1
  ];
}
function makeXRotation(angleInRadians) {
  var c = Math.cos(angleInRadians);
  var s = Math.sin(angleInRadians);
  return [
    1, 0, 0, 0,
    0, c, s, 0,
    0, -s, c, 0,
    0, 0, 0, 1
  ];
};
function makeYRotation(angleInRadians) {
  var c = Math.cos(angleInRadians);
  var s = Math.sin(angleInRadians);
  return [
    c, 0, -s, 0,
    0, 1, 0, 0,
    s, 0, c, 0,
    0, 0, 0, 1
  ];
};
function makeZRotation(angleInRadians) {
  var c = Math.cos(angleInRadians);
  var s = Math.sin(angleInRadians);
  return [
     c, s, 0, 0,
    -s, c, 0, 0,
     0, 0, 1, 0,
     0, 0, 0, 1,
  ];
}



function matrixMultiply(a, b) {
  var a00 = a[0*4+0];
  var a01 = a[0*4+1];
  var a02 = a[0*4+2];
  var a03 = a[0*4+3];
  var a10 = a[1*4+0];
  var a11 = a[1*4+1];
  var a12 = a[1*4+2];
  var a13 = a[1*4+3];
  var a20 = a[2*4+0];
  var a21 = a[2*4+1];
  var a22 = a[2*4+2];
  var a23 = a[2*4+3];
  var a30 = a[3*4+0];
  var a31 = a[3*4+1];
  var a32 = a[3*4+2];
  var a33 = a[3*4+3];
  var b00 = b[0*4+0];
  var b01 = b[0*4+1];
  var b02 = b[0*4+2];
  var b03 = b[0*4+3];
  var b10 = b[1*4+0];
  var b11 = b[1*4+1];
  var b12 = b[1*4+2];
  var b13 = b[1*4+3];
  var b20 = b[2*4+0];
  var b21 = b[2*4+1];
  var b22 = b[2*4+2];
  var b23 = b[2*4+3];
  var b30 = b[3*4+0];
  var b31 = b[3*4+1];
  var b32 = b[3*4+2];
  var b33 = b[3*4+3];
  return [a00 * b00 + a01 * b10 + a02 * b20 + a03 * b30,
          a00 * b01 + a01 * b11 + a02 * b21 + a03 * b31,
          a00 * b02 + a01 * b12 + a02 * b22 + a03 * b32,
          a00 * b03 + a01 * b13 + a02 * b23 + a03 * b33,
          a10 * b00 + a11 * b10 + a12 * b20 + a13 * b30,
          a10 * b01 + a11 * b11 + a12 * b21 + a13 * b31,
          a10 * b02 + a11 * b12 + a12 * b22 + a13 * b32,
          a10 * b03 + a11 * b13 + a12 * b23 + a13 * b33,
          a20 * b00 + a21 * b10 + a22 * b20 + a23 * b30,
          a20 * b01 + a21 * b11 + a22 * b21 + a23 * b31,
          a20 * b02 + a21 * b12 + a22 * b22 + a23 * b32,
          a20 * b03 + a21 * b13 + a22 * b23 + a23 * b33,
          a30 * b00 + a31 * b10 + a32 * b20 + a33 * b30,
          a30 * b01 + a31 * b11 + a32 * b21 + a33 * b31,
          a30 * b02 + a31 * b12 + a32 * b22 + a33 * b32,
          a30 * b03 + a31 * b13 + a32 * b23 + a33 * b33];
}