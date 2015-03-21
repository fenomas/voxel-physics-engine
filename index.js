'use strict';

var collisions = require('collide-3d-tilemap')
,   extend = require('extend')
,   aabb = require('aabb-3d')
,   vec3 = require('gl-vec3')

var RigidBody = require('./rigidBody')

module.exports = function(game, opts) {
  return new Physics(game, opts)
}

var defaults = {
  gravity: [0, -10, 0], 
  airFriction: 0.995,
  minBounceImpulse: .5, // lowest collision impulse that bounces

}


/* 
 *    CONSTRUCTOR - represents a world of rigid bodies.
 * 
 *  Takes in a getBlock(x,y,z) function to query block solidity.
*/
function Physics(opts, getBlock) {
  opts = extend( {}, defaults, opts )

  this.gravity = opts.gravity
  this.airFriction = opts.airFriction
  this.minBounceImpulse = opts.minBounceImpulse
  this.bodies = []

  // collision function - TODO: abstract this into a setter?
  this.collideWorld = collisions(
    getBlock,
    1,
    [Infinity, Infinity, Infinity],
    [-Infinity, -Infinity, -Infinity]
  )
}


/*
 *    ADDING AND REMOVING RIGID BODIES
*/

Physics.prototype.addBody = function(_aabb, mass,
                                      friction, restitution, gravMult,
                                      onCollide) {
  _aabb = _aabb || new aabb( [0,0,0], [1,1,1] )
  if (typeof mass == 'undefined') mass = 1
  if (typeof friction == 'undefined') friction = 1
  if (typeof restitution == 'undefined') restitution = 0
  if (typeof gravMult == 'undefined') gravMult = 1
  var b = new RigidBody(_aabb, mass, friction, restitution, gravMult, onCollide)
  this.bodies.push(b)
  return b
}

Physics.prototype.removeBody = function(b) {
  var i = this.bodies.indexOf(b)
  if (i < 0) return undefined
  this.bodies.splice(i, 1)
  b.aabb = b.onCollide = null // in case it helps the GC
}




/*
 *    PHYSICS AND COLLISIONS
*/

var b, i, j, len
var world_x0 = vec3.create(),
    world_x1 = vec3.create(),
    world_dx = vec3.create(),
    friction = vec3.create(),
    a = vec3.create(),
    g = vec3.create(),
    dv = vec3.create(),
    dx = vec3.create(),
    impacts = vec3.create(),
    tmpBox,
    tmpDx = vec3.create(),
    tmpResting = vec3.create(),
    flag = { // boolean holder to get around scope peculiarities below
      value: false
    }
    



Physics.prototype.tick = function(dt) {
  // convert dt to seconds
  dt = dt/1000
  for(i=0, len=this.bodies.length; i<len; ++i) {
    b = this.bodies[i]

    // semi-implicit Euler integration

    // a = f/m + gravity*gravityMultiplier
    vec3.scale( a, b._forces, 1/b.mass )
    vec3.scale( g, this.gravity, b.gravityMultiplier )
    vec3.add  ( a, a, g )

    // v1 = v0 + i/m + a*dt
    vec3.scale( dv, b._impulses, 1/b.mass )
    vec3.add  ( b.velocity, b.velocity, dv )
    vec3.scale( dv, a, dt )
    vec3.add  ( b.velocity, b.velocity, dv )

    // apply friction if body was on ground last frame
    if (b.resting[1]<0) {
      // friction force <= - u |vel|
      // max friction impulse = (F/m)*dt = (mg)/m*dt = u*g*dt = dt*b.friction
      var fMax = dt * b.friction
      // friction direction - inversed horizontal velocity
      vec3.scale( friction, b.velocity, -1 )
      friction[1] = 0
      var vAmt = vec3.length(friction)
      if (vAmt > fMax) { // slow down
        vec3.scale( friction, friction, fMax/vAmt )
        vec3.add( b.velocity, b.velocity, friction )
      } else { // stop
        b.velocity[0] = b.velocity[2] = 0
      }
    } else {
      // not on ground, apply air resistance
      vec3.scale( b.velocity, b.velocity, this.airFriction )
    }

    // x1-x0 = v1*dt
    vec3.scale( dx, b.velocity, dt )

    // clear forces and impulses for next timestep
    vec3.set( b._forces, 0, 0, 0 )
    vec3.set( b._impulses, 0, 0, 0 )

    // cache stepped base/dx values for autostep
    if (b.autoStep) {
      tmpBox = new aabb( b.aabb.base, b.aabb.vec )
      vec3.copy( tmpDx, dx )
    }

    // run collisions
    vec3.set( b.resting, 0, 0, 0 )
    // flag.value is a check whether the body was collided already before
    // taking the movement vector into account. It's wrapped in an object
    // so we can pass it to and reference it from processHit()
    flag.value = false
    this.collideWorld(b.aabb, dx, 
                      processHit.bind(null, dx, b.resting, flag) )

    // if autostep, and on ground, run collisions again with stepped up aabb
    if (b.autoStep && b.resting[1]<0 && (b.resting[0] || b.resting[2])) {
      vec3.set( tmpResting, 0, 0, 0 )
      var y = tmpBox.base[1]
      if (b.resting[1]<0) tmpDx[1]=0
      tmpBox.translate( [0, Math.floor(y+1.01)-y, 0] )
      this.collideWorld(tmpBox, tmpDx, 
                        processHit.bind(null, tmpDx, tmpResting, flag) )
      var stepx = b.resting[0] && !tmpResting[0]
      var stepz = b.resting[2] && !tmpResting[2]
      // if stepping avoids collisions, copy stepped results into real data
      if (!flag.value && (stepx || stepz)) {
        setBoxPos( b.aabb, tmpBox.base )
        if (b.resting[1]<0) tmpResting[1]=-1
        vec3.copy( b.resting, tmpResting )
        if (b.onStep) b.onStep();
      }
    }

    // Collision impacts. b.resting shows which axes had collisions:
    for (j=0; j<3; ++j) {
      impacts[j] = 0
      if (b.resting[j]) {
        impacts[j] = -b.velocity[j]
        b.velocity[j] = 0
      }
    }
    var mag = vec3.length(impacts)
    if (mag>.001) { // epsilon
      // bounce if over minBounceImpulse
      if (mag>this.minBounceImpulse && b.restitution) {
        vec3.scale(impacts, impacts, b.restitution)
        b.applyImpulse( impacts )
      }
      // collision event regardless
      if (b.onCollide) b.onCollide(impacts);
    }
  }
}

// the on-hit function called by the collide-tilemap library
function processHit(vec, resting, wasCollided, axis, tile, coords, dir, edge) {
  // assume all truthy tile values collide
  if (!tile) return
  if (Math.abs(vec[axis]) < Math.abs(edge)) {
    // true when the body started out already collided with terrain
    wasCollided.value = true
    return
  }
  // a collision happened, process it
  resting[axis] = dir
  vec[axis] = edge
  return true
}

// helper function, since aabb has no easy way of setting position
function setBoxPos(box, pos) {
  vec3.copy( box.base, pos )
  vec3.add( box.max, box.base, box.vec )
}
