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
  var b = this.bodies.splice(i, 1)[0]
  b.aabb = b.onCollide = null // in case it helps the GC
}




/*
 *    PHYSICS AND COLLISIONS
*/

var b, i, j, len, wasResting
var world_x0 = vec3.create()
,  world_x1 = vec3.create()
,  world_dx = vec3.create()
,  friction = vec3.create()
,  a = vec3.create()
,  g = vec3.create()
,  dv = vec3.create()
,  dx = vec3.create()
,  impacts = vec3.create()

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

    // remember resting and velocity values to detect impacts
    wasResting = b.resting.slice()
    vec3.copy( impacts, b.velocity )

    // collisions
    b.resting = [0,0,0]
    this.collideWorld( b.aabb, dx, processHit)
    // the collide function updates b.aabb

    // detect collision impact based on change in velocity
    vec3.subtract(impacts, b.velocity, impacts)
    for (j=0; j<3; ++j) if (wasResting[j]!==0) { impacts[j] = 0 }
    var mag = vec3.length(impacts)
    if (mag>this.minBounceImpulse && b.restitution) {
      vec3.scale(impacts, impacts, b.restitution)
      b.applyImpulse( impacts )
      if (b.onCollide) {
        b.onCollide(impacts)
      }
    }
  }
}


// this function rather confusingly refers to local vars (dx, b)
// in the context it's called from.. would be great to re-architect this
function processHit(axis, tile, coords, dir, edge) {
  // assume all truthy tile values collide
  if (!tile) return false
  if (Math.abs(dx[axis]) < Math.abs(edge)) {
    // this check is true when the body started out collided with terrain
    // ignore for now. TODO: try to move out of collision?
    return
  }
  // a collision happened, process it
  dx[axis] = edge
  b.velocity[axis] = 0
  b.resting[axis] = dir
  // TODO: emit collision event (on body?) with impulse amount
  return true
}



