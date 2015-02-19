'use strict';

var collisions = require('collide-3d-tilemap')
,   extend = require('extend')
,   aabb = require('aabb-3d')
,   vec3 = require('gl-vec3')

var RigidBody = require('./rigidBody')

module.exports = function(game, opts) {
  return new Physics(game, opts)
}

var _game
var defaults = {
  gravity: [0, -18, 0]
  , airFriction: 0.995

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

Physics.prototype.addBody = function(avatar, _aabb) {
  // for backwards compatibility, right new default dims to player size
  _aabb = _aabb || new aabb( [0,22,0], [2/3, 1.5, 2/3] )
  var b = new RigidBody(avatar, _aabb)
  this.bodies.push(b)
  return b
}

Physics.prototype.removeBody = function(b) {
  var i = this.bodies.indexOf(b)
  if (i < 0) return undefined
  this.bodies.splice(i, 1)
}




/*
 *    PHYSICS AND COLLISIONS
*/

var b, i, len
var world_x0 = vec3.create()
,  world_x1 = vec3.create()
,  world_dx = vec3.create()
,  friction = vec3.create()
,  a = vec3.create()
,  g = vec3.create()
,  dv = vec3.create()
,  dx = vec3.create()

Physics.prototype.tick = function(dt) {
  // convert dt to seconds
  dt = dt/1000
  for(i=0, len=this.bodies.length; i<len; ++i) {
    b = this.bodies[i]
    var onGround = (b.resting[1] < 0)
    b.resting = [0,0,0]

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

    // friction
    if (onGround) { // friction force <= - u |vel|
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
      // air resistance
      vec3.scale( b.velocity, b.velocity, this.airFriction )
    }

    // x1-x0 = v1*dt
    vec3.scale( dx, b.velocity, dt )

    // clear forces and impulses for next timestep
    vec3.set( b._forces, 0, 0, 0 )
    vec3.set( b._impulses, 0, 0, 0 )

    // collisions
    this.collideWorld( b.aabb, dx, function hit(axis, tile, coords, dir, edge) {
      if (!tile) return false
      if (Math.abs(dx[axis]) < Math.abs(edge)) {
        return
      }
      dx[axis] = edge
      b.velocity[axis] = 0
      b.resting[axis] = dir
      // TODO: emit collision event (on body?) with impulse amount
      return true
    })

    // the collide function updates b.aabb, so we're done
  }

}




