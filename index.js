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
  gravity: [0, -9.8 , 0]
  , friction: 0

}


/* 
 *    CONSTRUCTOR - represents a world of rigid bodies.
*/
function Physics(game, opts) {
  _game = game
  opts = extend( {}, defaults, opts )

  this.gravity = opts.gravity
  this.bodies = []

  // collision function - TODO: abstract this into a setter?
  this.collideWorld = collisions(
    _game.getBlock.bind(_game),
    1,
    [Infinity, Infinity, Infinity],
    [-Infinity, -Infinity, -Infinity]
  )
}



/*
 *    ADDING AND REMOVING RIGID BODIES
*/

Physics.prototype.addBody = function(avatar, dimensions) {
  // for backwards compatibility, right new default dims to player size
  dimensions = dimensions || [ 2/3, 1.5, 2/3 ]
  var b = new RigidBody(avatar, dimensions)
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
,  temp = vec3.create()
,  a = vec3.create()
,  dv = vec3.create()
,  dx = vec3.create()

Physics.prototype.tick = function(dt) {
  // convert dt to seconds
  dt = dt/1000
  for(i=0, len=this.bodies.length; i<len; ++i) {
    b = this.bodies[i]
    b.resting = [0,0,0]
    var onGround = (b.resting[1] < 0)

    // semi-implicit Euler integration
    
    // a = f/m + gravity
    vec3.scale( a, b._forces, 1/b._mass )
    vec3.add  ( a, a, this.gravity )
    
    // v1 = v0 + i/m + a*dt
    vec3.scale( dv, b._impulses, 1/b._mass )
    vec3.add  ( b.velocity, b.velocity, dv )
    vec3.scale( dv, a, dt )
    vec3.add  ( b.velocity, b.velocity, dv )
    
    // pseudo-physical friction for now - v1 *= drag
    var drag = (onGround) ? 0.8 : 0.98
    vec3.scale( b.velocity, b.velocity, drag )
    
    // x1-x0 = v1*dt
    vec3.scale( dx, b.velocity, dt )
    
    // clear forces and impulses for next timestep
    vec3.set( b._forces, 0, 0, 0 )
    vec3.set( b._impulses, 0, 0, 0 )

    // change dx into world coords - TODO: this will need to be undone
    vecSetXYZ(world_x0, b.avatar.position)

    b.avatar.translateX( dx[0] )
    b.avatar.translateY( dx[1] )
    b.avatar.translateZ( dx[2] )
    vecSetXYZ(world_x1, b.avatar.position)
    vec3.subtract(world_dx, world_x1, world_x0)

    // collisions
    var bb = b.aabb()

    this.collideWorld( bb, world_dx, function hit(axis, tile, coords, dir, edge) {
      if (!tile) return false
      if (Math.abs(world_dx[axis]) < Math.abs(edge)) {
        return
      }
      world_dx[axis] = edge
      b.velocity[axis] = 0
      b.resting[axis] = dir
      return true
    })

    b.avatar.position.x = world_x0[0]
    b.avatar.position.y = world_x0[1]
    b.avatar.position.z = world_x0[2]

    b.avatar.position.x += world_dx[0]
    b.avatar.position.y += world_dx[1]
    b.avatar.position.z += world_dx[2]

//    _game.pin({y:b.avatar.position.y})
  }

}

// remove this once we no longer use {x,y,z} vectors
function vecSetXYZ(vec,obj) {
  vec[0] = obj.x
  vec[1] = obj.y
  vec[2] = obj.z
}
















