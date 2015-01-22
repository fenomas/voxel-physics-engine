
var aabb = require('aabb-3d')
,   vec3 = require('gl-vec3')


module.exports = RigidBody
  

/*
 *    RIGID BODY - internal data structure
 *
 * Only AABB bodies right now. Someday will likely need spheres?
*/


function RigidBody(avatar, dimensions) {
  this.avatar = avatar
  this.dimensions = dimensions
  this.velocity = vec3.create()
  this.resting = [ false, false, false ]
  this.uFriction = .1
  this.restitution = .5
  // internals
  this._forces = vec3.create()
  this._impulses = vec3.create()
  this._mass = 1
  // temp?
  this.rotation = { x:0, y:0, z:0 }
}

RigidBody.prototype.aabb = function() {
  // from: https://github.com/chrisdickinson/voxel-physical/blob/master/index.js
  var pos = this.avatar.position
  var d = this.dimensions
  return aabb(
    [pos.x - (d[0]/2), pos.y, pos.z - (d[2]/2)],
    this.dimensions
  )
}

RigidBody.prototype.applyForce = function(f) {
  vec3.add( this._forces, this._forces, f )
}
RigidBody.prototype.applyImpulse = function(i) {
  vec3.add( this._impulses, this._impulses, i )
}


// temp
RigidBody.prototype.subjectTo = function() { /* NOP */ }
RigidBody.prototype.tick = function() { /* NOP */ }
RigidBody.prototype.atRestX = function() { return this.resting[0] }
RigidBody.prototype.atRestY = function() { return this.resting[1] }
RigidBody.prototype.atRestZ = function() { return this.resting[2] }

