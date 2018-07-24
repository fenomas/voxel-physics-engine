'use strict'

var aabb = require('aabb-3d')
var vec3 = require('gl-vec3')


var DEBUG = 0


module.exports = RigidBody



/*
 *    RIGID BODY - internal data structure
 *  Only AABB bodies right now. Someday will likely need spheres?
*/

function RigidBody(_aabb, mass, friction, restitution, gravMult, onCollide, autoStep) {
    this.aabb = new aabb(_aabb.base, _aabb.vec) // clone
    this.mass = mass
    this.friction = friction
    this.restitution = restitution
    this.gravityMultiplier = gravMult
    this.onCollide = onCollide
    this.autoStep = !!autoStep
    this.airDrag = -1   // overrides global airDrag when >= 0
    this.fluidDrag = -1 // overrides global fluidDrag when >= 0
    this.onStep = null
    
    // internals
    this.velocity = vec3.create()
    this.resting = [0, 0, 0]
    this.inFluid = false
    this._ratioInFluid = 0
    this._forces = vec3.create()
    this._impulses = vec3.create()
    this._sleepFrameCount = 10 | 0
}

RigidBody.prototype.setPosition = function (p) {
    sanityCheck(p)
    vec3.subtract(p, p, this.aabb.base)
    this.aabb.translate(p)
    this._markActive()
}
RigidBody.prototype.getPosition = function () {
    return vec3.clone(this.aabb.base)
}
RigidBody.prototype.applyForce = function (f) {
    sanityCheck(f)
    vec3.add(this._forces, this._forces, f)
    this._markActive()
}
RigidBody.prototype.applyImpulse = function (i) {
    sanityCheck(i)
    vec3.add(this._impulses, this._impulses, i)
    this._markActive()
}
RigidBody.prototype._markActive = function () {
    this._sleepFrameCount = 10 | 0
}



// temp
RigidBody.prototype.atRestX = function () { return this.resting[0] }
RigidBody.prototype.atRestY = function () { return this.resting[1] }
RigidBody.prototype.atRestZ = function () { return this.resting[2] }





var sanityCheck = function () { }
if (DEBUG) sanityCheck = function (v) {
    if (isNaN(vec3.length(v))) throw 'Vector with NAN: ', v
}
