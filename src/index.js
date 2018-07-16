'use strict'

var aabb = require('aabb-3d')
var vec3 = require('gl-vec3')
var sweep = require('voxel-aabb-sweep')
var RigidBody = require('./rigidBody')


module.exports = function (opts, testSolid, testFluid) {
    return new Physics(opts, testSolid, testFluid)
}

var defaults = {
    gravity: [0, -10, 0],
    airFriction: 0.9,
    minBounceImpulse: .5, // lowest collision impulse that bounces
    fluidDensity: 1.2,
    fluidDrag: 2.0,
}


/* 
 *    CONSTRUCTOR - represents a world of rigid bodies.
 * 
 *  Takes testSolid(x,y,z) function to query block solidity
 *  Takes testFluid(x,y,z) function to query if a block is a fluid
*/
function Physics(opts, testSolid, testFluid) {
    opts = Object.assign({}, defaults, opts)

    this.gravity = opts.gravity
    this.airFriction = opts.airFriction
    this.fluidDensity = opts.fluidDensity
    this.fluidDrag = opts.fluidDrag
    this.minBounceImpulse = opts.minBounceImpulse
    this.bodies = []

    // collision function - TODO: abstract this into a setter?
    this.testSolid = testSolid
    this.testFluid = testFluid
}


/*
 *    ADDING AND REMOVING RIGID BODIES
*/

Physics.prototype.addBody = function (_aabb, mass, friction,
    restitution, gravMult, onCollide) {
    _aabb = _aabb || new aabb([0, 0, 0], [1, 1, 1])
    if (typeof mass == 'undefined') mass = 1
    if (typeof friction == 'undefined') friction = 1
    if (typeof restitution == 'undefined') restitution = 0
    if (typeof gravMult == 'undefined') gravMult = 1
    var b = new RigidBody(_aabb, mass, friction, restitution, gravMult, onCollide)
    this.bodies.push(b)
    return b
}

Physics.prototype.removeBody = function (b) {
    var i = this.bodies.indexOf(b)
    if (i < 0) return undefined
    this.bodies.splice(i, 1)
    b.aabb = b.onCollide = null
}




/*
 *    PHYSICS AND COLLISIONS
*/

var friction = vec3.create()
var a = vec3.create()
var g = vec3.create()
var dv = vec3.create()
var dx = vec3.create()
var impacts = vec3.create()
var oldResting = vec3.create()


/*
 *    TICK HANDLER
*/
Physics.prototype.tick = function (dt) {
    // convert dt to seconds
    dt = dt / 1000
    var noGravity = equals(0, vec3.squaredLength(this.gravity))

    this.bodies.forEach(b => iterateBody(this, b, dt, noGravity))
}



/*
 *    PER-BODY MAIN PHYSICS ROUTINE
*/

function iterateBody(self, b, dt, noGravity) {
    vec3.copy(oldResting, b.resting)

    // skip bodies with no velocity/forces/impulses
    var localNoGrav = noGravity || (b.gravityMultiplier === 0)
    if (bodyAsleep(self, b, dt, localNoGrav)) return
    b._sleepFrameCount--

    // semi-implicit Euler integration

    // a = f/m + gravity*gravityMultiplier
    vec3.scale(a, b._forces, 1 / b.mass)
    vec3.scaleAndAdd(a, a, self.gravity, b.gravityMultiplier)

    // dv = i/m + a*dt
    // v1 = v0 + dv
    vec3.scale(dv, b._impulses, 1 / b.mass)
    vec3.scaleAndAdd(dv, dv, a, dt)
    vec3.add(b.velocity, b.velocity, dv)

    // apply friction based on change in velocity this frame
    if (b.friction) {
        applyFrictionByAxis(0, b, dv)
        applyFrictionByAxis(1, b, dv)
        applyFrictionByAxis(2, b, dv)
    }

    // linear air friction - effectively v *= drag
    // body airFriction overrides global airFriction
    var drag = b.airFriction || self.airFriction
    var dmult = 1 - (1 - drag) * dt / b.mass
    vec3.scale(b.velocity, b.velocity, dmult)

    // x1-x0 = v1*dt
    vec3.scale(dx, b.velocity, dt)

    // clear forces and impulses for next timestep
    vec3.set(b._forces, 0, 0, 0)
    vec3.set(b._impulses, 0, 0, 0)

    // cache old position for use in autostepping
    if (b.autoStep) {
        cloneAABB(tmpBox, b.aabb)
    }

    // sweeps aabb along dx and accounts for collisions
    processCollisions(self, b.aabb, dx, b.resting)

    // if autostep, and on ground, run collisions again with stepped up aabb
    if (b.autoStep) {
        tryAutoStepping(self, b, tmpBox, dx)
    }

    // Collision impacts. b.resting shows which axes had collisions:
    for (var i = 0; i < 3; ++i) {
        impacts[i] = 0
        if (b.resting[i]) {
            // count impact only if wasn't collided last frame
            if (!oldResting[i]) impacts[i] = -b.velocity[i]
            b.velocity[i] = 0
        }
    }
    var mag = vec3.length(impacts)
    if (mag > .001) { // epsilon
        // bounce if over minBounceImpulse
        if (mag > self.minBounceImpulse && b.restitution) {
            vec3.scale(impacts, impacts, b.restitution * b.mass)
            b.applyImpulse(impacts)
        }
        // send collision event regardless
        if (b.onCollide) b.onCollide(impacts)
    }

    // First pass at handling fluids. Assumes fluids are settled
    //   thus, only check at center of body, and only from bottom up
    var box = b.aabb
    var cx = Math.floor((box.base[0] + box.max[0]) / 2)
    var cz = Math.floor((box.base[2] + box.max[2]) / 2)
    var y0 = Math.floor(box.base[1])
    var y1 = Math.floor(box.max[1])
    var submerged = 0
    for (var cy = y0; cy <= y1; ++cy) {
        if (self.testFluid(cx, cy, cz)) {
            ++submerged
        } else {
            break
        }
    }

    if (submerged > 0) {
        // find how much of body is submerged
        var fluidLevel = y0 + submerged
        var heightInFluid = fluidLevel - box.base[1]
        var ratioInFluid = heightInFluid / box.vec[1]
        if (ratioInFluid > 1) ratioInFluid = 1
        var vol = box.vec[0] * box.vec[1] * box.vec[2]
        var displaced = vol * ratioInFluid
        // bouyant force = -gravity * fluidDensity * volumeDisplaced
        vec3.scale(g, self.gravity, -self.fluidDensity * displaced)
        // drag force = -dv for some constant d. Here scale it down by ratioInFluid
        vec3.scale(friction, b.velocity, -self.fluidDrag * ratioInFluid)
        vec3.add(g, g, friction)
        b.applyForce(g)
        b.inFluid = true
    } else {
        b.inFluid = false
    }

    // sleep check
    var vsq = vec3.squaredLength(b.velocity)
    if (vsq > 1e-5) b._markActive()
}






/*
 *    FRICTION
*/


function applyFrictionByAxis(axis, body, dvel) {
    // friction applies only if moving into a touched surface
    var restDir = body.resting[axis]
    var vNormal = dvel[axis]
    if (restDir === 0) return
    if (restDir * vNormal <= 0) return

    // current vel lateral to friction axis
    vec3.copy(lateralVel, body.velocity)
    lateralVel[axis] = 0
    var vCurr = vec3.length(lateralVel)
    if (equals(vCurr, 0)) return

    // treat current change in velocity as the result of a pseudoforce
    //        Fpseudo = m*dv/dt
    // Base friction force on normal component of the pseudoforce
    //        Ff = u * Fnormal
    //        Ff = u * m * dvnormal / dt
    // change in velocity due to friction force
    //        dvF = dt * Ff / m
    //            = dt * (u * m * dvnormal / dt) / m
    //            = u * dvnormal
    var dvMax = Math.abs(body.friction * vNormal)

    // decrease lateral vel by dvMax (or clamp to zero)
    var scaler = (vCurr > dvMax) ? (vCurr - dvMax) / vCurr : 0
    body.velocity[(axis + 1) % 3] *= scaler
    body.velocity[(axis + 2) % 3] *= scaler
}
var lateralVel = vec3.create()






/*
 *    COLLISION HANDLER
*/

// sweep aabb along velocity vector and set resting vector
function processCollisions(self, box, velocity, resting) {
    vec3.set(resting, 0, 0, 0)
    return sweep(self.testSolid, box, velocity, function (dist, axis, dir, vec) {
        resting[axis] = dir
        vec[axis] = 0
    })
}





/*
 *    AUTO-STEPPING
*/

var tmpBox = new aabb([], [])
var tmpResting = vec3.create()
var targetPos = vec3.create()
var upvec = vec3.create()
var leftover = vec3.create()

function tryAutoStepping(self, b, oldBox, dx) {
    if (b.resting[1] >= 0 && !b.inFluid) return

    // // direction movement was blocked before trying a step
    var xBlocked = (b.resting[0] !== 0)
    var zBlocked = (b.resting[2] !== 0)
    if (!(xBlocked || zBlocked)) return

    // continue autostepping only if headed sufficiently into obstruction
    var ratio = Math.abs(dx[0] / dx[2])
    var cutoff = 4
    if (!xBlocked && ratio > cutoff) return
    if (!zBlocked && ratio < 1 / cutoff) return

    // original target position before being obstructed
    vec3.add(targetPos, oldBox.base, dx)

    // move towards the target until the first X/Z collision
    var getVoxels = self.testSolid
    sweep(getVoxels, oldBox, dx, function (dist, axis, dir, vec) {
        if (axis === 1) vec[axis] = 0
        else return true
    })

    var y = b.aabb.base[1]
    var ydist = Math.floor(y + 1.001) - y
    vec3.set(upvec, 0, ydist, 0)
    var collided = false
    // sweep up, bailing on any obstruction
    sweep(getVoxels, oldBox, upvec, function (dist, axis, dir, vec) {
        collided = true
        return true
    })
    if (collided) return // could't move upwards

    // now move in X/Z however far was left over before hitting the obstruction
    vec3.subtract(leftover, targetPos, oldBox.base)
    leftover[1] = 0
    processCollisions(self, oldBox, leftover, tmpResting)

    // bail if no movement happened in the originally blocked direction
    if (xBlocked && !equals(oldBox.base[0], targetPos[0])) return
    if (zBlocked && !equals(oldBox.base[2], targetPos[2])) return

    // done - oldBox is now at the target autostepped position
    cloneAABB(b.aabb, oldBox)
    b.resting[0] = tmpResting[0]
    b.resting[2] = tmpResting[2]
    if (b.onStep) b.onStep()
}





/*
 *    SLEEP CHECK
*/

function bodyAsleep(self, body, dt, noGravity) {
    if (body._sleepFrameCount > 0) return false
    // without gravity bodies stay asleep until a force/impulse wakes them up
    if (noGravity) return true
    // otherwise check body is resting against something
    // i.e. sweep along by distance d = 1/2 g*t^2
    // and check there's still a collision
    var isResting = false
    var gmult = 0.5 * dt * dt * body.gravityMultiplier
    vec3.scale(sleepVec, self.gravity, gmult)
    sweep(self.testSolid, body.aabb, sleepVec, function () {
        isResting = true
        return true
    }, true)
    return isResting
}
var sleepVec = vec3.create()





function equals(a, b) { return Math.abs(a - b) < 1e-5 }

function cloneAABB(tgt, src) {
    for (var i = 0; i < 3; i++) {
        tgt.base[i] = src.base[i]
        tgt.max[i] = src.max[i]
        tgt.vec[i] = src.vec[i]
    }
}


