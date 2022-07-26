
var aabb = require('aabb-3d')
var vec3 = require('gl-vec3')

var DEBUG = 0



/*
 *    RIGID BODY - internal data structure
 *  Only AABB bodies right now. Someday will likely need spheres?
*/

export class RigidBody {
    constructor(_aabb, mass, friction, restitution, gravMult, onCollide, autoStep) {
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

        // internal state
        this.velocity = vec3.create()
        this.resting = [0, 0, 0]
        this.inFluid = false

        // internals
        /** @internal */
        this._ratioInFluid = 0
        /** @internal */
        this._forces = vec3.create()
        /** @internal */
        this._impulses = vec3.create()
        /** @internal */
        this._sleepFrameCount = 10 | 0
    }

    loadFromCopy(loadFrom) {
        vec3.copy(this.aabb.base, loadFrom.aabb.base)
        vec3.copy(this.aabb.vec, loadFrom.aabb.vec)
        vec3.copy(this.aabb.max, loadFrom.aabb.max)
        this.aabb.mag = loadFrom.aabb.mag
    
        this.mass = loadFrom.mass
        this.friction = loadFrom.friction
        this.restitution = loadFrom.restitution
        this.gravityMultiplier = loadFrom.gravityMultiplier
        this.onCollide = loadFrom.onCollide
        this.autoStep = loadFrom.autoStep
    
        this.airDrag = loadFrom.airDrag
        this.fluidDrag = loadFrom.fluidDrag
        this.onStep = loadFrom.onStep
    
        vec3.copy(this.velocity, loadFrom.velocity)
        vec3.copy(this.resting, loadFrom.resting)
        this.inFluid = loadFrom.inFluid
        this._ratioInFluid = loadFrom._ratioInFluid
        vec3.copy(this._forces, loadFrom._forces)
        vec3.copy(this._impulses, loadFrom._impulses)
        this._sleepFrameCount = loadFrom._sleepFrameCount
    }

    setPosition(p) {
        sanityCheck(p)
        vec3.subtract(p, p, this.aabb.base)
        this.aabb.translate(p)
        this._markActive()
    }
    getPosition() {
        return vec3.clone(this.aabb.base)
    }
    applyForce(f) {
        sanityCheck(f)
        vec3.add(this._forces, this._forces, f)
        this._markActive()
    }
    applyImpulse(i) {
        sanityCheck(i)
        vec3.add(this._impulses, this._impulses, i)
        this._markActive()
    }

    /** @internal */
    _markActive() {
        this._sleepFrameCount = 10 | 0
    }



    // temp
    atRestX() { return this.resting[0] }
    atRestY() { return this.resting[1] }
    atRestZ() { return this.resting[2] }
}




var sanityCheck = (v) => { }
if (DEBUG) sanityCheck = (v) => {
    if (isNaN(vec3.length(v))) throw 'Vector with NAN: ' + v
}
