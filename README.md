# voxel-physics-engine
An abstracted physics engine for voxel game engines.

This implements reasonably realistic physics for voxel games.
It was made to work with [noa](https://github.com/fenomas/noa) 
or `voxel.js`, but it just takes references to 
an abstracted `getBlock(x,y,z)` function and plain vectors, 
so it can be used with other engines. 

This replaces [voxel-physical](https://github.com/chrisdickinson/voxel-physical),
though it works quite differently and behaves more physically.

The engine can be seen in action in [noa](https://github.com/fenomas/noa) or projects using it
(such as [this talk on voxels](http://fenomas.github.io/noa-lt/).

###Usage:
 1. Create engine and supply a `getBlock(x,y,z)`-like function (which should return true when block xyz is solid)
 1. Register rigid bodies with `var body = addBody(..)`
 1. Control them with `body.addImpulse[0,1,0]` and so on
 1. Call `tick(dt)`

and the engine will manage all the physics and collisions.

#### Example

``` javascript
import {Physics} from 'voxel-physics-engine'
var opts = { gravity: [0,-10,0] }
var getter = function(x,y,z) { /* your logic here */ }

var phys = new Physics(opts, getter)
var body = phys.addBody( aabb, mass, friction, restitution, gravityMult, onCollide, autoStep )
phys.tick( dt_in_miliseconds )
phys.removeBody( body )
```

#### Features

 * Reasonably physical, supports standard properties (friction, restitution, etc.)
 * Query the body's `resting[axis]` property (-1,0,1 on each axis) to tell if it's currently touching a solid voxel.
   E.g. `body.resting[1]` is `1` when the body is colliding with the ceiling.
 * Collisions with terrain trigger each body's `onCollide(impacts)` callback. 
   The argument is a `vec3` of the change in velocity on each axis, so multiply by mass if you want the impulse.
   (A body resting on the ground will produce a small impact each tick due to gravity.)
 * If you set a body's `autoStep` property, the engine will cause it to 
   automatically "step" up hills (i.e. 1-block obstructions).
  
#### Changes in latest version:

 * `0.13.0` - minor fixes to typing and internals
 * `0.11.0` - export a named constructor function instead of a callback
 * `0.10.0`
   * Fixes `body.onCollide` arg to be a correctly scaled impulse vector
   * Body `onCollide` now passed before bounces are resolved (allowing client to adjust `body.restitution` depending on the terrain)
 * `0.9.0`
   * Air and fluid friction properties renamed to `airDrag` and `fluidDrag`, and now work equivalently (no drag at `0`, large drag at `1`)
   * Per body `airDrag` and `fluidDrag` override global settings if `>= 0`
 * `0.8.0`
   * Friction now uses regular coefficients, and works in all directions, not just downwards
   * Bodies have an `.airFriction` that overrides the global value (if nonzero)
   * Air and regular friction should now scale correctly with frame rate

