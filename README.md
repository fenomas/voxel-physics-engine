# voxel-physics-engine

Abstracted terrain physics for voxel game engines.

This implements reasonably realistic physics for voxel games. It was made to work with [noa](https://github.com/fenomas/noa) or `voxel.js`, but all it requires is abstracted `(x,y,z) => boolean` test functions for voxel properties, so it can be used with any engine.

This library replaces [voxel-physical](https://github.com/chrisdickinson/voxel-physical), though it works quite differently and behaves more physically. The engine can be seen in action in [noa](https://github.com/fenomas/noa) or projects using it.

Note: this simple engine only handles collisions between bodies and solid voxels. It does not handle collisions between rigid bodies, or non-cubic voxels.


### Usage:
 1. Create engine and supply a `(x,y,z) => boolean` functions that return whether a given voxel is solid or liquid.
 1. Add rigid bodies with `var body = addBody(..)`
 1. Modify body properties, or call its methods to apply forces, impulses, etc.
 1. Call `tick(dt)` on the engine object

and the engine will update each body and handle voxel collisions.

#### Example

``` javascript
import { Physics } from 'voxel-physics-engine'
var opts = { gravity: [0, -10, 0] }
var voxelIsSolid = (x, y, z) => (y < 0)
var voxelIsLiquid = (x, y, z) => (x > 5)

var phys = new Physics(opts, voxelIsSolid, voxelIsLiquid)
var body = phys.addBody( aabb, mass, friction, restitution, gravityMult, onCollide )
phys.tick( dt_in_miliseconds )
phys.removeBody( body )
```

#### Features

 * Reasonably physical, supports standard properties (friction, restitution, etc.)
 * Query the body's `resting[axis]` property (-1,0,1 on each axis) to tell if it's currently touching a solid voxel.
   E.g. `body.resting[1]` is `1` when the body is colliding with the ceiling.
 * Collisions with terrain trigger each body's `onCollide(impacts)` callback. 
   The argument is a `vec3` of the collision impulse on each axis.
   (A body resting on the ground will produce a small impact each tick due to gravity.)
 * If you set a body's `autoStep` property, the engine will cause it to 
   automatically "step" onto one-block obstructions.
  
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

