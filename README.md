# voxel-physics-engine
An abstracted physics engine for voxel game engines.

This implements reasonably realistic physics for voxel games.
It was made to work with voxel.js, but it only takes references to 
an abstracted `getBlock(x,y,z)` function and plain vectors, 
so it can be used with other engines. 

This replaces, and is modeled very loosely on, 
[voxel-physical](https://github.com/chrisdickinson/voxel-physical),
though it works quite differently and behaves more physically.

The engine can be seen in action in [noa-hello-world](https://github.com/andyhall/noa-hello-world).

###Usage:
 1. Create engine and supply a `getBlock(x,y,z)`-like function (which should return true when block xyz is solid)
 1. Register rigid bodies with `var body = addBody(..)`
 1. Control them with `body.addImpulse[0,1,0]` and so on
 1. Call `tick(dt)`

and the engine will manage all the physics and collisions.

#### Example

``` javascript
var makePhysics = require('voxel-physics-engine')
var opts = { gravity: [0,-10,0] }
var getter = queryBlock // function(x,y,z) 
var phys = makePhysics( opts, getter )
var body = phys.addBody( aabb, mass, friction, restitution, gravityMult, onCollide, autoStep )
phys.tick( dt_in_miliseconds )
phys.removeBody( body )
```

#### Features

 * Reasonably physical, supports standard properties (friction, restitution, etc.)
 * Query the body's `resting[axis]` property (-1,0,1 on each axis) to tell if it's currently resting against an obstacle.
   E.g. `body.resting[1]` is `1` when the body is colliding with the ceiling.
 * Collisions with terrain trigger each body's `onCollide(impacts)` callback. 
   The argument is a `vec3` of the change in velocity on each axis, so multiply by mass if you want the impulse.
   (A body resting on the ground will produce a small impact each tick due to gravity.)
 * If you set a body's `autoStep` property, the engine will cause it to 
   automatically "step" up hills (i.e. 1-block obstructions).
   