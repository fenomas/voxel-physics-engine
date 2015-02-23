# voxel-physics-engine
An abstracted physics engine for voxel game engines.

This implements reasonably realistic physics for voxel games.
It was made to work with voxel.js, but it only takes references to 
an abstracted `getBlock(x,y,z)` function and plain vectors, 
so it can be used with other engines. 

This replaces, and is modeled very loosely on, 
[voxel-physical](https://github.com/chrisdickinson/voxel-physical),
though it works quite differently and behaves more physically.

###Usage:
 1. Create engine and supply a `getBlock(x,y,z)`-like function (which should return true when block xyz is solid)
 1. Register rigid bodies with `var body = addBody(..)`
 1. Control them with `body.addImpulse[0,1,0]` and so on
 1. Call `tick(dt)`

and the engine will manage all the physics and collisions.

#### Example

``` javascript
var phys = require('voxel-physics-engine')()
var body = phys.addBody( aabb, mass, friction, restitution, gravityMultiplier, onCollide )
phys.tick( dt_in_miliseconds )
phys.removeBody( body )
```
