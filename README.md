# voxel-physics-engine
An abstracted physics engine for my experimental branch of voxeljs.

It's modeled loosely after 
[voxel-physical](https://github.com/chrisdickinson/voxel-physical),
though it works quite differently. 

###Usage:
 1. Register rigid bodies with `addBody()`
 1. Control them with `body.addImpulse[0,1,0]` and such
 1. Call `tick(dt)`

and the engine will manage all the physics and collisions.

Not really usable yet - it's still tightly coupled to several 
other modules. Extrication is underway but gradual.
