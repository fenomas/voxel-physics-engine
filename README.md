# voxel-physics-engine
An abstracted physics engine for my experimental branch of voxeljs.

This adds reasonably realistic physics to voxel js.
It replaces and is modeled loosely on 
[voxel-physical](https://github.com/chrisdickinson/voxel-physical),
though it works quite differently and behaves more physically.

###Usage:
 1. Register rigid bodies with `addBody()`
 1. Control them with `body.addImpulse[0,1,0]` and such
 1. Call `tick(dt)`

and the engine will manage all the physics and collisions.

At the moment it's tightly coupled to my experimental branch of
voxel-engine#ndarray, and wouldn't work in standard voxeljs.
