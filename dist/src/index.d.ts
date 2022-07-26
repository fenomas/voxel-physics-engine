export function DefaultOptions(): void;
export class DefaultOptions {
    airDrag: number;
    fluidDrag: number;
    fluidDensity: number;
    gravity: number[];
    minBounceImpulse: number;
}
/**
 *          Voxel Physics Engine
 *
 * Models a world of rigid bodies, to be integrated against
 * solid or liquid voxel terrain.
 *
 * Takes `testSolid(x,y,z)` function to query block solidity
 * Takes `testFluid(x,y,z)` function to query if a block is a fluid
 *
 * The `options` argument can take the following params:
 *
 * ```js
 * {
 *     airDrag: 0.1,
 *     fluidDrag: 0.4,
 *     fluidDensity: 2.0,
 *     gravity: [0, -10, 0],
 *     minBounceImpulse: .5, // lowest collision impulse that bounces
 * }
 *
 * ```
*/
export function Physics(opts: any, testSolid: any, testFluid: any): void;
export class Physics {
    /**
     *          Voxel Physics Engine
     *
     * Models a world of rigid bodies, to be integrated against
     * solid or liquid voxel terrain.
     *
     * Takes `testSolid(x,y,z)` function to query block solidity
     * Takes `testFluid(x,y,z)` function to query if a block is a fluid
     *
     * The `options` argument can take the following params:
     *
     * ```js
     * {
     *     airDrag: 0.1,
     *     fluidDrag: 0.4,
     *     fluidDensity: 2.0,
     *     gravity: [0, -10, 0],
     *     minBounceImpulse: .5, // lowest collision impulse that bounces
     * }
     *
     * ```
    */
    constructor(opts: any, testSolid: any, testFluid: any);
    gravity: any;
    airDrag: any;
    fluidDensity: any;
    fluidDrag: any;
    minBounceImpulse: any;
    bodies: any[];
    testSolid: any;
    testFluid: any;
    /**
     * Adds a physics body to the simulation
     * @returns {RigidBody}
    */
    addBody(_aabb: any, mass: any, friction: any, restitution: any, gravMult: any, onCollide: any): RigidBody;
    /** Removes a body, by direct reference */
    removeBody(b: any): any;
    tick(dt: any): void;
}
export { RigidBody };
import { RigidBody } from "./rigidBody";
