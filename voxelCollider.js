'use strict'


// simpler, much faster fork of chrisdickinson/collide-3d-tilemap


module.exports = function(solidityGetter) {

  var coords = new Float32Array(3)

  return function collide(box, vec, oncollision) {
    // collide x, then y - if vector has a nonzero component
    if (vec[0] !== 0) collideaxis(0, 1, 2, box, vec, oncollision)
    if (vec[1] !== 0) collideaxis(1, 2, 0, box, vec, oncollision)
    if (vec[2] !== 0) collideaxis(2, 1, 0, box, vec, oncollision)
  }

  function ceil(n) {
    return (n === 0) ? 0 : Math.ceil(n)
  }

  function collideaxis(i_axis, j_axis, k_axis, box, vec, oncollision) {

    var dir = (vec[i_axis] > 0) ? 1 : -1
    var leading = (dir > 0) ? box.max[i_axis] : box.base[i_axis]
    var i_start = Math.floor(leading)
    var i_end = (Math.floor((leading + vec[i_axis]))) + dir
    var j_start = Math.floor(box.base[j_axis])
    var j_end = ceil(box.max[j_axis])
    var k_start = Math.floor(box.base[k_axis])
    var k_end = ceil(box.max[k_axis])
    var edge_vector
    var edge
    var solidity

    // loop from the current tile coord to the dest tile coord
    //    -> loop on the opposite axis to get the other candidates
    //      -> if `oncollision` return `true` we've hit something and
    //         should break out of the loops entirely.
    //         NB: `oncollision` is where the client gets the chance
    //         to modify the `vec` in-flight.
    // once we're done translate the box to the vec results

    outer:
    for (var i = i_start; i !== i_end; i += dir) {
      coords[i_axis] = i
      for (var j = j_start; j !== j_end; ++j) {
        coords[j_axis] = j
        for (var k = k_start; k !== k_end; ++k) {
          coords[k_axis] = k

          solidity = solidityGetter(coords[0], coords[1], coords[2])
          if (!solidity) continue
          
          edge = dir > 0 ? i : i + 1
          edge_vector = edge - leading

          if (oncollision(i_axis, solidity, coords, dir, edge_vector)) {
            break outer
          }
        }
      }
    }

    coords[0] = coords[1] = coords[2] = 0
    coords[i_axis] = vec[i_axis]
    box.translate(coords)
  }
}



