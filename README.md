# go-collider
Golang collider library for particle simulations

This library manages a set of agent spheres and runs a simulation tick. For each
tick, the library ensures the agents will move towards their target velocity,
but will never collide (i.e. overlap)[^1].

[^1]: The agent may still run over other units if configured to do so.
      Projectiles are not checked for collisions.
