package agent

import (
	"github.com/downflux/game-db/agent/mask"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/2d/vector/polar"
)

type A struct {
	position vector.M
	velocity vector.M
	radius   float64

	// heading is a unit polar vector whose angular component is oriented to
	// the positive X-axis. The angle is calculated according to normal 2D
	// rotational rules, i.e. a vector lying on the positive Y-axis has an
	// angular componet of π / 2.
	heading polar.M

	maxVelocity            float64
	maxAngularVelocity     float64
	maxAcceleration        float64
	maxAngularAcceleration float64

	mask mask.M
}

func (a *A) Position() vector.V { return a.position.V() }
func (a *A) Velocity() vector.V { return a.velocity.V() }
func (a *A) Radius() float64    { return a.radius }
func (a *A) Heading() polar.V   { return a.heading.V() }

// IgnoreCollision checks if two agents need to do any additional processing in
// the case that their bounding circles overlap.
func (a *A) IgnoreCollision(b *A) bool {
	if a.mask|b.mask&mask.MSizeProjectile != 0 {
		return true
	}

	// Agents are able to overlap if (only) one of them is in the air.
	if a.mask^b.mask&mask.MTerrainAir == mask.MTerrainAir {
		return true
	}
	return false
}
