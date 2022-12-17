package agent

import (
	"github.com/downflux/game-db/agent/mask"
	"github.com/downflux/go-geometry/2d/vector"
)

type A struct {
	position vector.M
	velocity vector.M
	radius   float64

	// theta is a radian angle relative to the positive X-axis.
	theta float64

	maxVelocity        float64
	maxAngularVelocity float64
	maxAcceleration    float64

	mask mask.M
}

func (a *A) Position() vector.V { return a.position.V() }
func (a *A) Velocity() vector.V { return a.velocity.V() }
func (a *A) Radius() float64    { return a.radius }
func (a *A) Theta() float64     { return a.theta }

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
