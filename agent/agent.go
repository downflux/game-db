package agent

import (
	"github.com/downflux/game-db/agent/mask"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/2d/vector/polar"
)

type O struct {
	Position vector.V
	Velocity vector.V
	Radius   float64
	Heading  polar.V

	MaxVelocity            float64
	MaxAngularVelocity     float64
	MaxAcceleration        float64
	MaxAngularAcceleration float64

	Mask mask.M
}

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

func New(o O) *A {
	p := vector.V([]float64{0, 0}).M()
	p.Copy(o.Position)
	v := vector.V([]float64{0, 0}).M()
	v.Copy(o.Velocity)
	h := polar.V([]float64{0, 0}).M()
	h.Copy(o.Heading)

	a := &A{
		position: p,
		velocity: v,
		radius:   o.Radius,
		heading:  h,

		maxVelocity:            o.MaxVelocity,
		maxAngularVelocity:     o.MaxAngularVelocity,
		maxAcceleration:        o.MaxAcceleration,
		maxAngularAcceleration: o.MaxAngularAcceleration,
	}
	return a
}

func (a *A) SetMaxVelocity(c float64)            { a.maxVelocity = c }
func (a *A) SetMaxAngularVelocity(c float64)     { a.maxAngularVelocity = c }
func (a *A) SetMaxAcceleration(c float64)        { a.maxAcceleration = c }
func (a *A) SetMaxAngularAcceleration(c float64) { a.maxAngularAcceleration = c }

func (a *A) Position() vector.V { return a.position.V() }
func (a *A) Velocity() vector.V { return a.velocity.V() }
func (a *A) Radius() float64    { return a.radius }
func (a *A) Heading() polar.V   { return a.heading.V() }

// SetPosition and associated mutation functions are used by external API to set
// these values directly. Internally, we will leverage the mutability criteria
// of these values to incrementally update the agent.
func (a *A) SetPosition(v vector.V) { a.position.Copy(v) }
func (a *A) SetVelocity(v vector.V) { a.velocity.Copy(v) }
func (a *A) SetRadius(c float64)    { a.radius = c }
func (a *A) SetHeading(v polar.V)   { a.heading.Copy(v) }

// IgnoreCollision checks if two agents need to do any additional processing in
// the case that their bounding circles overlap.
func IgnoreCollision(a *A, b *A) bool {
	if a.mask|b.mask&mask.MSizeProjectile != 0 {
		return true
	}

	// Agents are able to overlap if (only) one of them is in the air.
	if a.mask^b.mask&mask.MTerrainAir == mask.MTerrainAir {
		return true
	}
	return false
}
