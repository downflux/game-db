package agent

import (
	"fmt"

	"github.com/downflux/go-bvh/id"
	"github.com/downflux/go-collider/mask"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/2d/vector/polar"
	"github.com/downflux/go-geometry/nd/hyperrectangle"

	vnd "github.com/downflux/go-geometry/nd/vector"
)

// O is an agent constructor option struct. All numbers are in SI units, e.g.
// meters, seconds, etc.
type O struct {
	Position vector.V
	Velocity vector.V

	// Radius is a non-negative number representing the size of the agent.
	Radius  float64
	Heading polar.V

	MaxVelocity        float64
	MaxAngularVelocity float64
	MaxAcceleration    float64

	Mask mask.M
}

type A struct {
	id id.ID

	position vector.M
	velocity vector.M
	radius   float64

	// heading is a unit polar vector whose angular component is oriented to
	// the positive X-axis. The angle is calculated according to normal 2D
	// rotational rules, i.e. a vector lying on the positive Y-axis has an
	// angular componet of π / 2.
	heading polar.M

	maxVelocity        float64
	maxAngularVelocity float64
	maxAcceleration    float64

	mask mask.M
}

func (a *A) ID() id.ID { return a.id }

func (a *A) Mask() mask.M                { return a.mask }
func (a *A) Position() vector.V          { return a.position.V() }
func (a *A) Velocity() vector.V          { return a.velocity.V() }
func (a *A) Radius() float64             { return a.radius }
func (a *A) Heading() polar.V            { return a.heading.V() }
func (a *A) MaxVelocity() float64        { return a.maxVelocity }
func (a *A) MaxAcceleration() float64    { return a.maxAcceleration }
func (a *A) MaxAngularVelocity() float64 { return a.maxAngularVelocity }

func (a *A) IsProjectile() bool { return a.mask&mask.MSizeProjectile != 0 }

// SetID is only called by internal libraries. This function must not be invoked
// by external users.
func SetID(a *A, x id.ID) { a.id = x }

// New is only called by internal libraries.
func New(o O) *A {
	if !mask.Validate(o.Mask) {
		panic(fmt.Sprintf("cannot create agent: invalid mask %v", o.Mask))
	}

	p := vector.V([]float64{0, 0}).M()
	p.Copy(o.Position)
	v := vector.V([]float64{0, 0}).M()
	v.Copy(o.Velocity)
	h := polar.V([]float64{0, 0}).M()
	h.Copy(polar.Normalize(o.Heading))

	a := &A{
		position: p,
		velocity: v,
		radius:   o.Radius,
		heading:  h,

		maxVelocity:        o.MaxVelocity,
		maxAngularVelocity: o.MaxAngularVelocity,
		maxAcceleration:    o.MaxAcceleration,

		mask: o.Mask,
	}
	return a
}

func AABB(p vector.V, r float64) hyperrectangle.R {
	x, y := p.X(), p.Y()
	return *hyperrectangle.New(
		vnd.V{
			x - r,
			y - r,
		},
		vnd.V{
			x + r,
			y + r,
		},
	)
}

func SetVelocity(a *A, v vector.M) {
	if c := vector.Magnitude(v.V()); c > a.maxVelocity {
		v.Scale(a.maxVelocity / c)
	}

	buf := vector.M{0, 0}
	buf.Copy(v.V())
	buf.Sub(a.Velocity())

	// Do not apply acceleration limits for breaking.
	if vector.Magnitude(a.Velocity()) < vector.Magnitude(v.V()) {
		if c := vector.Magnitude(buf.V()); c > a.maxAcceleration {
			buf.Scale(a.maxAcceleration / c)
			v.Copy(a.Velocity())
			v.Add(buf.V())
		}
	}
}
