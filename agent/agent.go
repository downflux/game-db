package agent

import (
	"github.com/downflux/game-db/agent/mask"
	"github.com/downflux/go-bvh/id"
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

	MaxVelocity            float64
	MaxAngularVelocity     float64
	MaxAcceleration        float64
	MaxAngularAcceleration float64

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

	maxVelocity            float64
	maxAngularVelocity     float64
	maxAcceleration        float64
	maxAngularAcceleration float64

	mask mask.M
}

func (a *A) ID() id.ID { return a.id }

func (a *A) Position() vector.V { return a.position.V() }
func (a *A) Velocity() vector.V { return a.velocity.V() }
func (a *A) Radius() float64    { return a.radius }
func (a *A) Heading() polar.V   { return a.heading.V() }

func IsColliding(a *A, b *A) bool {
	if a.id == b.id {
		return true
	}

	r := a.Radius() + b.Radius()
	if vector.SquaredMagnitude(vector.Sub(a.Position(), b.Position())) > r*r {
		return false
	}

	if a.mask|b.mask&mask.MSizeProjectile != 0 {
		return false
	}

	// Agents are able to overlap if (only) one of them is in the air.
	if a.mask^b.mask&mask.MTerrainAir == mask.MTerrainAir {
		return false
	}
	return true
}

// SetID is only called by internal libraries. This function must not be invoked
// by external users.
func SetID(a *A, x id.ID) { a.id = x }

// New is only called by internal libraries.
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

func SetLeafAABB(a *A, q hyperrectangle.M) {
	r := a.Radius()
	x, y := a.Position().X(), a.Position().Y()
	q.Min().SetX(vnd.AXIS_X, x-r)
	q.Min().SetX(vnd.AXIS_Y, y-r)
	q.Max().SetX(vnd.AXIS_X, x+r)
	q.Max().SetX(vnd.AXIS_Y, y+r)
}

func LeafAABB(a *A) hyperrectangle.R {
	buf := hyperrectangle.New(vnd.V{0, 0}, vnd.V{0, 0}).M()
	SetLeafAABB(a, buf)
	return buf.R()
}

func SetBroadPhaseAABB(a *A, q hyperrectangle.M) {
	r := 3 * a.Radius()
	x, y := a.Position().X(), a.Position().Y()
	q.Min().SetX(vnd.AXIS_X, x-r)
	q.Min().SetX(vnd.AXIS_Y, y-r)
	q.Max().SetX(vnd.AXIS_X, x+r)
	q.Max().SetX(vnd.AXIS_Y, y+r)
}

func BroadPhaseAABB(a *A) hyperrectangle.R {
	buf := hyperrectangle.New(vnd.V{0, 0}, vnd.V{0, 0}).M()
	SetBroadPhaseAABB(a, buf)
	return buf.R()
}
