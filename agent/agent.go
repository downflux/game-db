package agent

import (
	"time"

	"github.com/downflux/game-db/agent/mask"
	"github.com/downflux/go-bvh/id"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/2d/vector/polar"
	"github.com/downflux/go-geometry/epsilon"
	"github.com/downflux/go-geometry/nd/hyperrectangle"

	vnd "github.com/downflux/go-geometry/nd/vector"
)

// O is an agent constructor option struct. All numbers are in SI units, e.g.
// meters, seconds, etc.
type O struct {
	Mass     float64
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

	mass     float64
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

func (a *A) Mass() float64      { return a.mass }
func (a *A) Position() vector.V { return a.position.V() }
func (a *A) Velocity() vector.V { return a.velocity.V() }
func (a *A) Radius() float64    { return a.radius }
func (a *A) Heading() polar.V   { return a.heading.V() }

func (a *A) IsProjectile() bool { return a.mask&mask.MSizeProjectile != 0 }

func IsSquishableColliding(a *A, b *A) bool {
	if a.id == b.id {
		return false
	}

	if IsColliding(a, b) {
		// TODO(minkezhang): Check for team.
		if a.mask&mask.SizeCheck > b.mask&mask.SizeCheck {
			return false
		}
		return true
	}
	return false
}

// IsColliding checks if two agents are actually physically overlapping. This
// does not care about the extra logic for e.g. squishing.
func IsColliding(a *A, b *A) bool {
	if a.id == b.id {
		return false
	}

	r := a.Radius() + b.Radius()
	if vector.SquaredMagnitude(vector.Sub(a.Position(), b.Position())) > r*r {
		return false
	}

	if (a.mask|b.mask)&mask.MSizeProjectile != 0 {
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
	h.Copy(polar.Normalize(o.Heading))

	a := &A{
		mass:     o.Mass,
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

// SetCollisionVelocityStrict geenerates a velocity vector for two colliding
// objects.
//
// The input velocity vector v is a velocity buffer for the first agent a; if
// this vector points towards the neighbor b, then the vector as a whole is set
// to zero -- that is, a is forced to stop for the current tick.
//
// This is a much simpler way to deal with the three body problem -- this, the
// case of when the constant "flip-flip" from SetCollisionVelocity can
// accidentally flip the velocity vector back into a neighbor.
func SetCollisionVelocityStrict(a *A, b *A, v vector.M) {
	// Find the unit collision vector pointing from a to b.
	buf := vector.M{0, 0}
	buf.Copy(b.Position())
	buf.Sub(a.Position())

	// If the vectors are pointing in the same direction, then force the
	// object to stop moving.
	if c := vector.Dot(buf.V(), v.V()); c > 0 {
		v.SetX(0)
		v.SetY(0)
	}
}

// SetCollisionVelocity generates a velocity vector for two colliding objects by
// setting the normal components to zero. This does not model, and does not
// intend to model, an inelastic collision -- this is the final check we do to
// avoid odd rendering behavior where a unit is forced into a collision.
//
// In the case of an inelastic collision between three objects, we can imagine a
// small object stuck between two massive ones. The collision vector for either
// massive object is not changed very much by the middle object, but because the
// two massive objects are not yet colliding, they will continue moving towards
// one another, which will force a collision with the middle object.
//
// The collision avoidance layer should generate velocities for each agent which
// anticipates collisions (which itself may be modeled as a near-field inelastic
// repulsive force) to avoid unintuitive pathing behavior.
//
// This function does not check if we need to generate a collision velocity in
// the first place -- that should be done by the caller by e.g. checking for
// radius overlap.
//
// To generate a final collision vector between several colliding objects, this
// function should be called iteratively for a single object and other
// colliders, e.g.
//
//	v := vector.M{0, 0}
//	v.Copy(a.Velocity())
//
//	SetCollisionVelocity(a, b, v)
//	SetCollisionVelocity(a, c, v)
//
// This allows us to generate a final velocity for agent a.
//
// N.B.: This may generate a velocity vector which flips back into a forbidden
// zone. In order to take this into account, the caller must do two passes,
// where the second pass calls SetCollisionVelocityStrict to force the velocity
// to zero in case of continued velocity violations.
func SetCollisionVelocity(a *A, b *A, v vector.M) {
	// Find the unit collision vector pointing from a to b.
	buf := vector.M{0, 0}
	buf.Copy(b.Position())
	buf.Sub(a.Position())
	buf.Unit()

	// If the vectors are pointing in the same direction, then force the
	// object to stop moving.
	if c := vector.Dot(buf.V(), v.V()); c > 0 {
		buf.Scale(c)
		v.Sub(buf.V())
	}
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

// SetHeading sets the input velocity and heading vectors to the appropriate
// simulated values for the next tick.
//
// TODO(minkezhang): Handle agents that can reverse.
func SetHeading(a *A, d time.Duration, v vector.M, h polar.M) {
	if epsilon.Within(vector.Magnitude(v.V()), 0) {
		return
	}

	h.Copy(a.Heading())
	p := polar.Polar(v.V())

	// We do not need to worry about scaling v by t, as we only care about
	// the angular difference between v and the heading.
	omega := a.maxAngularVelocity * (float64(d) / float64(time.Second))

	if a.Heading().Theta() > p.Theta() {
		if a.Heading().Theta()-p.Theta() > omega {
			h.SetTheta(a.Heading().Theta() - omega)
			v.SetX(0)
			v.SetY(0)
		} else {
			h.SetTheta(p.Theta())
		}
	} else if a.Heading().Theta() < p.Theta() {
		if p.Theta()-a.Heading().Theta() > omega {
			h.SetTheta(a.Heading().Theta() + omega)
			v.SetX(0)
			v.SetY(0)
		} else {
			h.SetTheta(p.Theta())
		}
	}

	h.Normalize()
}
