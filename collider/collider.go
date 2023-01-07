package collider

import (
	"fmt"
	"sync"
	"time"

	"github.com/downflux/go-collider/internal/kinematics"
	"github.com/downflux/go-database/agent"
	"github.com/downflux/go-database/database"
	"github.com/downflux/go-database/feature"
	"github.com/downflux/go-database/filters"
	"github.com/downflux/go-database/projectile"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/2d/vector/polar"
)

var (
	// DefaultO provides a default set of options for setting up the
	// collider. The values here are tailored to an N = 1000 simulation, and
	// is dependent on a variety of factors, e.g. CPU count.
	DefaultO O = O{
		PoolSize: 24,
	}
)

type O struct {
	PoolSize int
}

type C struct {
	db       *database.DB
	poolSize int
}

func New(db *database.DB, o O) *C {
	if o.PoolSize < 2 {
		panic(fmt.Sprintf("PoolSize specified %v is smaller than the minimum value of 2", o.PoolSize))
	}
	return &C{
		db:       db,
		poolSize: o.PoolSize,
	}
}

func (c *C) generate(d time.Duration) ([]am, []pm) {
	ams := make([]am, 0, 256)
	pms := make([]pm, 0, 256)

	amsch := make(chan am, 256)
	pmsch := make(chan pm, 256)

	go func(amsch chan<- am, pmsch chan<- pm) {
		var wg sync.WaitGroup

		wg.Add(c.poolSize)
		go func(ch chan<- pm) {
			defer wg.Done()
			for p := range c.db.ListProjectiles() {
				pmsch <- pm{
					projectile: p,
					v:          p.TargetVelocity(),
					h:          polar.Polar(vector.Unit(p.TargetVelocity())),
				}
			}
			close(ch)
		}(pmsch)

		in := c.db.ListAgents()
		for i := 0; i < c.poolSize-1; i++ {
			go func(out chan<- am) {
				defer wg.Done()
				for a := range in {
					v := vector.M{0, 0}
					v.Copy(a.TargetVelocity())

					aabb := a.AABB()
					ns := c.db.QueryAgents(aabb, func(b agent.RO) bool {
						return filters.AgentIsCollidingNotSquishable(a, b)
					})
					fs := c.db.QueryFeatures(aabb, func(f feature.RO) bool {
						return filters.AgentIsCollidingWithFeature(a, f)
					})

					for _, f := range fs {
						kinematics.SetFeatureCollisionVelocity(a, f, v)
					}

					// Check for collisions which the agent
					// cares about, e.g. care about
					// squishability. These functions set
					// the input vector v to ensure that the
					// normal components of the velocity is
					// filtered out for each individual
					// entity. However, this method is not
					// always reliable, and a multi-body
					// collision may flip the velocity back
					// into the body of an existing entity.
					for _, n := range ns {
						kinematics.SetCollisionVelocity(a, n, v)
					}

					kinematics.ClampVelocity(a, v)
					kinematics.ClampAcceleration(a, v, d)

					// N.B.: The velocity can be further reduced to
					// zero here due to the physical limitations of
					// the agent.
					h := polar.M{0, 0}
					h.Copy(a.Heading())
					kinematics.ClampHeading(a, d, v, h)

					// Second pass ensures agent is not
					// colliding with any static features
					// after the velocity manipulations.
					for _, f := range fs {
						kinematics.ClampFeatureCollisionVelocity(a, f, v)
					}

					// Second pass across neighbors forces
					// the velocity to zero if a velocity
					// has flip-flopped back into the
					// forbidden zone of another agent.
					for _, n := range ns {
						kinematics.ClampCollisionVelocity(a, n, v)
					}

					out <- am{
						agent: a,
						v:     v.V(),
						h:     h.V(),
					}
				}
			}(amsch)
		}

		wg.Wait()
		close(amsch)

	}(amsch, pmsch)

	for r := range amsch {
		ams = append(ams, r)
	}
	for r := range pmsch {
		pms = append(pms, r)
	}

	return ams, pms
}

// Tick advances the world by one tick. During this execution, agents must not
// be modified by the user.
func (c *C) Tick(d time.Duration) {
	t := float64(d) / float64(time.Second)

	ams, pms := c.generate(d)
	// Concurrent BVH ams is not supported.
	for _, r := range ams {
		c.db.SetAgentPosition(r.agent.ID(), vector.Add(r.agent.Position(), vector.Scale(t, r.v)))
		c.db.SetAgentHeading(r.agent.ID(), r.h)
		c.db.SetAgentVelocity(r.agent.ID(), r.v)
	}
	for _, r := range pms {
		c.db.SetProjectilePosition(r.projectile.ID(), vector.Add(r.projectile.Position(), vector.Scale(t, r.v)))
		c.db.SetProjectileHeading(r.projectile.ID(), r.h)
		c.db.SetProjectileVelocity(r.projectile.ID(), r.v)
	}
}

type pm struct {
	projectile projectile.RO
	v          vector.V
	h          polar.V
}

type am struct {
	agent agent.RO
	v     vector.V
	h     polar.V
}
