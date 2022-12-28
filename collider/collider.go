package collider

import (
	"fmt"
	"sync"
	"time"

	"github.com/downflux/go-bvh/bvh"
	"github.com/downflux/go-bvh/container"
	"github.com/downflux/go-bvh/id"
	"github.com/downflux/go-collider/agent"
	"github.com/downflux/go-collider/feature"
	"github.com/downflux/go-collider/internal/collider"
	"github.com/downflux/go-collider/internal/kinematics"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/2d/vector/polar"
	"github.com/downflux/go-geometry/nd/hyperrectangle"
)

var (
	// DefaultO provides a default set of options for setting up the
	// database. The values here are tailored to an N = 1000 simulation, and
	// is dependent on a variety of factors, e.g. CPU count and surface area
	// coverage.
	DefaultO O = O{
		LeafSize:  8,
		Tolerance: 1.15,
		PoolSize:  24,
	}
)

type O struct {
	LeafSize  int
	Tolerance float64
	PoolSize  int
}

type C struct {
	agents      map[id.ID]*agent.A
	projectiles map[id.ID]*agent.A
	features    map[id.ID]*feature.F

	bvhL        sync.RWMutex
	bvh         container.C
	bvhFeatures container.C

	poolSize int
	counter  uint64
}

func New(o O) *C {
	if o.PoolSize < 2 {
		panic(fmt.Sprintf("PoolSize specified %v is smaller than the minimum value of 2", o.PoolSize))
	}
	return &C{
		agents:      make(map[id.ID]*agent.A, 1024),
		projectiles: make(map[id.ID]*agent.A, 1024),
		features:    make(map[id.ID]*feature.F, 1024),
		bvh: bvh.New(bvh.O{
			K:         2,
			LeafSize:  o.LeafSize,
			Tolerance: o.Tolerance,
		}),
		bvhFeatures: bvh.New(bvh.O{
			K:         2,
			LeafSize:  o.LeafSize,
			Tolerance: 1,
		}),
		poolSize: o.PoolSize,
	}
}

func (c *C) InsertFeature(o feature.O) *feature.F {
	c.bvhL.Lock()
	defer c.bvhL.Unlock()

	x := id.ID(c.counter)
	c.counter += 1

	f := feature.New(o)
	feature.SetID(f, x)

	c.features[x] = f
	if err := c.bvhFeatures.Insert(x, f.AABB()); err != nil {
		panic(fmt.Sprintf("cannot insert feature: %v", err))
	}

	return f
}

func (c *C) DeleteFeature(x id.ID) {
	c.bvhL.Lock()
	defer c.bvhL.Unlock()

	f := c.getFeatureOrDie(x)
	delete(c.features, f.ID())
	if err := c.bvhFeatures.Remove(f.ID()); err != nil {
		panic(fmt.Sprintf("cannot delete feature: %v", err))
	}
}

// Insert creates a new agent and adds to the scene. This function is executed
// serially.
func (c *C) Insert(o agent.O) *agent.A {
	c.bvhL.Lock()
	defer c.bvhL.Unlock()

	x := id.ID(c.counter)
	c.counter += 1

	a := agent.New(o)
	agent.SetID(a, x)

	if a.IsProjectile() {
		c.projectiles[x] = a
	} else {
		c.agents[x] = a
		if err := c.bvh.Insert(x, agent.AABB(a.Position(), a.Radius())); err != nil {
			panic(fmt.Sprintf("cannot insert agent: %v", err))
		}
	}

	return a
}

// Delete removes an agent from the scene. This function is executed serially.
func (c *C) Delete(x id.ID) {
	c.bvhL.Lock()
	defer c.bvhL.Unlock()

	a := c.getOrDie(x)
	if a.IsProjectile() {
		delete(c.projectiles, x)
	} else {
		delete(c.agents, x)
		if err := c.bvh.Remove(x); err != nil {
			panic(fmt.Sprintf("cannot delete agent: %v", err))
		}
	}
}

// Query returns a list of neighboring agents to the input. The input filter is
// a narrow phase filter function. The function returns true if the two agents
// are found to be in conflict.
//
// This function may be called concurrently.
func (c *C) Query(q hyperrectangle.R, filter func(a *agent.A) bool) []id.ID {
	c.bvhL.RLock()
	defer c.bvhL.RUnlock()

	return c.query(q, filter)
}

func (c *C) QueryFeatures(q hyperrectangle.R, filter func(f *feature.F) bool) []id.ID {
	c.bvhL.RLock()
	defer c.bvhL.RUnlock()

	return c.queryFeatures(q, filter)
}

func (c *C) queryFeatures(q hyperrectangle.R, filter func(f *feature.F) bool) []id.ID {
	broadphase := c.bvhFeatures.BroadPhase(q)
	collisions := make([]id.ID, 0, len(broadphase))
	for _, x := range broadphase {
		f := c.features[x]
		if filter(f) {
			collisions = append(collisions, x)
		}
	}
	return collisions
}

func (c *C) query(q hyperrectangle.R, filter func(a *agent.A) bool) []id.ID {
	broadphase := c.bvh.BroadPhase(q)
	collisions := make([]id.ID, 0, len(broadphase))
	for _, x := range broadphase {
		a := c.agents[x]
		if filter(a) {
			collisions = append(collisions, x)
		}
	}
	return collisions
}

func (c *C) GetFeatureOrDie(x id.ID) *feature.F {
	c.bvhL.RLock()
	defer c.bvhL.RUnlock()

	return c.getFeatureOrDie(x)
}

// GetOrDie returns an agent from the scene. This function may be called
// concurrently.
func (c *C) GetOrDie(x id.ID) *agent.A {
	c.bvhL.RLock()
	defer c.bvhL.RUnlock()

	return c.getOrDie(x)
}

func (c *C) getFeatureOrDie(x id.ID) *feature.F {
	f, ok := c.features[x]
	if !ok {
		panic(fmt.Sprintf("cannot find feature %v", x))
	}

	return f
}

func (c *C) getOrDie(x id.ID) *agent.A {
	a, ok := c.agents[x]
	if !ok {
		a = c.projectiles[x]
	}
	if a == nil {
		panic(fmt.Sprintf("cannot find agent %v", x))
	}
	return a
}

// SetPosition sets the agent's position directly. This is not part of a normal
// user's calling pattern -- the user should default to the collision resolution
// library's Tick() to set the position indirectly. This functino is used for
// when a user wants to specify an e.g. teleportation event. This function may
// be called concurrently.
func (c *C) SetPosition(x id.ID, v vector.V) {
	c.bvhL.Lock()
	defer c.bvhL.Unlock()

	a := c.getOrDie(x)
	a.Position().M().Copy(v)

	if !a.IsProjectile() {
		c.bvh.Update(x, agent.AABB(a.Position(), a.Radius()))
	}
}

// SetVelocity directly sets the agent's new velocity vector. This function is
// called when an agent's goal vector is updated. This function may be called
// concurrently.
func (c *C) SetVelocity(x id.ID, v vector.V) {
	// SetVelocity does not mutate the BVH, but the central Tick function
	// does need to read the velocity.
	c.bvhL.RLock()
	defer c.bvhL.RUnlock()

	c.getOrDie(x).Velocity().M().Copy(v)
}

func (c *C) generate() []result {
	results := make([]result, 0, 256)

	in := make(chan *agent.A, 256)
	out := make(chan result, 256)

	go func(ch chan<- *agent.A) {
		for _, a := range c.agents {
			ch <- a
		}
		close(ch)
	}(in)

	go func(in <-chan *agent.A, out chan<- result) {

		var wg sync.WaitGroup

		wg.Add(c.poolSize)
		go func(ch chan<- result) {
			defer wg.Done()
			for _, a := range c.projectiles {
				out <- result{
					agent: a,
					v:     a.Velocity(),
				}
			}
		}(out)

		for i := 0; i < c.poolSize-1; i++ {
			go func(in <-chan *agent.A, out chan<- result) {
				defer wg.Done()
				for a := range in {
					v := vector.M{0, 0}
					// TODO(minkezhang): Investigate what
					// happens if we change this velocity to
					// the nearest 8-directional alignment.
					v.Copy(a.Velocity())

					aabb := agent.AABB(a.Position(), a.Radius())
					ns := c.query(aabb, func(b *agent.A) bool { return collider.IsSquishableColliding(a, b) })
					fs := c.queryFeatures(aabb, func(f *feature.F) bool { return collider.IsCollidingFeature(a, f) })

					for _, y := range fs {
						kinematics.SetFeatureCollisionVelocity(a, c.features[y], v)
					}

					// Check for collisions which the agent
					// cares about, e.g. care about
					// squishability.
					for _, y := range ns {
						kinematics.SetCollisionVelocity(a, c.agents[y], v)
					}

					// Second pass ensures agent is not
					// colliding with any static features.
					for _, y := range fs {
						kinematics.SetFeatureCollisionVelocityStrict(a, c.features[y], v)
					}

					// Second pass across neighbors forces
					// the velocity to zero if a velocity
					// has flip-flopped back into the
					// forbidden zone of another agent.
					//
					// Note that this flip-flop behavior is
					// only a problem if there is more than
					// one neighbor -- this actually allows
					// us to natively do some crowd-control
					// without higher-level pathing by
					// letting the edges of a collision
					// move out and gradually shrinking the
					// collision radius.
					if len(ns) > 1 {
						for _, y := range ns {
							kinematics.SetCollisionVelocityStrict(a, c.agents[y], v)
						}
					}

					out <- result{
						agent: a,
						v:     v.V(),
					}
				}
			}(in, out)
		}

		wg.Wait()
		close(out)

	}(in, out)

	for r := range out {
		results = append(results, r)
	}

	return results
}

// Tick advances the world by one tick. During this execution, agents must not
// be modified by the user.
func (c *C) Tick(d time.Duration) {
	c.bvhL.Lock()
	defer c.bvhL.Unlock()

	in := make(chan result, 256)
	go func(ch chan<- result) {
		for _, r := range c.generate() {
			ch <- r
		}
		close(ch)
	}(in)

	t := float64(d) / float64(time.Second)

	var wg sync.WaitGroup
	wg.Add(c.poolSize)

	for i := 0; i < c.poolSize; i++ {
		go func(ch <-chan result) {
			defer wg.Done()
			for r := range ch {
				agent.SetVelocity(r.agent, r.v.M())

				// N.B.: The velocity can be further reduced to
				// zero here due to the physical limitations of
				// the agent.
				h := polar.V{1, r.agent.Heading().Theta()}
				kinematics.SetHeading(r.agent, d, r.v.M(), h.M())

				r.agent.Position().M().Add(vector.Scale(t, r.v))
				r.agent.Heading().M().Copy(h)
			}
		}(in)
	}

	wg.Wait()

	// Concurrent BVH mutations is not supported.
	for x, a := range c.agents {
		c.bvh.Update(x, agent.AABB(a.Position(), a.Radius()))
	}
}

type result struct {
	agent *agent.A
	v     vector.V
}
