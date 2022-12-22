package db

import (
	"fmt"
	"sync"
	"time"

	"github.com/downflux/go-bvh/bvh"
	"github.com/downflux/go-bvh/container"
	"github.com/downflux/go-bvh/id"
	"github.com/downflux/go-collisions/agent"
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

type DB struct {
	agents      map[id.ID]*agent.A
	projectiles map[id.ID]*agent.A

	// TODO(minkezhang): Add wall collision detection.

	bvhL sync.RWMutex
	bvh  container.C

	poolSize int
	counter  uint64
}

func New(o O) *DB {
	if o.PoolSize < 2 {
		panic(fmt.Sprintf("PoolSize specified %v is smaller than the minimum value of 2", o.PoolSize))
	}
	return &DB{
		agents:      make(map[id.ID]*agent.A, 1024),
		projectiles: make(map[id.ID]*agent.A, 1024),
		bvh: bvh.New(bvh.O{
			K:         2,
			LeafSize:  o.LeafSize,
			Tolerance: o.Tolerance,
		}),
		poolSize: o.PoolSize,
	}
}

func (db *DB) Delete(x id.ID) {
	db.bvhL.Lock()
	defer db.bvhL.Unlock()

	a := db.getOrDie(x)
	if a.IsProjectile() {
		delete(db.projectiles, x)
	} else {
		delete(db.agents, x)
		if err := db.bvh.Remove(x); err != nil {
			panic(fmt.Sprintf("cannot delete agent: %v", err))
		}
	}
}

func (db *DB) Insert(o agent.O) *agent.A {
	db.bvhL.Lock()
	defer db.bvhL.Unlock()

	x := id.ID(db.counter)
	db.counter += 1

	a := agent.New(o)
	agent.SetID(a, x)

	if a.IsProjectile() {
		db.projectiles[x] = a
	} else {
		db.agents[x] = a
		if err := db.bvh.Insert(x, agent.AABB(a.Position(), a.Radius())); err != nil {
			panic(fmt.Sprintf("cannot insert agent: %v", err))
		}
	}

	return a
}

// Neighbors returns a list of neighboring agents to the input.
func (db *DB) Neighbors(x id.ID, q hyperrectangle.R, filter func(a, b *agent.A) bool) []id.ID {
	db.bvhL.RLock()
	defer db.bvhL.RUnlock()

	return db.neighbors(x, q, filter)
}

func (db *DB) neighbors(x id.ID, q hyperrectangle.R, filter func(a, b *agent.A) bool) []id.ID {
	a := db.getOrDie(x)
	broadphase := db.bvh.BroadPhase(q)
	collisions := make([]id.ID, 0, len(broadphase))
	for _, y := range broadphase {
		b := db.agents[y]
		if filter(a, b) {
			collisions = append(collisions, y)
		}
	}
	return collisions
}

func (db *DB) GetOrDie(x id.ID) *agent.A {
	db.bvhL.RLock()
	defer db.bvhL.RUnlock()

	return db.getOrDie(x)
}

func (db *DB) getOrDie(x id.ID) *agent.A {
	a, ok := db.agents[x]
	if !ok {
		a = db.projectiles[x]
	}
	if a == nil {
		panic(fmt.Sprintf("cannot find agent %v", x))
	}
	return a
}

func (db *DB) SetPosition(x id.ID, v vector.V) {
	db.bvhL.Lock()
	defer db.bvhL.Unlock()

	a := db.getOrDie(x)
	a.Position().M().Copy(v)

	if !a.IsProjectile() {
		db.bvh.Update(x, agent.AABB(a.Position(), a.Radius()))
	}
}

func (db *DB) SetVelocity(x id.ID, v vector.V) {
	// SetVelocity does not mutate the BVH, but the central Tick function
	// does need to read the velocity.
	db.bvhL.RLock()
	defer db.bvhL.RUnlock()

	db.getOrDie(x).Velocity().M().Copy(v)
}

func (db *DB) generate() []result {
	results := make([]result, 0, 256)

	in := make(chan *agent.A, 256)
	out := make(chan result, 256)

	go func(ch chan<- *agent.A) {
		for _, a := range db.agents {
			ch <- a
		}
		close(ch)
	}(in)

	go func(in <-chan *agent.A, out chan<- result) {

		var wg sync.WaitGroup

		wg.Add(db.poolSize)
		go func(ch chan<- result) {
			defer wg.Done()
			for _, a := range db.projectiles {
				out <- result{
					agent: a,
					v:     a.Velocity(),
				}
			}
		}(out)

		for i := 0; i < db.poolSize-1; i++ {
			go func(in <-chan *agent.A, out chan<- result) {
				defer wg.Done()
				for a := range in {
					v := vector.M{0, 0}
					// TODO(minkezhang): Investigate what happens if
					// we change this velocity to the nearest
					// 8-directional alignment.
					v.Copy(a.Velocity())

					ns := db.neighbors(
						a.ID(),
						agent.AABB(
							a.Position(),
							a.Radius(),
						),
						agent.IsSquishableColliding,
					)

					// Check for collisions which the agent cares
					// about, e.g. care about squishability.
					for _, y := range ns {
						agent.SetCollisionVelocity(a, db.agents[y], v)
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
					//
					// TODO(minkezhang): Take into account
					// walls as additional input points
					// here.
					if len(ns) > 1 {
						for _, y := range ns {
							agent.SetCollisionVelocityStrict(a, db.agents[y], v)
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
func (db *DB) Tick(d time.Duration) {
	db.bvhL.Lock()
	defer db.bvhL.Unlock()

	in := make(chan result, 256)
	go func(ch chan<- result) {
		for _, r := range db.generate() {
			ch <- r
		}
		close(ch)
	}(in)

	t := float64(d) / float64(time.Second)

	var wg sync.WaitGroup
	wg.Add(db.poolSize)

	for i := 0; i < db.poolSize; i++ {

		go func(ch <-chan result) {
			defer wg.Done()
			for r := range ch {
				agent.SetVelocity(r.agent, r.v.M())

				// N.B.: The velocity can be further reduced to
				// zero here due to the physical limitations of
				// the agent.
				h := polar.V{1, r.agent.Heading().Theta()}
				agent.SetHeading(r.agent, d, r.v.M(), h.M())

				r.agent.Position().M().Add(vector.Scale(t, r.v))
				r.agent.Heading().M().Copy(h)
			}
		}(in)
	}

	wg.Wait()

	// Concurrent BVH mutations is not supported.
	for x, a := range db.agents {
		db.bvh.Update(x, agent.AABB(a.Position(), a.Radius()))
	}
}

type result struct {
	agent *agent.A
	v     vector.V
}
