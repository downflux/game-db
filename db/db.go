package db

import (
	"fmt"
	"sync"
	"time"

	"github.com/downflux/game-db/agent"
	"github.com/downflux/go-bvh/bvh"
	"github.com/downflux/go-bvh/container"
	"github.com/downflux/go-bvh/id"
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
		Tolerance: 1.1,
		PoolSize:  8,
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

	frame int  // DEBUG
}

func New(o O) *DB {
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

	for _, a := range db.projectiles {
		results = append(results, result{
			agent: a,
			v:     a.Velocity(),
		})
	}

	for _, a := range db.agents {
		v := vector.M{0, 0}
		// TODO(minkezhang): Investigate what happens if
		// we change this velocity to the nearest
		// 8-directional alignment.
		v.Copy(a.Velocity())

		// Check for collisions which the agent cares
		// about, e.g. care about squishability.
		for _, y := range db.neighbors(
			a.ID(),
			agent.AABB(
				a.Position(),
				a.Radius(),
			),
			agent.IsSquishableColliding,
		) {
			b := db.agents[y]

			// DEBUG
			colliding := vector.Magnitude(
				vector.Sub(
					a.Position(),
					b.Position(),
				),
			) < a.Radius() + b.Radius()

			if colliding {
				dp := vector.Sub(b.Position(), a.Position())
				fmt.Printf("DEBUG(main.go): Error(frame %v): %v and %v colliding! (d = %v)\n", db.frame, a.ID(), b.ID(), vector.Magnitude(dp))
				fmt.Printf("DEBUG(main.go): %v.P() = %v, %v.P() = %v, vin = %v\n", a.ID(), a.Position(), b.ID(), b.Position(), v)
			}

			agent.SetCollisionVelocity(a, b, v)

			if colliding {
				dp := vector.Sub(b.Position(), a.Position())
				c := vector.Dot(dp, v.V())
				fmt.Printf("DEBUG(main.go): dp = %v, vout = %v, dp * vout = %v > 0 = %v\n", dp, v, c, c > 0)
			}
		}

		results = append(results, result{
			agent: a,
			v:     v.V(),
		})
	}

	return results
}

// Tick advances the world by one tick. During this execution, agents must not
// be modified by the user.
func (db *DB) Tick(d time.Duration) {
	db.bvhL.Lock()
	defer db.bvhL.Unlock()

	t := float64(d) / float64(time.Second)

	for _, r := range db.generate() {
		agent.SetVelocity(r.agent, r.v.M())

		// N.B.: The velocity can be further reduced to
		// zero here due to the physical limitations of
		// the agent.
		h := polar.V{1, r.agent.Heading().Theta()}
		agent.SetHeading(r.agent, d, r.v.M(), h.M())

		r.agent.Position().M().Add(vector.Scale(t, r.v))
		r.agent.Heading().M().Copy(h)
	}

	for x, a := range db.agents {
		db.bvh.Update(x, agent.AABB(a.Position(), a.Radius()))
	}

	db.frame += 1
}

type result struct {
	agent *agent.A
	v     vector.V
}
