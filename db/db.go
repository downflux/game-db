package db

import (
	"fmt"
	"sync"
	"time"

	"github.com/downflux/game-db/agent"
	"github.com/downflux/go-bvh/bvh"
	"github.com/downflux/go-bvh/id"
	"github.com/downflux/go-geometry/2d/vector"
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

	bvh *bvh.T

	poolSize int
	counter  uint64
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
	a, ok := db.agents[x]
	if !ok {
		a = db.projectiles[x]
	}

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
	x := id.ID(db.counter)
	db.counter++

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
	a, ok := db.agents[x]
	if !ok {
		a = db.projectiles[x]
	}

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

func (db *DB) SetPosition(x id.ID, v vector.V) { db.agents[x].Position().M().Copy(v) }
func (db *DB) SetVelocity(x id.ID, v vector.V) { db.agents[x].Velocity().M().Copy(v) }

type result struct {
	agent *agent.A
	v     vector.V
}

func (db *DB) generate(out chan<- result) {
	ch := make(chan *agent.A, 256)

	go func() {
		for _, a := range db.agents {
			ch <- a
		}
		close(ch)
	}()

	var wg sync.WaitGroup

	wg.Add(1)
	go func() {
		defer wg.Done()
		for _, a := range db.projectiles {
			out <- result{
				agent: a,
				v:     a.Velocity(),
			}
		}
	}()

	for i := 0; i < db.poolSize; i++ {
		wg.Add(1)
		go func() {
			defer wg.Done()
			for a := range ch {
				v := vector.M{0, 0}
				v.Copy(a.Velocity())

				// Check for only geometric collisions, i.e.
				// overlapping radii.
				for _, y := range db.Neighbors(
					a.ID(),
					agent.AABB(
						a.Position(),
						a.Radius(),
					),
					agent.IsSquishableColliding,
				) {
					agent.SetCollisionVelocity(a, db.agents[y], v)
				}

				out <- result{
					agent: a,
					v:     v.V(),
				}
			}
		}()
	}

	wg.Wait()
	close(out)
}

// Tick advances the world by one tick. During this execution, agents must not
// be modified by the user.
func (db *DB) Tick(d time.Duration) {
	out := make(chan result, 256)
	go db.generate(out)

	t := float64(d) / float64(time.Second)

	var wg sync.WaitGroup
	for i := 0; i < db.poolSize; i++ {
		wg.Add(1)
		go func() {
			defer wg.Done()
			for r := range out {
				r.agent.Position().M().Add(vector.Scale(t, r.v))
			}
		}()
	}
	wg.Wait()

	for x, a := range db.agents {
		db.bvh.Update(x, agent.AABB(a.Position(), a.Radius()))
	}
}
