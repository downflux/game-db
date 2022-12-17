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
	DefaultO O = O{
		LeafSize:  8,
		Tolerance: 2.0,
		PoolSize:  8,
	}
)

type O struct {
	LeafSize  int
	Tolerance float64
	PoolSize  int
}

type DB struct {
	agents map[id.ID]*agent.A

	aabbCache map[id.ID]hyperrectangle.M

	bvh *bvh.T

	poolSize int
	counter  uint64
}

func New(o O) *DB {
	return &DB{
		agents:    make(map[id.ID]*agent.A, 1024),
		aabbCache: make(map[id.ID]hyperrectangle.M, 1024),
		bvh: bvh.New(bvh.O{
			K:         2,
			LeafSize:  o.LeafSize,
			Tolerance: o.Tolerance,
		}),
		poolSize: o.PoolSize,
	}
}

func (db *DB) Delete(x id.ID) {
	delete(db.agents, x)
	if err := db.bvh.Remove(x); err != nil {
		panic(fmt.Sprintf("cannot delete agent: %v", err))
	}
}

func (db *DB) Insert(o agent.O) *agent.A {
	x := id.ID(db.counter)
	db.counter++

	a := agent.New(o)
	agent.SetID(a, x)

	db.agents[x] = a

	if err := db.bvh.Insert(x, agent.AABB(a.Position(), a.Radius())); err != nil {
		panic(fmt.Sprintf("cannot insert agent: %v", err))
	}

	return a
}

// Neighbors returns a list of neighboring agents to the input.
func (db *DB) Neighbors(x id.ID, q hyperrectangle.R, filter func(a, b *agent.A) bool) []id.ID {
	a := db.agents[x]

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

func (db *DB) generateVelocity() {
	ch := make(chan *agent.A, 256)
	go func() {
		for _, a := range db.agents {
			ch <- a
		}
		close(ch)
	}()

	var wg sync.WaitGroup
	go func() {
		wg.Add(1)
		go func() {
			defer wg.Done()
			for a := range ch {
				r := 2 * a.Radius()
				for _, y := range db.Neighbors(a.ID(), agent.AABB(a.Position(), r), agent.IsColliding) {
					b := db.agents[y]

					// TODO(minkezhang): Calculate new
					// velocities based on elastic or
					// inelastic collisions.
					a.Velocity().M().Copy(a.Velocity())
					b.Velocity().M().Copy(b.Velocity())
				}
			}
		}()
	}()
}

func (db *DB) Tick(d time.Duration) {
	db.generateVelocity()

	t := d / time.Second

	ch := make(chan *agent.A, 256)
	go func() {
		for _, a := range db.agents {
			ch <- a
		}
		close(ch)
	}()

	var wg sync.WaitGroup
	go func() {
		wg.Add(1)
		go func() {
			defer wg.Done()
			for a := range ch {
				a.Position().M().Add(vector.Scale(float64(t), a.Velocity()))
			}
		}()
	}()
}
