package db

import (
	"fmt"
	"sync"
	"time"

	"github.com/downflux/game-db/agent"
	"github.com/downflux/go-bvh/bvh"
	"github.com/downflux/go-bvh/id"
	"github.com/downflux/go-geometry/nd/hyperrectangle"
	"github.com/downflux/go-geometry/nd/vector"

	v2d "github.com/downflux/go-geometry/2d/vector"
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

	cacheIsValid   bool
	aabbCache      map[id.ID]hyperrectangle.M
	neighborsCache map[id.ID][]id.ID

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
	if db.cacheIsValid {
		for _, n := range db.neighborsCache[x] {
			nNeighbors := db.neighborsCache[n]
			for i, m := range nNeighbors {
				if m == x {
					nNeighbors[i] = nNeighbors[len(nNeighbors)-1]
					nNeighbors = nNeighbors[:len(nNeighbors)-1]
					break
				}
			}
		}
	}

	delete(db.agents, x)
	delete(db.neighborsCache, x)
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

	if err := db.bvh.Insert(x, agent.LeafAABB(db.agents[x])); err != nil {
		panic(fmt.Sprintf("cannot insert agent: %v", err))
	}

	if db.cacheIsValid {
		db.neighborsCache[x] = db.bvh.BroadPhase(agent.BroadPhaseAABB(db.agents[x]))
		for _, n := range db.neighborsCache[x] {
			db.neighborsCache[n] = append(db.neighborsCache[n], x)
		}
	}

	return a
}

// Neighbors returns a list of neighboring agents to the input.
func (db *DB) Neighbors(x id.ID) []id.ID {
	// Use the cache if every agent is fully updated. Otherwise recompute.
	if db.cacheIsValid {
		neighbors := make([]id.ID, len(db.neighborsCache[x]))
		copy(neighbors, db.neighborsCache[x])
		return neighbors
	}
	db.neighborsCache[x] = db.narrowPhase(
		x,
		db.bvh.BroadPhase(agent.BroadPhaseAABB(db.agents[x])),
	)
	return db.neighborsCache[x]
}

func (db *DB) narrowPhase(x id.ID, broadphase []id.ID) []id.ID {
	collisions := make([]id.ID, 0, len(broadphase))
	n := db.agents[x]
	for _, y := range broadphase {
		m := db.agents[y]
		if agent.IsColliding(n, m) {
			collisions = append(collisions, y)
		}
	}
	return collisions
}

func (db *DB) updateNeighbors() {
	if db.cacheIsValid {
		return
	}

	// Detect collisions and pre-processing.
	ch := make(chan *agent.A, 256)
	go func() {
		for _, n := range db.agents {
			ch <- n
		}
		close(ch)
	}()

	var wg sync.WaitGroup
	for i := 0; i < db.poolSize; i++ {
		wg.Add(1)
		go func() {
			defer wg.Done()

			buf := hyperrectangle.New(
				vector.V{0, 0},
				vector.V{0, 0},
			).M()

			for a := range ch {
				agent.SetBroadPhaseAABB(a, buf)
				db.neighborsCache[a.ID()] = db.narrowPhase(a.ID(), db.bvh.BroadPhase(buf.R()))
			}
		}()
	}
	wg.Wait()

	db.cacheIsValid = true
}

func (db *DB) SetVelocity(x id.ID, v v2d.V) {
	// TODO(minkezhang): Figure out if this velocity is actually usable --
	// check neighboring collisions and rescale the velocity vector as
	// appropriate.
	db.agents[x].Velocity().M().Copy(v)

	// TODO(minkezhang): If agent.BroadPhaseAABB re-calculates the size of
	// the bounding box with velocity, we need to update neighbor cache.
}

func (db *DB) Tick(d time.Duration) {
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
				a.Position().M().Add(v2d.Scale(float64(t), a.Velocity()))
			}
		}()
	}()

	db.cacheIsValid = false

	db.updateNeighbors()
}
