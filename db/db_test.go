package db

import (
	"fmt"
	"math"
	"math/rand"
	"testing"
	"time"

	"github.com/downflux/game-db/agent"
	"github.com/downflux/game-db/agent/mask"
	"github.com/downflux/go-bvh/id"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/2d/vector/polar"
	"github.com/downflux/go-geometry/nd/hyperrectangle"
	"github.com/google/go-cmp/cmp"
	"github.com/google/go-cmp/cmp/cmpopts"
)

const (
	N   = 1000
	R   = 0.5
	Min = 0
	Max = 300
)

func rn(min, max float64) float64 { return min + rand.Float64()*(max-min) }
func rv(min, max float64) vector.V {
	return vector.V{
		rn(min, max),
		rn(min, max),
	}
}

func TestNeighbors(t *testing.T) {
	type config struct {
		name string
		db   *DB
		x    id.ID
		q    hyperrectangle.R
		want []id.ID
	}

	configs := []config{
		func() config {
			db := New(DefaultO)
			a := db.Insert(agent.O{
				Position: vector.V{10, 10},
				Velocity: vector.V{0, 0},
				Heading:  polar.V{1, 0},
				Radius:   1,
				Mask:     mask.MSizeSmall,
			})
			return config{
				name: "Exclude/Self",
				db:   db,
				x:    a.ID(),
				q:    agent.AABB(a.Position(), a.Radius()),
				want: []id.ID{},
			}
		}(),
		func() config {
			db := New(DefaultO)
			a := db.Insert(agent.O{
				Position: vector.V{10, 10},
				Velocity: vector.V{0, 0},
				Heading:  polar.V{1, 0},
				Radius:   1,
				Mask:     mask.MSizeSmall,
			})

			b := db.Insert(agent.O{
				Position: a.Position(),
				Velocity: a.Velocity(),
				Heading:  a.Heading(),
				Radius:   a.Radius(),
				Mask:     mask.MSizeSmall,
			})
			db.Insert(agent.O{
				Position: a.Position(),
				Velocity: a.Velocity(),
				Heading:  a.Heading(),
				Radius:   a.Radius(),
				Mask:     mask.MSizeProjectile,
			})
			return config{
				name: "Exclude/Projectiles",
				db:   db,
				x:    a.ID(),
				q:    agent.AABB(a.Position(), a.Radius()),
				want: []id.ID{b.ID()},
			}
		}(),
		func() config {
			db := New(DefaultO)
			a := db.Insert(agent.O{
				Position: vector.V{10, 10},
				Velocity: vector.V{0, 0},
				Heading:  polar.V{1, 0},
				Radius:   1,
				Mask:     mask.MSizeMedium,
			})
			b := db.Insert(agent.O{
				Position: a.Position(),
				Velocity: a.Velocity(),
				Heading:  a.Heading(),
				Radius:   a.Radius(),
				Mask:     mask.MSizeLarge,
			})
			db.Insert(agent.O{
				Position: a.Position(),
				Velocity: a.Velocity(),
				Heading:  a.Heading(),
				Radius:   a.Radius(),
				Mask:     mask.MSizeSmall,
			})
			return config{
				name: "Exclude/Squishable",
				db:   db,
				x:    a.ID(),
				q:    agent.AABB(a.Position(), a.Radius()),
				want: []id.ID{b.ID()},
			}
		}(),
	}

	for _, c := range configs {
		t.Run(c.name, func(t *testing.T) {
			got := c.db.neighbors(c.x, c.q, agent.IsSquishableColliding)
			if diff := cmp.Diff(c.want, got, cmpopts.SortSlices(func(a, b id.ID) bool { return a < b })); diff != "" {
				t.Errorf("neighbors() mismatch (-want +got):\n%v", diff)
			}
		})
	}
}

func TestTick(t *testing.T) {
	type config struct {
		name string
		db   *DB
		d    time.Duration
		want map[id.ID]vector.V
	}

	configs := []config{
		func() config {
			db := New(DefaultO)
			a := db.Insert(agent.O{
				Position:    vector.V{10, 10},
				Velocity:    vector.V{1, 1},
				MaxVelocity: math.Sqrt(2),
				Heading:     polar.V{1, math.Pi / 4},
				Radius:      1,
				Mask:        mask.MSizeSmall,
			})
			return config{
				name: "Trivial",
				db:   db,
				d:    100 * time.Millisecond,
				want: map[id.ID]vector.V{
					a.ID(): vector.V{10.1, 10.1},
				},
			}
		}(),
		func() config {
			db := New(DefaultO)
			a := db.Insert(agent.O{
				Position:    vector.V{10, 10},
				Velocity:    vector.V{0, 1},
				MaxVelocity: 1,
				Heading:     polar.V{1, math.Pi / 2},
				Radius:      1,
				Mask:        mask.MSizeSmall,
			})
			b := db.Insert(agent.O{
				Position:    vector.V{10, 12},
				Velocity:    vector.V{0, -1},
				MaxVelocity: 1,
				Heading:     polar.V{1, -math.Pi / 2},
				Radius:      1,
				Mask:        mask.MSizeSmall,
			})
			return config{
				name: "Collision",
				db:   db,
				d:    100 * time.Millisecond,
				want: map[id.ID]vector.V{
					a.ID(): vector.V{10, 10},
					b.ID(): vector.V{10, 12},
				},
			}
		}(),
		func() config {
			db := New(DefaultO)
			a := db.Insert(agent.O{
				Position:    vector.V{10, 10},
				Velocity:    vector.V{0, 1},
				MaxVelocity: 1,
				Heading:     polar.V{1, math.Pi / 2},
				Radius:      1,
				Mask:        mask.MSizeSmall,
			})
			b := db.Insert(agent.O{
				Position:    vector.V{10, 12},
				Velocity:    vector.V{0, -1},
				MaxVelocity: 1,
				Heading:     polar.V{1, -math.Pi / 2},
				Radius:      1,
				Mask:        mask.MSizeProjectile,
			})
			return config{
				name: "Collision/IgnoreProjectile",
				db:   db,
				d:    100 * time.Millisecond,
				want: map[id.ID]vector.V{
					a.ID(): vector.V{10, 10.1},
					b.ID(): vector.V{10, 11.9},
				},
			}
		}(),
	}

	for _, c := range configs {
		t.Run(c.name, func(t *testing.T) {
			c.db.Tick(c.d)
			for x, want := range c.want {
				got := c.db.GetOrDie(x).Position()
				if !vector.Within(got, want) {
					t.Errorf("Position() = %v, want = %v", got, want)
				}
			}
		})
	}
}

func BenchmarkTick(b *testing.B) {
	db := New(DefaultO)
	for i := 0; i < N; i++ {
		db.Insert(agent.O{
			Radius:   R,
			Position: rv(Min, Max),
			Velocity: rv(-1, 1),
			Heading:  polar.V{1, 0},
			Mask:     mask.MSizeSmall,
		})
	}

	b.Run(fmt.Sprintf("N=%v", N), func(b *testing.B) {
		for i := 0; i < b.N; i++ {
			db.Tick(33 * time.Millisecond)
		}
	})
}
