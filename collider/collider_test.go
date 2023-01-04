package collider

import (
	"fmt"
	"math"
	"math/rand"
	"testing"
	"time"

	"github.com/downflux/go-bvh/id"
	"github.com/downflux/go-database/agent"
	"github.com/downflux/go-database/database"
	"github.com/downflux/go-database/feature"
	"github.com/downflux/go-database/flags"
	"github.com/downflux/go-database/projectile"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/2d/vector/polar"
)

const (
	R = 0.5
)

func rn(min, max float64) float64 { return min + rand.Float64()*(max-min) }
func rv(min, max float64) vector.V {
	return vector.V{
		rn(min, max),
		rn(min, max),
	}
}

func TestTick(t *testing.T) {
	type config struct {
		name     string
		collider *C
		db       *database.DB
		d        time.Duration
		want     map[id.ID]vector.V
	}

	configs := []config{
		func() config {
			db := database.New(database.DefaultO)
			collider := New(O{DB: db, PoolSize: DefaultO.PoolSize})
			a := db.InsertAgent(agent.O{
				Position:       vector.V{0, 0},
				TargetVelocity: vector.V{1, 1},
				Velocity:       vector.V{1, 1},
				Heading:        polar.V{1, math.Pi / 4},
				Radius:         1,
				Flags:          flags.FSizeSmall,
			})
			b := db.InsertAgent(agent.O{
				Position:       vector.V{0, 1},
				TargetVelocity: vector.V{0, -1},
				Velocity:       vector.V{0, -1},
				Heading:        polar.V{1, 3 * math.Pi / 2},
				Radius:         1,
				Flags:          flags.FSizeSmall,
			})
			c := db.InsertAgent(agent.O{
				Position:       vector.V{0, -1},
				TargetVelocity: vector.V{0, 1},
				Velocity:       vector.V{0, 1},
				Heading:        polar.V{1, math.Pi / 2},
				Radius:         1,
				Flags:          flags.FSizeSmall,
			})
			d := db.InsertAgent(agent.O{
				Position:       vector.V{1, 0},
				TargetVelocity: vector.V{-1, 0},
				Velocity:       vector.V{-1, 0},
				Heading:        polar.V{1, math.Pi},
				Radius:         1,
				Flags:          flags.FSizeSmall,
			})
			e := db.InsertAgent(agent.O{
				Position:       vector.V{-1, 0},
				TargetVelocity: vector.V{1, 0},
				Velocity:       vector.V{1, 0},
				Heading:        polar.V{1, 2 * math.Pi},
				Radius:         1,
				Flags:          flags.FSizeSmall,
			})
			return config{
				name:     "Stuck",
				collider: collider,
				db:       db,
				d:        100 * time.Millisecond,
				want: map[id.ID]vector.V{
					a.ID(): vector.V{0, 0},
					b.ID(): vector.V{0, 1},
					c.ID(): vector.V{0, -1},
					d.ID(): vector.V{1, 0},
					e.ID(): vector.V{-1, 0},
				},
			}
		}(),
		func() config {
			db := database.New(database.DefaultO)
			collider := New(O{DB: db, PoolSize: DefaultO.PoolSize})
			a := db.InsertAgent(agent.O{
				Position:       vector.V{10, 10},
				TargetVelocity: vector.V{1, 1},
				Velocity:       vector.V{1, 1},
				MaxVelocity:    math.Sqrt(2),
				Heading:        polar.V{1, math.Pi / 4},
				Radius:         1,
				Flags:          flags.FSizeSmall,
			})
			return config{
				name:     "Trivial",
				collider: collider,
				db:       db,
				d:        100 * time.Millisecond,
				want: map[id.ID]vector.V{
					a.ID(): vector.V{10.1, 10.1},
				},
			}
		}(),
		func() config {
			db := database.New(database.DefaultO)
			collider := New(O{DB: db, PoolSize: DefaultO.PoolSize})
			a := db.InsertAgent(agent.O{
				Position:       vector.V{10, 10},
				TargetVelocity: vector.V{0, 1},
				Velocity:       vector.V{0, 1},
				MaxVelocity:    1,
				Heading:        polar.V{1, math.Pi / 2},
				Radius:         1,
				Flags:          flags.FSizeSmall,
			})
			b := db.InsertAgent(agent.O{
				Position:       vector.V{10, 12},
				TargetVelocity: vector.V{0, -1},
				Velocity:       vector.V{0, -1},
				MaxVelocity:    1,
				Heading:        polar.V{1, 3 * math.Pi / 2},
				Radius:         1,
				Flags:          flags.FSizeSmall,
			})
			return config{
				name:     "Collision",
				collider: collider,
				db:       db,
				d:        100 * time.Millisecond,
				want: map[id.ID]vector.V{
					a.ID(): vector.V{10, 10},
					b.ID(): vector.V{10, 12},
				},
			}
		}(),
		func() config {
			db := database.New(database.DefaultO)
			collider := New(O{DB: db, PoolSize: DefaultO.PoolSize})
			a := db.InsertAgent(agent.O{
				Position:       vector.V{10, 10},
				TargetVelocity: vector.V{0, 1},
				Velocity:       vector.V{0, 1},
				MaxVelocity:    1,
				Heading:        polar.V{1, math.Pi / 2},
				Radius:         1,
				Flags:          flags.FSizeSmall,
			})
			db.InsertProjectile(projectile.O{
				Position:       vector.V{10, 12},
				TargetVelocity: vector.V{0, -1},
				Velocity:       vector.V{0, -1},
				Heading:        polar.V{1, 3 * math.Pi / 2},
				Radius:         1,
				Flags:          flags.FSizeProjectile,
			})
			return config{
				name:     "Collision/IgnoreProjectile",
				collider: collider,
				db:       db,
				d:        100 * time.Millisecond,
				want: map[id.ID]vector.V{
					a.ID(): vector.V{10, 10.1},
				},
			}
		}(),
		func() config {
			db := database.New(database.DefaultO)
			collider := New(O{DB: db, PoolSize: DefaultO.PoolSize})
			a := db.InsertAgent(agent.O{
				Position:           vector.V{60.0040783686527, 80.40391843262739},
				TargetVelocity:     vector.V{10, 10},
				Velocity:           vector.V{10, 10},
				Heading:            polar.V{1, 1.550798992821703},
				Radius:             10,
				MaxVelocity:        100,
				MaxAngularVelocity: math.Pi / 2,
				MaxAcceleration:    5,
				Flags:              flags.FSizeSmall,
			})
			db.InsertFeature(feature.O{
				Min: vector.V{70, 20},
				Max: vector.V{90, 80},
			})
			// Ensure agent can still move when right at the corner.
			// This case exposes a floating point error when
			// checking for strict velocity feature collisions that
			// needs to be taken into account.
			return config{
				name:     "Experimental/CutCorner",
				collider: collider,
				db:       db,
				d:        20 * time.Millisecond,
				want: map[id.ID]vector.V{
					a.ID(): vector.V{
						60.01247289297099,
						80.61166088862839,
					},
				},
			}
		}(),
	}

	for _, c := range configs {
		t.Run(c.name, func(t *testing.T) {
			c.collider.Tick(c.d)
			for x, want := range c.want {
				got := c.db.GetAgentOrDie(x).Position()
				if !vector.Within(got, want) {
					t.Errorf("Position() = %v, want = %v", got, want)
				}
			}
		})
	}
}

func BenchmarkTick(b *testing.B) {
	type config struct {
		name     string
		n        int
		coverage float64
	}

	configs := []config{}
	for _, n := range []int{1e3, 1e4, 1e5} {
		for _, coverage := range []float64{0.01, 0.05, 0.1} {
			configs = append(configs, config{
				name:     fmt.Sprintf("N=%v/ρ=%v", n, coverage),
				n:        n,
				coverage: coverage,
			})
		}
	}

	for _, c := range configs {
		b.Run(c.name, func(b *testing.B) {
			b.StopTimer()
			area := float64(c.n) * math.Pi * R * R / c.coverage
			min := 0.0
			max := math.Sqrt(area)

			db := database.New(database.DefaultO)
			collider := New(O{DB: db, PoolSize: DefaultO.PoolSize})
			for i := 0; i < c.n; i++ {
				db.InsertAgent(agent.O{
					Radius:             R,
					Position:           rv(min, max),
					TargetVelocity:     rv(-1, 1),
					Velocity:           rv(-1, 1),
					MaxVelocity:        60,
					MaxAcceleration:    10,
					MaxAngularVelocity: math.Pi / 4,
					Heading:            polar.V{1, 0},
					Flags:              flags.FSizeSmall,
				})
			}

			// Add world borders.
			// Add xmin border.
			db.InsertFeature(feature.O{
				Min: vector.V{min - 1, min - 1},
				Max: vector.V{min, max + 1},
			})
			// Add xmax border.
			db.InsertFeature(feature.O{
				Min: vector.V{max, min - 1},
				Max: vector.V{max + 1, max + 1},
			})
			// Add ymin border.
			db.InsertFeature(feature.O{
				Min: vector.V{min, min - 1},
				Max: vector.V{max, min},
			})
			// Add ymax border.
			db.InsertFeature(feature.O{
				Min: vector.V{min, max},
				Max: vector.V{max, max + 1},
			})

			b.StartTimer()
			for i := 0; i < b.N; i++ {
				collider.Tick(33 * time.Millisecond)
			}
		})
	}
}
