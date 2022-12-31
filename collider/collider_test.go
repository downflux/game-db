package collider

import (
	"fmt"
	"math"
	"math/rand"
	"testing"
	"time"

	"github.com/downflux/go-bvh/id"
	"github.com/downflux/go-collider/agent"
	"github.com/downflux/go-collider/feature"
	"github.com/downflux/go-collider/internal/collider"
	"github.com/downflux/go-collider/mask"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/2d/vector/polar"
	"github.com/downflux/go-geometry/nd/hyperrectangle"
	"github.com/google/go-cmp/cmp"
	"github.com/google/go-cmp/cmp/cmpopts"
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

func TestQueryFeatures(t *testing.T) {
	type config struct {
		name     string
		collider *C
		x        id.ID
		q        hyperrectangle.R
		want     []id.ID
	}

	configs := []config{
		func() config {
			collider := New(DefaultO)
			a := collider.Insert(agent.O{
				Position:       vector.V{5, 15},
				TargetVelocity: vector.V{0, 0},
				Heading:        polar.V{1, 0},
				Radius:         5,
				Mask:           mask.MSizeSmall,
			})
			f := collider.InsertFeature(feature.O{
				Min: vector.V{10, 10},
				Max: vector.V{20, 20},
			})
			return config{
				name:     "Touch",
				collider: collider,
				x:        a.ID(),
				q:        agent.AABB(a.Position(), a.Radius()),
				want:     []id.ID{f.ID()},
			}
		}(),
		func() config {
			collider := New(DefaultO)
			a := collider.Insert(agent.O{
				Position:       vector.V{9, 10},
				TargetVelocity: vector.V{0, 0},
				Heading:        polar.V{1, 0},
				Radius:         1,
				Mask:           mask.MSizeSmall,
			})
			f := collider.InsertFeature(feature.O{
				Min: vector.V{10, 10},
				Max: vector.V{20, 20},
			})
			return config{
				name:     "Slide",
				collider: collider,
				x:        a.ID(),
				q:        agent.AABB(a.Position(), a.Radius()),
				want:     []id.ID{f.ID()},
			}
		}(),
		func() config {
			collider := New(DefaultO)
			a := collider.Insert(agent.O{
				Position:       vector.V{5, 5},
				TargetVelocity: vector.V{0, 0},
				Heading:        polar.V{1, 0},
				Radius:         5,
				Mask:           mask.MSizeSmall,
			})
			f := collider.InsertFeature(feature.O{
				Min: vector.V{10, 10},
				Max: vector.V{20, 20},
			})
			return config{
				name:     "Corner",
				collider: collider,
				x:        a.ID(),
				q:        agent.AABB(a.Position(), a.Radius()),
				want:     []id.ID{f.ID()},
			}
		}(),
	}

	for _, c := range configs {
		t.Run(c.name, func(t *testing.T) {
			got := c.collider.QueryFeatures(c.q, func(f *feature.F) bool {
				return collider.IsCollidingFeature(c.collider.getOrDie(c.x), f)
			})
			if diff := cmp.Diff(c.want, got, cmpopts.SortSlices(func(a, b id.ID) bool { return a < b })); diff != "" {
				t.Errorf("QueryFeatures() mismatch (-want +got):\n%v", diff)
			}
		})
	}
}

func TestQuery(t *testing.T) {
	type config struct {
		name     string
		collider *C
		x        id.ID
		q        hyperrectangle.R
		want     []id.ID
	}

	configs := []config{
		func() config {
			collider := New(DefaultO)
			a := collider.Insert(agent.O{
				Position:       vector.V{10, 10},
				TargetVelocity: vector.V{0, 0},
				Heading:        polar.V{1, 0},
				Radius:         1,
				Mask:           mask.MSizeSmall,
			})
			return config{
				name:     "Exclude/Self",
				collider: collider,
				x:        a.ID(),
				q:        agent.AABB(a.Position(), a.Radius()),
				want:     []id.ID{},
			}
		}(),
		func() config {
			collider := New(DefaultO)
			a := collider.Insert(agent.O{
				Position:       vector.V{10, 10},
				TargetVelocity: vector.V{0, 0},
				Heading:        polar.V{1, 0},
				Radius:         1,
				Mask:           mask.MSizeSmall,
			})

			b := collider.Insert(agent.O{
				Position:       a.Position(),
				TargetVelocity: a.TargetVelocity(),
				Heading:        a.Heading(),
				Radius:         a.Radius(),
				Mask:           mask.MSizeSmall,
			})
			collider.Insert(agent.O{
				Position:       a.Position(),
				TargetVelocity: a.TargetVelocity(),
				Heading:        a.Heading(),
				Radius:         a.Radius(),
				Mask:           mask.MSizeProjectile,
			})
			return config{
				name:     "Exclude/Projectiles",
				collider: collider,
				x:        a.ID(),
				q:        agent.AABB(a.Position(), a.Radius()),
				want:     []id.ID{b.ID()},
			}
		}(),
		func() config {
			collider := New(DefaultO)
			a := collider.Insert(agent.O{
				Position:       vector.V{10, 10},
				TargetVelocity: vector.V{0, 0},
				Heading:        polar.V{1, 0},
				Radius:         1,
				Mask:           mask.MSizeMedium,
			})
			b := collider.Insert(agent.O{
				Position:       a.Position(),
				TargetVelocity: a.TargetVelocity(),
				Heading:        a.Heading(),
				Radius:         a.Radius(),
				Mask:           mask.MSizeLarge,
			})
			collider.Insert(agent.O{
				Position:       a.Position(),
				TargetVelocity: a.TargetVelocity(),
				Heading:        a.Heading(),
				Radius:         a.Radius(),
				Mask:           mask.MSizeSmall,
			})
			return config{
				name:     "Exclude/Squishable",
				collider: collider,
				x:        a.ID(),
				q:        agent.AABB(a.Position(), a.Radius()),
				want:     []id.ID{b.ID()},
			}
		}(),
	}

	for _, c := range configs {
		t.Run(c.name, func(t *testing.T) {
			got := c.collider.Query(c.q, func(b *agent.A) bool {
				return collider.IsSquishableColliding(c.collider.getOrDie(c.x), b)
			})
			if diff := cmp.Diff(c.want, got, cmpopts.SortSlices(func(a, b id.ID) bool { return a < b })); diff != "" {
				t.Errorf("query() mismatch (-want +got):\n%v", diff)
			}
		})
	}
}

func TestTick(t *testing.T) {
	type config struct {
		name     string
		collider *C
		d        time.Duration
		want     map[id.ID]vector.V
	}

	configs := []config{
		func() config {
			collider := New(DefaultO)
			a := collider.Insert(agent.O{
				Position:       vector.V{0, 0},
				TargetVelocity: vector.V{1, 1},
				Heading:        polar.V{1, math.Pi / 4},
				Radius:         1,
				Mask:           mask.MSizeSmall,
			})
			b := collider.Insert(agent.O{
				Position:       vector.V{0, 1},
				TargetVelocity: vector.V{0, -1},
				Heading:        polar.V{1, 3 * math.Pi / 2},
				Radius:         1,
				Mask:           mask.MSizeSmall,
			})
			c := collider.Insert(agent.O{
				Position:       vector.V{0, -1},
				TargetVelocity: vector.V{0, 1},
				Heading:        polar.V{1, math.Pi / 2},
				Radius:         1,
				Mask:           mask.MSizeSmall,
			})
			d := collider.Insert(agent.O{
				Position:       vector.V{1, 0},
				TargetVelocity: vector.V{-1, 0},
				Heading:        polar.V{1, math.Pi},
				Radius:         1,
				Mask:           mask.MSizeSmall,
			})
			e := collider.Insert(agent.O{
				Position:       vector.V{-1, 0},
				TargetVelocity: vector.V{1, 0},
				Heading:        polar.V{1, 2 * math.Pi},
				Radius:         1,
				Mask:           mask.MSizeSmall,
			})
			return config{
				name:     "Stuck",
				collider: collider,
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
			collider := New(DefaultO)
			a := collider.Insert(agent.O{
				Position:       vector.V{10, 10},
				TargetVelocity: vector.V{1, 1},
				MaxVelocity:    math.Sqrt(2),
				Heading:        polar.V{1, math.Pi / 4},
				Radius:         1,
				Mask:           mask.MSizeSmall,
			})
			return config{
				name:     "Trivial",
				collider: collider,
				d:        100 * time.Millisecond,
				want: map[id.ID]vector.V{
					a.ID(): vector.V{10.1, 10.1},
				},
			}
		}(),
		func() config {
			collider := New(DefaultO)
			a := collider.Insert(agent.O{
				Position:       vector.V{10, 10},
				TargetVelocity: vector.V{0, 1},
				MaxVelocity:    1,
				Heading:        polar.V{1, math.Pi / 2},
				Radius:         1,
				Mask:           mask.MSizeSmall,
			})
			b := collider.Insert(agent.O{
				Position:       vector.V{10, 12},
				TargetVelocity: vector.V{0, -1},
				MaxVelocity:    1,
				Heading:        polar.V{1, 3 * math.Pi / 2},
				Radius:         1,
				Mask:           mask.MSizeSmall,
			})
			return config{
				name:     "Collision",
				collider: collider,
				d:        100 * time.Millisecond,
				want: map[id.ID]vector.V{
					a.ID(): vector.V{10, 10},
					b.ID(): vector.V{10, 12},
				},
			}
		}(),
		func() config {
			collider := New(DefaultO)
			a := collider.Insert(agent.O{
				Position:       vector.V{10, 10},
				TargetVelocity: vector.V{0, 1},
				MaxVelocity:    1,
				Heading:        polar.V{1, math.Pi / 2},
				Radius:         1,
				Mask:           mask.MSizeSmall,
			})
			b := collider.Insert(agent.O{
				Position:       vector.V{10, 12},
				TargetVelocity: vector.V{0, -1},
				MaxVelocity:    1,
				Heading:        polar.V{1, 3 * math.Pi / 2},
				Radius:         1,
				Mask:           mask.MSizeProjectile,
			})
			return config{
				name:     "Collision/IgnoreProjectile",
				collider: collider,
				d:        100 * time.Millisecond,
				want: map[id.ID]vector.V{
					a.ID(): vector.V{10, 10.1},
					b.ID(): vector.V{10, 11.9},
				},
			}
		}(),
		func() config {
			collider := New(DefaultO)
			a := collider.Insert(agent.O{
				Position:           vector.V{60.0040783686527, 80.40391843262739},
				TargetVelocity:     vector.V{10, 10},
				Heading:            polar.V{1, 1.550798992821703},
				Radius:             10,
				MaxVelocity:        100,
				MaxAngularVelocity: math.Pi / 2,
				MaxAcceleration:    5,
				Mask:               mask.MSizeSmall,
			})
			collider.InsertFeature(feature.O{
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
				got := c.collider.GetOrDie(x).Position()
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

			collider := New(DefaultO)
			for i := 0; i < c.n; i++ {
				collider.Insert(agent.O{
					Radius:             R,
					Position:           rv(min, max),
					TargetVelocity:     rv(-1, 1),
					MaxVelocity:        60,
					MaxAcceleration:    10,
					MaxAngularVelocity: math.Pi / 4,
					Heading:            polar.V{1, 0},
					Mask:               mask.MSizeSmall,
				})
			}

			// Add world borders.
			// Add xmin border.
			collider.InsertFeature(feature.O{
				Min: vector.V{min - 1, min - 1},
				Max: vector.V{min, max + 1},
			})
			// Add xmax border.
			collider.InsertFeature(feature.O{
				Min: vector.V{max, min - 1},
				Max: vector.V{max + 1, max + 1},
			})
			// Add ymin border.
			collider.InsertFeature(feature.O{
				Min: vector.V{min, min - 1},
				Max: vector.V{max, min},
			})
			// Add ymax border.
			collider.InsertFeature(feature.O{
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
