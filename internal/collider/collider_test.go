package collider

import (
	"testing"

	"github.com/downflux/go-collider/agent"
	"github.com/downflux/go-collider/mask"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/2d/vector/polar"
)

func TestIsColliding(t *testing.T) {
	type config struct {
		name string
		a    *agent.A
		b    *agent.A
		want bool
	}

	configs := []config{
		func() config {
			a := agent.New(agent.O{
				Heading:        polar.V{1, 0},
				TargetVelocity: vector.V{0, 0},
				Position:       vector.V{1, 1},
			})
			agent.SetID(a, 1)
			return config{
				name: "NoCollide/SelfID",
				a:    a,
				b:    a,
				want: false,
			}
		}(),
		func() config {
			a := agent.New(agent.O{
				Position:       vector.V{1, 1},
				Radius:         1,
				TargetVelocity: vector.V{0, 0},
				Heading:        polar.V{1, 0},
			})
			b := agent.New(agent.O{
				Position:       vector.V{1, 1},
				Radius:         1,
				TargetVelocity: vector.V{0, 0},
				Heading:        polar.V{1, 0},
				Mask:           mask.MSizeProjectile,
			})
			agent.SetID(a, 1)
			agent.SetID(b, 2)
			return config{
				name: "NoCollide/Projectile",
				a:    a,
				b:    b,
				want: false,
			}
		}(),
		func() config {
			a := agent.New(agent.O{
				Position:       vector.V{1, 1},
				Radius:         1,
				TargetVelocity: vector.V{0, 0},
				Heading:        polar.V{1, 0},
				Mask:           mask.MSizeSmall | mask.MTerrainAir | mask.MTerrainAccessibleAir,
			})
			b := agent.New(agent.O{
				Position:       vector.V{1, 1},
				Radius:         1,
				TargetVelocity: vector.V{0, 0},
				Heading:        polar.V{1, 0},
				Mask:           mask.MSizeSmall | mask.MTerrainLand | mask.MTerrainAccessibleLand,
			})
			agent.SetID(a, 1)
			agent.SetID(b, 2)
			return config{
				name: "NoCollide/ExclusiveAir",
				a:    a,
				b:    b,
				want: false,
			}
		}(),
		func() config {
			a := agent.New(agent.O{
				Position:       vector.V{1, 1},
				Radius:         1,
				TargetVelocity: vector.V{0, 0},
				Heading:        polar.V{1, 0},
				Mask:           mask.MSizeSmall | mask.MTerrainAir | mask.MTerrainAccessibleAir,
			})
			b := agent.New(agent.O{
				Position:       vector.V{1, 1},
				Radius:         1,
				TargetVelocity: vector.V{0, 0},
				Heading:        polar.V{1, 0},
				Mask:           mask.MSizeSmall | mask.MTerrainAir | mask.MTerrainAccessibleAir,
			})
			agent.SetID(a, 1)
			agent.SetID(b, 2)
			return config{
				name: "Collide/BothAir",
				a:    a,
				b:    b,
				want: true,
			}
		}(),
	}

	for _, c := range configs {
		t.Run(c.name, func(t *testing.T) {
			if got := IsColliding(c.a, c.b); got != c.want {
				t.Errorf("IsColliding() = %v, want = %v", got, c.want)
			}
		})
	}
}
