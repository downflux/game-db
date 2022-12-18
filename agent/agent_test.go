package agent

import (
	"testing"

	"github.com/downflux/go-geometry/2d/vector"
)

func TestSetCollisionVelocity(t *testing.T) {
	type config struct {
		name string
		p    vector.V
		q    vector.V
		v    vector.V
		want vector.V
	}

	configs := []config{
		{
			name: "Simple",
			p:    vector.V{0, 0},
			q:    vector.V{0, 100},
			v:    vector.V{0, 1},
			want: vector.V{0, 0},
		},
		{
			name: "Simple/NoCollision/Perpendicular",
			p:    vector.V{0, 0},
			q:    vector.V{100, 0},
			v:    vector.V{0, 1},
			want: vector.V{0, 1},
		},
		{
			name: "Simple/NoCollision",
			p:    vector.V{0, 0},
			q:    vector.V{0, -100},
			v:    vector.V{0, 1},
			want: vector.V{0, 1},
		},
		{
			name: "Angle",
			p:    vector.V{0, 0},
			q:    vector.V{0, 100},
			v:    vector.V{1, 1},
			want: vector.V{1, 0},
		},
	}

	for _, c := range configs {
		t.Run(c.name, func(t *testing.T) {
			a := &A{
				position: c.p.M(),
			}
			b := &A{
				position: c.q.M(),
			}

			SetCollisionVelocity(a, b, c.v.M())
			if !vector.Within(c.v, c.want) {
				t.Errorf("SetCollisionVelocity() = %v, want = %v", c.v, c.want)
			}
		})
	}
}
