package agent

import (
	"fmt"
	"math"
	"math/rand"
	"testing"
	"time"

	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/2d/vector/polar"
)

func TestSetVelocity(t *testing.T) {
	type config struct {
		name            string
		agentV          vector.V
		v               vector.V
		maxVelocity     float64
		maxAcceleration float64
		want            vector.V
	}

	configs := []config{
		{
			name:            "Within",
			agentV:          vector.V{0, 0},
			v:               vector.V{10, 0},
			maxVelocity:     100,
			maxAcceleration: 100,
			want:            vector.V{10, 0},
		},
		{
			name:            "TooFast",
			agentV:          vector.V{0, 0},
			v:               vector.V{10, 0},
			maxVelocity:     5,
			maxAcceleration: 100,
			want:            vector.V{5, 0},
		},
		{
			name:            "TooSudden",
			agentV:          vector.V{1, 0},
			v:               vector.V{10, 0},
			maxVelocity:     100,
			maxAcceleration: 1,
			want:            vector.V{2, 0},
		},
		{
			name:            "AlwaysStop",
			agentV:          vector.V{100, 0},
			v:               vector.V{0, 0},
			maxVelocity:     100,
			maxAcceleration: 0,
			want:            vector.V{0, 0},
		},
	}

	for _, c := range configs {
		t.Run(c.name, func(t *testing.T) {
			a := &A{
				velocity:        c.agentV.M(),
				maxVelocity:     c.maxVelocity,
				maxAcceleration: c.maxAcceleration,
			}

			SetVelocity(a, c.v.M())
			if !vector.Within(c.v, c.want) {
				t.Errorf("v = %v, want = %v", c.v, c.want)
			}
		})
	}
}

func rn(min, max float64) float64  { return min + rand.Float64()*(max-min) }
func rv(min, max float64) vector.V { return vector.V{rn(min, max), rn(min, max)} }

func TestSetCollisionVelocity(t *testing.T) {
	const n = 0
	type config struct {
		name string
		p    vector.V
		qs   []vector.V
		v    vector.V
		want vector.V
	}

	configs := []config{
		{
			name: "Simple",
			p:    vector.V{0, 0},
			qs:   []vector.V{vector.V{0, 100}},
			v:    vector.V{0, 2},
			want: vector.V{0, 0},
		},
		{
			name: "Simple/NoCollision/Perpendicular",
			p:    vector.V{0, 0},
			qs:   []vector.V{vector.V{100, 0}},
			v:    vector.V{0, 2},
			want: vector.V{0, 2},
		},
		{
			name: "Simple/NoCollision",
			p:    vector.V{0, 0},
			qs:   []vector.V{vector.V{0, -100}},
			v:    vector.V{0, 2},
			want: vector.V{0, 2},
		},
		{
			name: "Angle",
			p:    vector.V{0, 0},
			qs:   []vector.V{vector.V{0, 100}},
			v:    vector.V{2, 2},
			want: vector.V{2, 0},
		},
		{
			name: "Multiple",
			p:    vector.V{0, 0},
			qs: []vector.V{
				vector.V{0, 1},
				vector.V{1, 0},
			},
			v:    vector.V{1, 1},
			want: vector.V{0, 0},
		},
		{
			name: "Multiple/Reverse",
			p:    vector.V{0, 0},
			qs: []vector.V{
				vector.V{0, 1},
				vector.V{1, 0},
			},
			v:    vector.V{-1, -1},
			want: vector.V{-1, -1},
		},
		{
			name: "Multiple/Flanked",
			p:    vector.V{0, 0},
			qs: []vector.V{
				vector.V{1, 0},
				vector.V{-1, 0},
			},
			v:    vector.V{3, 1},
			want: vector.V{0, 1},
		},
		{
			name: "Multiple/Flanked/Reversed",
			p:    vector.V{0, 0},
			qs: []vector.V{
				vector.V{-1, 0},
				vector.V{1, 0},
			},
			v:    vector.V{3, 1},
			want: vector.V{0, 1},
		},
		// A = (0, 0)
		// B = (1, 0)
		// C = (math.Sqrt(3), 1)
		// v = (1, 1)
		//
		// C lies π / 6 above the Y-axis.
		// v lies π / 4 above the Y-axis.
		{
			name: "OrderInvariant/Custom",
			p:    vector.V{0, 0},
			qs: []vector.V{
				vector.V{1, 0},
				vector.V{math.Sqrt(3), 1},
			},
			v:    vector.V{1, 1},
			want: vector.V{-math.Sqrt(3) / 4, 0.75},
		},
		{
			name: "OrderInvariant/Custom/Inverse",
			p:    vector.V{0, 0},
			qs: []vector.V{
				vector.V{math.Sqrt(3), 1},
				vector.V{1, 0},
			},
			v:    vector.V{1, 1},
			want: vector.V{-math.Sqrt(3) / 4, 0.75},
		},
	}

	for i := 0; i < n; i++ {
		const min, max = 0, 500
		p := rv(min, max)
		qs := []vector.V{rv(min, max), rv(min, max)}
		v := rv(min, max)

		want := vector.M{0, 0}
		want.Copy(v)

		for _, q := range qs {
			SetCollisionVelocity(&A{
				position: p.M(),
			}, &A{
				position: q.M(),
			},
				want,
			)
		}
		configs = append(configs, config{
			name: fmt.Sprintf("OrderInvariant/%v", i),
			p:    p,
			qs:   []vector.V{qs[1], qs[0]},
			v:    v,
			want: want.V(),
		})
	}

	for _, c := range configs {
		t.Run(c.name, func(t *testing.T) {
			v := vector.M{0, 0}
			v.Copy(c.v)

			a := &A{
				position: c.p.M(),
			}
			for _, q := range c.qs {
				b := &A{
					position: q.M(),
				}
				SetCollisionVelocity(a, b, v)
			}

			if !vector.Within(v.V(), c.want) {
				t.Errorf("DEBUG: %v, %v, %v", c.p, c.qs, c.v)
				t.Errorf("SetCollisionVelocity() = %v, want = %v", v, c.want)
			}
		})
	}
}

func TestSetHeading(t *testing.T) {
	type config struct {
		name  string
		v     vector.V
		h     polar.V
		omega float64
		wantV vector.V
		wantH polar.V
	}

	configs := []config{
		{
			name:  "Aligned",
			v:     vector.V{10, 0},
			h:     polar.V{1, 0},
			omega: 0,
			wantV: vector.V{10, 0},
			wantH: polar.V{1, 0},
		},
		{
			name:  "Movable",
			v:     vector.V{10, 0},
			h:     polar.V{1, 1},
			omega: math.Pi / 2,
			wantV: vector.V{10, 0},
			wantH: polar.V{1, 0},
		},
		{
			name:  "SharpTurn",
			v:     vector.V{10, 0},
			h:     polar.V{1, math.Pi},
			omega: math.Pi / 2,
			wantV: vector.V{0, 0},
			wantH: polar.V{1, math.Pi / 2},
		},
	}

	for _, c := range configs {
		t.Run(c.name, func(t *testing.T) {
			a := &A{
				heading:            c.h.M(),
				maxAngularVelocity: c.omega,
			}

			gotH := polar.M{0, 0}
			SetHeading(a, time.Second, c.v.M(), gotH)
			if !vector.Within(c.v, c.wantV) {
				t.Errorf("v = %v, want = %v", c.v, c.wantV)
			}
			if !polar.Within(gotH.V(), c.wantH) {
				t.Errorf("h = %v, want = %v", gotH, c.wantH)
			}
		})
	}
}
