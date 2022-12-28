package kinematics

import (
	"math"
	"math/rand"
	"testing"
	"time"

	"github.com/downflux/go-collider/agent"
	"github.com/downflux/go-collider/feature"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/2d/vector/polar"
	"github.com/downflux/go-geometry/nd/hyperrectangle"

	vnd "github.com/downflux/go-geometry/nd/vector"
)

func TestSetFeatureCollisionVelocityStrict(t *testing.T) {
	type config struct {
		name string
		p    vector.V
		aabb hyperrectangle.R
		v    vector.V
		want vector.V
	}

	configs := []config{
		{
			name: "Simple",
			p:    vector.V{0, 5},
			aabb: *hyperrectangle.New(
				vnd.V{1, 0},
				vnd.V{2, 10},
			),
			v:    vector.V{1, 0},
			want: vector.V{0, 0},
		},
		{
			name: "Simple/NoCollision/Perpendicular",
			p:    vector.V{0, 5},
			aabb: *hyperrectangle.New(
				vnd.V{1, 0},
				vnd.V{2, 10},
			),
			v:    vector.V{0, 1},
			want: vector.V{0, 1},
		},
		{
			name: "Corner",
			p:    vector.V{0, 0.9},
			aabb: *hyperrectangle.New(
				vnd.V{1, 0},
				vnd.V{2, 10},
			),
			v:    vector.V{1, -1},
			want: vector.V{0, 0},
		},
	}

	for _, c := range configs {
		t.Run(c.name, func(t *testing.T) {
			v := vector.M{0, 0}
			v.Copy(c.v)

			a := agent.New(agent.O{
				Velocity: vector.V{0, 0},
				Heading:  polar.V{1, 0},
				Position: c.p,
			})
			f := feature.New(feature.O{
				Min: vector.V(c.aabb.Min()),
				Max: vector.V(c.aabb.Max()),
			})
			SetFeatureCollisionVelocityStrict(a, f, v)

			if !vector.Within(v.V(), c.want) {
				t.Errorf("SetFeatureCollisionVelocity() = %v, want = %v", v, c.want)
			}
		})
	}
}

func TestSetCollisionVelocityStrict(t *testing.T) {
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
			v:    vector.V{0, 2},
			want: vector.V{0, 0},
		},
		{
			name: "Simple/NoCollision/Perpendicular",
			p:    vector.V{0, 0},
			q:    vector.V{100, 0},
			v:    vector.V{0, 2},
			want: vector.V{0, 2},
		},
	}

	for _, c := range configs {
		t.Run(c.name, func(t *testing.T) {
			v := vector.M{0, 0}
			v.Copy(c.v)

			a := agent.New(agent.O{
				Velocity: vector.V{0, 0},
				Heading:  polar.V{1, 0},
				Position: c.p,
			})
			agent.SetID(a, 1)
			b := agent.New(agent.O{
				Velocity: vector.V{0, 0},
				Heading:  polar.V{1, 0},
				Position: c.q,
			})
			agent.SetID(b, 2)
			SetCollisionVelocityStrict(a, b, v)

			if !vector.Within(v.V(), c.want) {
				t.Errorf("SetCollisionVelocity() = %v, want = %v", v, c.want)
			}
		})
	}
}

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
			a := agent.New(agent.O{
				Position:        vector.V{0, 0},
				Heading:         polar.V{1, 0},
				Velocity:        c.agentV,
				MaxVelocity:     c.maxVelocity,
				MaxAcceleration: c.maxAcceleration,
			})

			SetVelocity(a, c.v.M())
			if !vector.Within(c.v, c.want) {
				t.Errorf("v = %v, want = %v", c.v, c.want)
			}
		})
	}
}

func rn(min, max float64) float64  { return min + rand.Float64()*(max-min) }
func rv(min, max float64) vector.V { return vector.V{rn(min, max), rn(min, max)} }

func TestSetFeatureCollisionVelocity(t *testing.T) {
	type config struct {
		name  string
		p     vector.V
		aabbs []hyperrectangle.R
		v     vector.V
		want  vector.V
	}

	configs := []config{
		{
			name: "MinX",
			p:    vector.V{1, 1},
			aabbs: []hyperrectangle.R{
				*hyperrectangle.New(
					vnd.V{2, 0},
					vnd.V{10, 10},
				),
			},
			v:    vector.V{1, 1},
			want: vector.V{0, 1},
		},
		{
			name: "MaxX",
			p:    vector.V{11, 1},
			aabbs: []hyperrectangle.R{
				*hyperrectangle.New(
					vnd.V{2, 0},
					vnd.V{10, 10},
				),
			},
			v:    vector.V{-1, 1},
			want: vector.V{0, 1},
		},
		{
			name: "MinY",
			p:    vector.V{0, -1},
			aabbs: []hyperrectangle.R{
				*hyperrectangle.New(
					vnd.V{2, 0},
					vnd.V{10, 10},
				),
			},
			v:    vector.V{1, 1},
			want: vector.V{1, 0},
		},
		{
			name: "MaxY",
			p:    vector.V{0, 11},
			aabbs: []hyperrectangle.R{
				*hyperrectangle.New(
					vnd.V{2, 0},
					vnd.V{10, 10},
				),
			},
			v:    vector.V{1, -1},
			want: vector.V{1, 0},
		},
		{
			name: "Corner",
			p:    vector.V{0, 0.9},
			aabbs: []hyperrectangle.R{
				*hyperrectangle.New(
					vnd.V{1, 0},
					vnd.V{2, 10},
				),
			},
			v:    vector.V{1, -1},
			want: vector.V{0, -1},
		},
	}

	for _, c := range configs {
		t.Run(c.name, func(t *testing.T) {
			v := vector.M{0, 0}
			v.Copy(c.v)

			a := agent.New(agent.O{
				Heading:  polar.V{1, 0},
				Velocity: vector.V{0, 0},
				Position: c.p,
			})
			for _, aabb := range c.aabbs {
				f := feature.New(feature.O{
					Min: vector.V(aabb.Min()),
					Max: vector.V(aabb.Max()),
				})

				SetFeatureCollisionVelocity(a, f, v)
			}

			if !vector.Within(v.V(), c.want) {
				t.Errorf("SetFeatureCollisionVelocity() = %v, want = %v", v, c.want)
			}
		})
	}
}

func TestSetCollisionVelocity(t *testing.T) {
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
	}

	for _, c := range configs {
		t.Run(c.name, func(t *testing.T) {
			v := vector.M{0, 0}
			v.Copy(c.v)

			a := agent.New(agent.O{
				Velocity: vector.V{0, 0},
				Heading:  polar.V{1, 0},
				Position: c.p,
			})
			agent.SetID(a, 1)
			for _, q := range c.qs {
				b := agent.New(agent.O{
					Velocity: vector.V{0, 0},
					Heading:  polar.V{1, 0},
					Position: q,
				})
				agent.SetID(b, 2)
				SetCollisionVelocity(a, b, v)
			}

			if !vector.Within(v.V(), c.want) {
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
			a := agent.New(agent.O{
				Velocity:           vector.V{0, 0},
				Position:           vector.V{0, 0},
				Heading:            c.h,
				MaxAngularVelocity: c.omega,
			})

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
