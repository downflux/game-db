package kinematics

import (
	"math"
	"math/rand"
	"testing"
	"time"

	"github.com/downflux/go-database/agent"
	"github.com/downflux/go-database/feature"
	"github.com/downflux/go-database/flags"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/2d/vector/polar"
	"github.com/downflux/go-geometry/nd/hyperrectangle"

	magent "github.com/downflux/go-database/agent/mock"
	mfeature "github.com/downflux/go-database/feature/mock"
	vnd "github.com/downflux/go-geometry/nd/vector"
)

func TestClampFeatureCollisionVelocity(t *testing.T) {
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

			a := magent.New(0, agent.O{
				Velocity: vector.V{0, 0},
				Heading:  polar.V{1, 0},
				Position: c.p,
				Flags:    flags.FSizeSmall,
			})
			f := mfeature.New(0, feature.O{
				Min: vector.V(c.aabb.Min()),
				Max: vector.V(c.aabb.Max()),
			})
			ClampFeatureCollisionVelocity(a, f, v)

			if !vector.Within(v.V(), c.want) {
				t.Errorf("ClampFeatureCollisionVelocity() = %v, want = %v", v, c.want)
			}
		})
	}
}

func TestClampCollisionVelocity(t *testing.T) {
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

			a := magent.New(1, agent.O{
				Velocity: vector.V{0, 0},
				Heading:  polar.V{1, 0},
				Position: c.p,
				Flags:    flags.FSizeSmall,
			})
			b := magent.New(2, agent.O{
				Velocity: vector.V{0, 0},
				Heading:  polar.V{1, 0},
				Position: c.q,
				Flags:    flags.FSizeSmall,
			})
			ClampCollisionVelocity(a, b, v)

			if !vector.Within(v.V(), c.want) {
				t.Errorf("SetCollisionVelocity() = %v, want = %v", v, c.want)
			}
		})
	}
}

func TestClampAcceleration(t *testing.T) {
	type config struct {
		name            string
		tv              vector.V
		v               vector.V
		maxAcceleration float64
		want            vector.V
	}

	configs := []config{
		{
			name:            "TooSudden",
			tv:              vector.V{1, 0},
			v:               vector.V{10, 0},
			maxAcceleration: 1,
			want:            vector.V{2, 0},
		},
		{
			name:            "AlwaysStop",
			tv:              vector.V{100, 0},
			v:               vector.V{0, 0},
			maxAcceleration: 0,
			want:            vector.V{0, 0},
		},
	}

	for _, c := range configs {
		t.Run(c.name, func(t *testing.T) {
			a := magent.New(0, agent.O{
				Heading:  polar.V{1, 0},
				Position: vector.V{0, 0},

				Velocity:        c.tv,
				MaxAcceleration: c.maxAcceleration,
				Flags:           flags.FSizeSmall,
			})

			ClampAcceleration(a, c.v.M(), time.Second)
			if !vector.Within(c.v, c.want) {
				t.Errorf("ClampAcceleration() = %v, want = %v", c.v, c.want)
			}
		})
	}

}

func TestClampVelocity(t *testing.T) {
	type config struct {
		name        string
		v           vector.V
		maxVelocity float64
		want        vector.V
	}

	configs := []config{
		{
			name:        "Within",
			v:           vector.V{10, 0},
			maxVelocity: 100,
			want:        vector.V{10, 0},
		},
		{
			name:        "TooFast",
			v:           vector.V{10, 0},
			maxVelocity: 5,
			want:        vector.V{5, 0},
		},
	}

	for _, c := range configs {
		t.Run(c.name, func(t *testing.T) {
			a := magent.New(0, agent.O{
				Position: vector.V{0, 0},
				Heading:  polar.V{1, 0},
				Velocity: vector.V{0, 0},

				MaxVelocity: c.maxVelocity,
				Flags:       flags.FSizeSmall,
			})

			ClampVelocity(a, c.v.M())
			if !vector.Within(c.v, c.want) {
				t.Errorf("ClampVelocity() = %v, want = %v", c.v, c.want)
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
			p:    vector.V{4, -1},
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
			p:    vector.V{4, 11},
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
		{
			name: "Corner/Slide",
			p:    vector.V{0, -0.1},
			aabbs: []hyperrectangle.R{
				*hyperrectangle.New(
					vnd.V{1, 0},
					vnd.V{2, 10},
				),
			},
			v: vector.V{1, -1},
			// Determined experimentally; this result should be
			// close to
			//
			//  vector.V{0, -1}
			want: vector.V{
				0.10891089108910867,
				-1.0891089108910892,
			},
		},
		{
			name: "Experimental",
			p: vector.V{
				60.0040783686527,
				80.40391843262739,
			},
			aabbs: []hyperrectangle.R{
				*hyperrectangle.New(
					vnd.V{70, 20},
					vnd.V{90, 80},
				),
			},
			v: vector.V{10, 10},
			// This is determined experimentally, but shouldb e
			// fairly close to
			//
			//  vector.V{0, 10}
			want: vector.V{0.4197262159146238, 10.387122800050154},
		},
	}

	for _, c := range configs {
		t.Run(c.name, func(t *testing.T) {
			v := vector.M{0, 0}
			v.Copy(c.v)

			a := magent.New(0, agent.O{
				Heading:  polar.V{1, 0},
				Velocity: vector.V{0, 0},
				Position: c.p,
				Flags:    flags.FSizeSmall,
			})
			for _, aabb := range c.aabbs {
				f := mfeature.New(0, feature.O{
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

			a := magent.New(1, agent.O{
				Velocity: vector.V{0, 0},
				Heading:  polar.V{1, 0},
				Position: c.p,
				Flags:    flags.FSizeSmall,
			})
			for _, q := range c.qs {
				b := magent.New(2, agent.O{
					Velocity: vector.V{0, 0},
					Heading:  polar.V{1, 0},
					Position: q,
					Flags:    flags.FSizeSmall,
				})
				SetCollisionVelocity(a, b, v)
			}

			if !vector.Within(v.V(), c.want) {
				t.Errorf("SetCollisionVelocity() = %v, want = %v", v, c.want)
			}
		})
	}
}

func TestClampHeading(t *testing.T) {
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
			a := magent.New(0, agent.O{
				Velocity:           vector.V{0, 0},
				Position:           vector.V{0, 0},
				Heading:            c.h,
				MaxAngularVelocity: c.omega,
				Flags:              flags.FSizeSmall,
			})

			gotH := polar.M{0, 0}
			ClampHeading(a, time.Second, c.v.M(), gotH)
			if !vector.Within(c.v, c.wantV) {
				t.Errorf("v = %v, want = %v", c.v, c.wantV)
			}
			if !polar.Within(gotH.V(), c.wantH) {
				t.Errorf("h = %v, want = %v", gotH, c.wantH)
			}
		})
	}
}
