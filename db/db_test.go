package db

import (
	"fmt"
	"math/rand"
	"testing"
	"time"

	"github.com/downflux/game-db/agent"
	"github.com/downflux/game-db/agent/mask"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/2d/vector/polar"
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
