package collider

import (
	"github.com/downflux/go-collider/agent"
	"github.com/downflux/go-collider/feature"
	"github.com/downflux/go-collider/mask"
	"github.com/downflux/go-geometry/2d/vector"
	"github.com/downflux/go-geometry/nd/hyperrectangle"

	chr "github.com/downflux/go-collider/internal/geometry/hyperrectangle"
)

// IsColliding checks if two agents are actually physically overlapping. This
// does not care about the extra logic for e.g. squishing.
func IsColliding(a *agent.A, b *agent.A) bool {
	if a.ID() == b.ID() {
		return false
	}

	m, n := a.Mask(), b.Mask()
	if (m|n)&mask.MSizeProjectile != 0 {
		return false
	}

	// Agents are allowed to overlap if (only) one of them is in the air.
	if (m^n)&mask.MTerrainAir == mask.MTerrainAir {
		return false
	}

	r := a.Radius() + b.Radius()
	if vector.SquaredMagnitude(vector.Sub(a.Position(), b.Position())) > r*r {
		return false
	}
	return true

}

func IsSquishableColliding(a *agent.A, b *agent.A) bool {
	if IsColliding(a, b) {
		// TODO(minkezhang): Check for team.
		if a.Mask()&mask.SizeCheck > b.Mask()&mask.SizeCheck {
			return false
		}
		return true
	}
	return false
}

func IsCollidingFeature(a *agent.A, f *feature.F) bool {
	m, n := a.Mask(), f.Mask()

	// Feature and agent are allowed to overlap if (only) one of them is in
	// the air.
	if (m^n)&mask.MTerrainAir == mask.MTerrainAir {
		return false
	}

	if hyperrectangle.Disjoint(agent.AABB(a.Position(), a.Radius()), f.AABB()) {
		return false
	}

	return chr.Collide(f.AABB(), a.Position(), a.Radius())
}
