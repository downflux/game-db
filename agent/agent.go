package agent

import (
	"github.com/downflux/go-geometry/2d/vector"
)

type AgentSizeClass uint8

const (
	AgentSizeClassUnknown AgentSizeClass = iota
	AgentSizeClassSmall
	AgentSizeClassMedium
	AgentSizeClassLarge
)

type Team uint8

type TerrainLayerMask uint8

const (
	TerrainLayerMaskNone TerrainLayerMask = iota
	TerrainLayerMaskLand                  = 1 << iota
	TerrainLayerMaskAir
	TerrainLayerMaskSea
)

type LayerMask uint64

const (
	LayerMaskNone       LayerMask = iota
	LayerMaskProjectile           = 1 << iota
	LayerMaskGround
	LayerMaskAir
	LayerMaskSea
)

func (m LayerMask) Team() Team                     { return Team(m >> 62) }
func (m LayerMask) SizeClass() AgentSizeClass      { return AgentSizeClass((m << 2) >> 60) }
func (m LayerMask) TerrainLayer() TerrainLayerMask { return TerrainLayerMask((m << 4) >> 59) }

type A struct {
	position vector.M
	velocity vector.M
	radius   float64
	// orientation is a radian angle relative to the positive
	orientation float64

	mask LayerMask
}

func (a *A) Position() vector.V   { return a.position.V() }
func (a *A) Velocity() vector.V   { return a.velocity.V() }
func (a *A) Radius() float64      { return a.radius }
func (a *A) Orientation() float64 { return a.orientation }

// IgnoreCollision checks if two agents need to do any additional processing in
// the case that their bounding circles overlap.
func (a *A) IgnoreCollision(b *A) bool {
	if a.mask|b.mask&LayerMaskProjectile != 0 {
		return true
	}

	if a.mask.TerrainLayer()^b.mask.TerrainLayer()&TerrainLayerMaskAir != 0 {
		return true
	}
	return false
}
