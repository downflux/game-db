package mask

import (
	"fmt"
	"testing"
)

func TestValidate(t *testing.T) {
	type config struct {
		name string
		m    M
		want bool
	}

	var configs []config
	for _, m := range []M{MSizeProjectile, MSizeSmall, MSizeMedium, MSizeLarge} {
		configs = append(configs, config{
			name: fmt.Sprintf("Valid/Size=%v", m),
			m:    m,
			want: true,
		})
	}
	configs = append(configs,
		config{
			name: "InvalidMultipleSize/Projectile/Small",
			m:    MSizeProjectile | MSizeSmall,
			want: false,
		},
		config{
			name: "InvalidMultipleSize/Projectile/Small/Medium/Large",
			m:    MSizeProjectile | MSizeSmall | MSizeMedium | MSizeLarge,
			want: false,
		},

		config{
			name: "Valid/TerrainAir",
			m:    MTerrainAccessibleAir | MTerrainAir,
			want: true,
		},
		config{
			name: "Invalid/TerrainAir",
			m:    MTerrainAir,
			want: false,
		},
	)

	for _, c := range configs {
		t.Run(c.name, func(t *testing.T) {
			if got := Validate(c.m); got != c.want {
				t.Errorf("Validate() = %v, want = %v", got, c.want)
			}
		})
	}
}
