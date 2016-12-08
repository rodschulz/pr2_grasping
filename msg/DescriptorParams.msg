# Search radius used in the KNN search
float64 patchSize

# Number of bands in the descriptor
int32 bandNumber

# Width of each band
float64 bandWidth

# True if bands are bidirectional
bool bidirectional

# True if angles were computed projecting the vectors over the band's plane
bool useProjection

# Size of the bins used in the sequence construction
float64 sequenceBin

# Statistic to use in the sequence calculation
int32 sequenceStat


# Sequence stats
int32 STAT_MEAN = 0
int32 STAT_MEDIAN = 1
