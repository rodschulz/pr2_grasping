# Descriptor type
int32 type

# Search radius used in the KNN search
float64 searchRadius

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

# Descriptor type
int32 TYPE_UNKNOWN = 0
int32 TYPE_DCH = 1
int32 TYPE_SHOT = 2
#int32 TYPE_USC = 3		# not implemented yet
#int32 TYPE_PFH = 4		# not implemented yet
#int32 TYPE_ROPS = 5	# not implemented yet
