#ifndef VISION_GEOMETRY_H
#define VISION_GEOMETRY_H
/* Sensor already crops full-view x~10..90%, y~28..92% after double flip.
 * Defaults consume the WHOLE delivered window. Overrides are for historical
 * fixture regression only; the firmware project defines no overrides. */
#ifndef VISION_SEARCH_START_PERCENT
#define VISION_SEARCH_START_PERCENT 0U
#endif
#ifndef VISION_SEARCH_END_PERCENT
#define VISION_SEARCH_END_PERCENT 100U
#endif
#ifndef VISION_TRACK_START_PERCENT
#define VISION_TRACK_START_PERCENT 0U
#endif
#ifndef VISION_NEAR_START_PERCENT
#define VISION_NEAR_START_PERCENT 66U
#endif
#if VISION_SEARCH_START_PERCENT >= VISION_SEARCH_END_PERCENT || VISION_SEARCH_END_PERCENT > 100U
#error "Invalid vision window"
#endif
#if VISION_TRACK_START_PERCENT < VISION_SEARCH_START_PERCENT || VISION_TRACK_START_PERCENT >= VISION_SEARCH_END_PERCENT
#error "Track window must lie inside search window"
#endif
#if VISION_NEAR_START_PERCENT < VISION_TRACK_START_PERCENT || VISION_NEAR_START_PERCENT >= VISION_SEARCH_END_PERCENT
#error "Invalid near-field seed zone"
#endif
#endif
