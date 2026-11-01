#ifndef _IMG_PROCESS_H_
#define _IMG_PROCESS_H_
#include <stdint.h>

/* In-place 3x3 integer mean (sum / 9), with the outer border unchanged.
 * The caller supplies at least 2 * cols scratch bytes; that used region must
 * not overlap the image. Returns 1 on success, or 0 without changing either
 * buffer for NULL pointers, dimensions below 3, insufficient/overlapping scratch.
 */
uint8_t image_mean3x3(uint8_t *image, uint16_t rows, uint16_t cols,
                      uint8_t *scratch, uint32_t scratch_size);

/* All persistent values use the original sensor grey scale (Q8), never an
 * enhanced scale that changes with gain. Zero-initialize before first use.
 * Only parameters are retained; image positions/pixels are never retained.
 */
#define IMAGE_TRACK_TILE_COLS 4U
#define IMAGE_TRACK_TILE_ROWS 3U
#define IMAGE_TRACK_TILE_COUNT (IMAGE_TRACK_TILE_COLS * IMAGE_TRACK_TILE_ROWS)
/* Vision tuning: larger white weight raises the threshold (more black).
 * Larger hole limits remove more dirt; long/edge-connected black areas stay.
 */
#ifndef IMAGE_TRACK_WHITE_WEIGHT_PERCENT
#define IMAGE_TRACK_WHITE_WEIGHT_PERCENT 42U
#endif
#ifndef IMAGE_TRACK_HOLE_MAX_SIDE
#define IMAGE_TRACK_HOLE_MAX_SIDE 7U
#endif
#define IMAGE_TRACK_HOLE_MAX_AREA (IMAGE_TRACK_HOLE_MAX_SIDE * IMAGE_TRACK_HOLE_MAX_SIDE)
#ifndef IMAGE_TRACK_PARAMETER_TAU_MS
#define IMAGE_TRACK_PARAMETER_TAU_MS 25U
#endif
#ifndef IMAGE_TRACK_LOCAL_STRENGTH_PERCENT
#define IMAGE_TRACK_LOCAL_STRENGTH_PERCENT 50U
#endif
#define IMAGE_TRACK_TARGET_LEVEL 192U
#define IMAGE_TRACK_MAX_GAIN 32U
#define IMAGE_TRACK_GLOBAL_QUALITY_MIN 650U
#define IMAGE_TRACK_LOCAL_QUALITY_MIN 720U
#define IMAGE_TRACK_WEAK_QUALITY_MIN 550U
#define IMAGE_TRACK_WEAK_HOLD_MS 100U
#define IMAGE_TRACK_RESET_GAP_MS 250U
#if IMAGE_TRACK_MAX_GAIN < 1U || IMAGE_TRACK_MAX_GAIN > 64U
#error "Maximum digital gain must stay within 1..64"
#endif
#if IMAGE_TRACK_LOCAL_STRENGTH_PERCENT > 100U || IMAGE_TRACK_PARAMETER_TAU_MS == 0U
#error "Local strength must be 0..100; smoothing time must be positive"
#endif
#if IMAGE_TRACK_WHITE_WEIGHT_PERCENT < 30U || IMAGE_TRACK_WHITE_WEIGHT_PERCENT > 60U
#error "White weight must stay within 30..60 percent"
#endif
#if IMAGE_TRACK_HOLE_MAX_SIDE < 1U || IMAGE_TRACK_HOLE_MAX_SIDE > 9U
#error "Hole side must stay within 1..9 pixels"
#endif
typedef struct {
    uint32_t last_ms, last_good_ms;
    uint16_t rows, cols, roi_first, roi_end;
    uint16_t gain_q8, threshold_raw_q8;
    uint16_t tile_raw_q8[IMAGE_TRACK_TILE_COUNT];
    uint8_t ready;
} image_track_preprocess_state_t;

/* Ground-only threshold estimation and denoising, full-frame 0/255 output.
 * roi_first is inclusive, roi_end exclusive, in the already oriented image.
 * Excluded rows cannot influence gain, filtering, threshold or ROI output.
 * scratch needs 5 * cols bytes, histogram 256 uint32_t entries, all disjoint
 * from the image and each other. threshold receives the enhanced-domain value.
 * Returns 0 without mutation on invalid arguments, 1 after processing.
 * Sizes up to 480 rows, 752 columns. Constant/weakly separated ground is
 * reported with threshold=255 (invalid for control); current binary preview
 * is retained using the previous threshold or a fallback. No hardware writes.
 */
uint8_t image_track_preprocess(uint8_t *image, uint16_t rows, uint16_t cols,
                               uint16_t roi_first, uint16_t roi_end,
                               uint8_t *scratch, uint32_t scratch_size,
                               uint32_t *histogram, uint8_t *threshold);

/* Stateful variant used by the camera loop. Parameter smoothing uses elapsed
 * milliseconds; first frame, changed geometry or a long gap resets it.
 * The stateless API above remains available for independent frame replay.
 */
uint8_t image_track_preprocess_stable(uint8_t *image, uint16_t rows, uint16_t cols,
                                      uint16_t roi_first, uint16_t roi_end,
                                      uint8_t *scratch, uint32_t scratch_size,
                                      uint32_t *histogram, uint8_t *threshold,
                                      image_track_preprocess_state_t *state, uint32_t now_ms);

/* Fill complete 8-connected black components only within the configured area
 * and bounding-box limits. Components touching any region edge are preserved. Returns the
 * number of filled pixels. No full-image queue or visited bitmap is needed.
 */
uint32_t image_fill_small_black_holes(uint8_t *image, uint16_t rows, uint16_t cols);

/* Remove isolated 1x1 / horizontal 2x1 spots with a completely opposite-colour
 * surround, plus detached white 2x2 islands surrounded by black. Two scratch
 * rows, disjoint from image; input is 0/255. Outer border, connected white
 * road, black 2x2 holes, longer lines and larger islands are preserved.
 */
uint8_t image_clean_binary(uint8_t *image, uint16_t rows, uint16_t cols,
                           uint8_t *scratch, uint32_t scratch_size);

void get_hist_gram(const uint8_t* _img,
                   uint16_t _rows,
                   uint16_t _cols,
                   uint32_t* _hist_gram);
uint8_t get_threshold_otsu(const uint32_t* _hist_gram);
void binaryzation_process(uint8_t* _img,
                          uint16_t _rows,
                          uint16_t _cols,
                          uint32_t _threshold_value);
void auxiliary_process(const uint8_t* _src_pixel_mat,
                       uint16_t _src_rows, uint16_t _src_cols,
                       uint8_t _threshold_val,
                       uint16_t* _left_line, uint16_t* _mid_line, uint16_t* _right_line);


#endif
