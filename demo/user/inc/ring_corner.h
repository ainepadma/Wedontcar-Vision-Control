#ifndef RING_CORNER_H
#define RING_CORNER_H
#include <stdint.h>

/* Native, already double-flipped image coordinates: x right, y down. */
#define RING_CORNER_MAX_PER_SIDE 3U /* 3 means ambiguous; never truncate to a pair. */
#define RING_CORNER_CENTER_SAMPLES 16U
#define RING_CORNER_CONFIRM_FRAMES 3U
#define RING_CORNER_CLEAR_FRAMES 3U
#define RING_CORNER_ACTION_PERCENT 48U
#define RING_CORNER_MIN_TRAVEL 20.0f
#define RING_CORNER_HOLD_MS 120U
#define RING_CORNER_MIN_QUALITY 55U
#define RING_CORNER_REACQUIRE_FRAMES 2U

typedef enum { RING_CORNER_NONE=0, RING_CORNER_A, RING_CORNER_B } ring_corner_class_t;
typedef enum {
    RING_GUIDE_NONE=0, RING_GUIDE_270_EXTEND, RING_GUIDE_270_CONNECT,
    RING_GUIDE_A_LEFT, RING_GUIDE_A_RIGHT,
    RING_GUIDE_B_BRANCH_LEFT, RING_GUIDE_B_BRANCH_RIGHT,
    RING_GUIDE_B_MAIN_LEFT, RING_GUIDE_B_MAIN_RIGHT,
    RING_GUIDE_PDF_BASE,
    RING_CORNER_GUIDE_COUNT=RING_GUIDE_PDF_BASE+14
} ring_corner_guide_id_t;

typedef struct {
    float x,y;
    float main_dx,main_dy,branch_dx,branch_dy; /* Rays FROM vertex into real contour. */
    uint8_t kind,quality;
} ring_corner_point_t;

typedef struct {
    float x0,y0,x1,y1;
    uint8_t valid;
} ring_corner_line_t;

typedef struct {
    int16_t error,heading; /* Same units/sign as ordinary track_path_observation_t. */
    uint8_t valid,confidence;
    ring_corner_line_t boundary[2]; /* LEFT/RIGHT virtual segments, not centers. */
    uint8_t corner_side,corner_kind;
    float corner_x,corner_y;
    uint16_t center[RING_CORNER_CENTER_SAMPLES],center_valid;
    uint16_t center_link_valid; /* bit i: every row between samples i-1 and i is valid. */
} ring_corner_guide_t;

typedef struct {
    uint32_t frame_number;
    uint16_t rows,cols,detector_near_row; /* Last vertex row supported by every sampling radius. */
    uint16_t border[2][480], border_index[2][480];
    uint8_t border_flags[480];
    uint8_t quality[2],continuous[2],gap[2],bottom_open[2],paired;
    uint8_t entry[2],cross_entry;
    ring_corner_point_t a[2],p[2],v[2],breakpoint[2];
    ring_corner_point_t mouth_upper[2],mouth_lower[2];
    uint8_t paired_mouth;
    uint8_t count[2],road_clear,continued_mask; /* bit side: locally reacquired A; never an event vote. */
    ring_corner_point_t point[2][RING_CORNER_MAX_PER_SIDE]; /* Near to far. */
    ring_corner_guide_t guide[RING_CORNER_GUIDE_COUNT];
} ring_corner_frame_t;

typedef struct {
    const uint16_t *left,*right;
    const uint8_t *flags; /* bit0/1 reliable left/right, bit2/3 open. */
    float width_near,width_slope,center_near,center_slope;
    uint8_t model_ready;
} ring_corner_context_t;

typedef struct {
    ring_corner_point_t point[2],upper[2];
    uint16_t rows,cols;
    uint8_t valid[2],misses[2];
} ring_corner_tracker_t;

/* Pure bounded per-frame geometry; must never alter binary or carry motion state.
 * The ISR chooses among precomputed guide candidates, including on a transition
 * frame. No state-dependent old-frame line can be selected accidentally. */
void ring_corner_process(const uint8_t *binary,uint16_t rows,uint16_t cols,
                         const ring_corner_context_t *context,ring_corner_frame_t *out);
/* Once per processed image, after raw detection. Counts remain raw; recovered
 * guides are recomputed from CURRENT pixels, and expire after two misses. */
void ring_corner_stabilize(const uint8_t *binary,const ring_corner_context_t *context,
                          ring_corner_tracker_t *tracker,ring_corner_frame_t *out);
unsigned ring_pdf_guide(unsigned stage,unsigned side);
#endif
