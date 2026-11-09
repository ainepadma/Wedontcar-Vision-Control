#ifndef TRACK_STOP_DIAGNOSTICS_H
#define TRACK_STOP_DIAGNOSTICS_H
#include "track_control.h"

/* ISR writes once on an automatic running->stopped transition. Main reads
 * under its existing IRQ lock. Manual restart does not erase the evidence. */
typedef struct {
    uint32_t sequence, ms;
    int16_t left, right;
    uint8_t reason, wheels, protection;
    float target, left_pwm, right_pwm;
} track_stop_snapshot_t;

static inline void track_stop_record(volatile track_stop_snapshot_t *record,
    uint8_t was_running, uint8_t running, uint8_t reason, uint8_t wheels,
    uint8_t protection, uint32_t ms, int16_t left, int16_t right,
    float target, float left_pwm, float right_pwm)
{
    if (!was_running || running || reason < TRACK_STOP_NOT_READY || reason > TRACK_STOP_STRIPE)
        return;
    record->ms=ms;record->reason=reason;record->wheels=wheels;
    record->protection=protection;record->left=left;record->right=right;
    record->target=target;record->left_pwm=left_pwm;record->right_pwm=right_pwm;
    ++record->sequence;
}
#endif
