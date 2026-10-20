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
/* In-place vertical median of three rows. It rejects a one-row bright/dark
 * impulse without mixing neighboring columns or moving lane side edges. */
uint8_t image_vertical_median3(uint8_t *image, uint16_t rows, uint16_t cols,
                               uint8_t *scratch, uint32_t scratch_size);
/* Edge-preserving 3x3 denoise. Pixels are averaged only within the local
 * intensity cluster selected around the center (or the local median when the
 * center is an impulse), so filtering is neither row-wise nor column-wise. */
uint8_t image_near_color_denoise3x3(uint8_t *image, uint16_t rows,
                                    uint16_t cols, uint8_t color_radius,
                                    uint8_t *scratch, uint32_t scratch_size);
uint8_t image_binary_despeckle3x3(uint8_t *image, uint16_t rows,
                                  uint16_t cols, uint8_t *scratch,
                                  uint32_t scratch_size);
uint8_t image_banded_percentile_stretch(uint8_t *image, uint16_t rows,
                                        uint16_t cols, uint16_t band_rows,
                                        uint8_t low_percent,
                                        uint8_t high_percent,
                                        uint32_t *histogram);

void get_hist_gram(const uint8_t* _img,
                   uint16_t _rows,
                   uint16_t _cols,
                   uint32_t* _hist_gram);
uint8_t get_threshold_otsu(const uint32_t* _hist_gram);
uint8_t image_banded_otsu_binary_offset(uint8_t *image, uint16_t rows,
                                        uint16_t cols, uint16_t band_rows,
                                        int16_t threshold_offset,
                                        uint32_t *histogram);
void binaryzation_process(uint8_t* _img,
                          uint16_t _rows,
                          uint16_t _cols,
                          uint32_t _threshold_value);
void auxiliary_process(const uint8_t* _src_pixel_mat,
                       uint16_t _src_rows, uint16_t _src_cols,
                       uint8_t _threshold_val,
                       uint16_t* _left_line, uint16_t* _mid_line, uint16_t* _right_line);


#endif
