/*
 *@第十九届东南大学智能车竞赛基础四轮组部分开源学习代码
 */


#include "img_process.h"
#include "string.h"

static uint16_t image_band_center(uint16_t first, uint16_t count)
{
    return (uint16_t)(first + count / 2U);
}

static uint8_t image_interpolate_u8(uint8_t from, uint8_t to,
                                    uint16_t position, uint16_t distance)
{
    if (distance == 0U || position >= distance)
        return to;
    return (uint8_t)(((uint32_t)from * (distance - position) +
                      (uint32_t)to * position + distance / 2U) / distance);
}

static void image_band_percentiles(const uint8_t *image, uint16_t count,
                                   uint16_t cols, uint8_t low_percent,
                                   uint8_t high_percent, uint32_t *histogram,
                                   uint8_t *low, uint8_t *high)
{
    get_hist_gram(image, count, cols, histogram);
    const uint32_t samples = (uint32_t)count * cols;
    const uint32_t low_target = (samples * low_percent + 99U) / 100U;
    const uint32_t high_target = (samples * high_percent + 99U) / 100U;
    uint32_t cumulative = 0U;
    uint16_t value = 0U;
    for (; value < 255U; ++value)
    {
        cumulative += histogram[value];
        if (cumulative >= low_target)
            break;
    }
    *low = (uint8_t)value;
    cumulative = 0U;
    for (value = 0U; value < 255U; ++value)
    {
        cumulative += histogram[value];
        if (cumulative >= high_target)
            break;
    }
    *high = (uint8_t)value;
}

static uint8_t image_median3_value(uint8_t first, uint8_t second,
                                   uint8_t third)
{
    if (first > second)
    {
        uint8_t swap = first;
        first = second;
        second = swap;
    }
    if (second > third)
    {
        second = third;
        if (first > second)
            second = first;
    }
    return second;
}

uint8_t image_mean3x3(uint8_t *image, uint16_t rows, uint16_t cols,
                      uint8_t *scratch, uint32_t scratch_size)
{
    uint32_t required = (uint32_t)cols * 2U;
    uint32_t image_size = (uint32_t)rows * cols;
    uintptr_t image_address = (uintptr_t)image;
    uintptr_t scratch_address = (uintptr_t)scratch;
    uint8_t *previous;
    uint8_t *current;

    if (image == NULL || scratch == NULL || rows < 3U || cols < 3U ||
        scratch_size < required)
        return 0U;
    /* Compare integer address distances, not unrelated C pointers, and avoid
     * endpoint-addition overflow. Only the two actually used rows are checked. */
    if ((scratch_address >= image_address &&
         scratch_address - image_address < image_size) ||
        (image_address > scratch_address &&
         image_address - scratch_address < required))
        return 0U;

    previous = scratch;
    current = scratch + cols;
    memcpy(previous, image, cols);
    memcpy(current, image + cols, cols);
    for (uint16_t row = 1U; row < rows - 1U; ++row)
    {
        uint8_t *destination = image + (uint32_t)row * cols;
        const uint8_t *next = destination + cols;
        for (uint16_t col = 1U; col < cols - 1U; ++col)
        {
            uint32_t sum = (uint32_t)previous[col - 1U] + previous[col] + previous[col + 1U] +
                           current[col - 1U] + current[col] + current[col + 1U] +
                           next[col - 1U] + next[col] + next[col + 1U];
            destination[col] = (uint8_t)(sum / 9U);
        }
        /* Save the next unfiltered row before it becomes the destination;
         * the other scratch row already holds this row's original pixels. */
        uint8_t *recycled = previous;
        previous = current;
        current = recycled;
        if (row + 1U < rows - 1U)
            memcpy(current, next, cols);
    }
    return 1U;
}

uint8_t image_vertical_median3(uint8_t *image, uint16_t rows, uint16_t cols,
                               uint8_t *scratch, uint32_t scratch_size)
{
    const uint32_t required = (uint32_t)cols * 2U;
    const uint32_t image_size = (uint32_t)rows * cols;
    const uintptr_t image_address = (uintptr_t)image;
    const uintptr_t scratch_address = (uintptr_t)scratch;
    if (image == NULL || scratch == NULL || rows < 3U || cols == 0U ||
        scratch_size < required)
        return 0U;
    if ((scratch_address >= image_address &&
         scratch_address - image_address < image_size) ||
        (image_address > scratch_address &&
         image_address - scratch_address < required))
        return 0U;

    uint8_t *previous = scratch;
    uint8_t *current = scratch + cols;
    memcpy(previous, image, cols);
    memcpy(current, image + cols, cols);
    for (uint16_t row = 1U; row + 1U < rows; ++row)
    {
        const uint8_t *next = image + (uint32_t)(row + 1U) * cols;
        uint8_t *output = image + (uint32_t)row * cols;
        for (uint16_t col = 0U; col < cols; ++col)
            output[col] = image_median3_value(previous[col], current[col],
                                              next[col]);
        uint8_t *recycled = previous;
        previous = current;
        current = recycled;
        if (row + 1U < rows - 1U)
            memcpy(current, next, cols);
    }
    return 1U;
}

static uint8_t image_median9(uint8_t values[9])
{
    for (uint8_t index = 1U; index < 9U; ++index)
    {
        const uint8_t value = values[index];
        uint8_t position = index;
        while (position > 0U && values[position - 1U] > value)
        {
            values[position] = values[position - 1U];
            --position;
        }
        values[position] = value;
    }
    return values[4];
}

static uint8_t image_near_color_average(const uint8_t values[9],
                                        uint8_t reference,
                                        uint8_t color_radius,
                                        uint8_t *sample_count)
{
    uint16_t sum = 0U;
    uint8_t count = 0U;
    for (uint8_t index = 0U; index < 9U; ++index)
    {
        const uint8_t value = values[index];
        const uint8_t difference = value > reference ?
            (uint8_t)(value - reference) : (uint8_t)(reference - value);
        if (difference <= color_radius)
        {
            sum += value;
            ++count;
        }
    }
    *sample_count = count;
    return count == 0U ? reference :
        (uint8_t)((sum + count / 2U) / count);
}

uint8_t image_near_color_denoise3x3(uint8_t *image, uint16_t rows,
                                    uint16_t cols, uint8_t color_radius,
                                    uint8_t *scratch, uint32_t scratch_size)
{
    const uint32_t required = (uint32_t)cols * 2U;
    const uint32_t image_size = (uint32_t)rows * cols;
    const uintptr_t image_address = (uintptr_t)image;
    const uintptr_t scratch_address = (uintptr_t)scratch;
    if (image == NULL || scratch == NULL || rows < 3U || cols < 3U ||
        scratch_size < required)
        return 0U;
    if ((scratch_address >= image_address &&
         scratch_address - image_address < image_size) ||
        (image_address > scratch_address &&
         image_address - scratch_address < required))
        return 0U;

    uint8_t *previous = scratch;
    uint8_t *current = scratch + cols;
    memcpy(previous, image, cols);
    memcpy(current, image + cols, cols);
    for (uint16_t row = 1U; row + 1U < rows; ++row)
    {
        const uint8_t *next = image + (uint32_t)(row + 1U) * cols;
        uint8_t *output = image + (uint32_t)row * cols;
        for (uint16_t col = 1U; col + 1U < cols; ++col)
        {
            uint8_t values[9];
            uint8_t value_index = 0U;
            for (int16_t dx = -1; dx <= 1; ++dx)
            {
                values[value_index++] = previous[(int32_t)col + dx];
                values[value_index++] = current[(int32_t)col + dx];
                values[value_index++] = next[(int32_t)col + dx];
            }
            uint8_t count = 0U;
            uint8_t filtered = image_near_color_average(
                values, current[col], color_radius, &count);
            if (count < 4U)
            {
                uint8_t ordered[9];
                memcpy(ordered, values, sizeof(ordered));
                const uint8_t median = image_median9(ordered);
                filtered = image_near_color_average(
                    values, median, color_radius, &count);
            }
            output[col] = filtered;
        }
        uint8_t *recycled = previous;
        previous = current;
        current = recycled;
        if (row + 1U < rows - 1U)
            memcpy(current, next, cols);
    }
    return 1U;
}

uint8_t image_binary_despeckle3x3(uint8_t *image, uint16_t rows,
                                  uint16_t cols, uint8_t *scratch,
                                  uint32_t scratch_size)
{
    if (image == NULL || scratch == NULL || rows < 3U || cols < 3U ||
        scratch_size < (uint32_t)cols * 2U)
        return 0U;

    uintptr_t image_address = (uintptr_t)image;
    uintptr_t scratch_address = (uintptr_t)scratch;
    uint32_t image_size = (uint32_t)rows * cols;
    uint32_t scratch_needed = (uint32_t)cols * 2U;
    if ((scratch_address >= image_address &&
         scratch_address - image_address < image_size) ||
        (image_address > scratch_address &&
         image_address - scratch_address < scratch_needed))
        return 0U;

    uint8_t *previous = scratch;
    uint8_t *current = scratch + cols;
    memcpy(previous, image, cols);
    memcpy(current, image + cols, cols);
    for (uint16_t row = 1U; row + 1U < rows; ++row)
    {
        const uint8_t *next = image + (uint32_t)(row + 1U) * cols;
        uint8_t *output = image + (uint32_t)row * cols;
        for (uint16_t col = 1U; col + 1U < cols; ++col)
        {
            uint8_t white = 0U;
            for (int16_t dx = -1; dx <= 1; ++dx)
            {
                white += previous[(int32_t)col + dx] != 0U;
                white += current[(int32_t)col + dx] != 0U;
                white += next[(int32_t)col + dx] != 0U;
            }
            if (current[col] != 0U && white == 1U)
                output[col] = 0U;
            else if (current[col] == 0U && white == 8U)
                output[col] = 255U;
            else
                output[col] = current[col];
        }
        uint8_t *recycled = previous;
        previous = current;
        current = recycled;
        if (row + 1U < rows - 1U)
            memcpy(current, next, cols);
    }
    return 1U;
}


uint8_t image_banded_percentile_stretch(uint8_t *image, uint16_t rows,
                                        uint16_t cols, uint16_t band_rows,
                                        uint8_t low_percent,
                                        uint8_t high_percent,
                                        uint32_t *histogram)
{
    if (image == NULL || histogram == NULL || rows == 0U || cols == 0U ||
        band_rows == 0U || low_percent >= high_percent || high_percent > 100U)
        return 0U;

    uint16_t first = 0U;
    uint16_t count = band_rows < rows ? band_rows : rows;
    uint16_t center = image_band_center(first, count);
    uint8_t low;
    uint8_t high;
    image_band_percentiles(image, count, cols, low_percent, high_percent,
                           histogram, &low, &high);
    uint16_t previous_center = center;
    uint8_t previous_low = low;
    uint8_t previous_high = high;

    while (first < rows)
    {
        const uint16_t next_first = (uint16_t)(first + count);
        uint16_t next_count = 0U;
        uint16_t next_center = center;
        uint8_t next_low = low;
        uint8_t next_high = high;
        if (next_first < rows)
        {
            next_count = band_rows;
            if ((uint32_t)next_first + next_count > rows)
                next_count = (uint16_t)(rows - next_first);
            next_center = image_band_center(next_first, next_count);
            image_band_percentiles(image + (uint32_t)next_first * cols,
                                   next_count, cols, low_percent, high_percent,
                                   histogram, &next_low, &next_high);
        }

        for (uint16_t row = first; row < next_first; ++row)
        {
            uint8_t row_low = low;
            uint8_t row_high = high;
            if (row < center && center > previous_center)
            {
                row_low = image_interpolate_u8(previous_low, low,
                                               (uint16_t)(row - previous_center),
                                               (uint16_t)(center - previous_center));
                row_high = image_interpolate_u8(previous_high, high,
                                                (uint16_t)(row - previous_center),
                                                (uint16_t)(center - previous_center));
            }
            else if (row > center && next_center > center)
            {
                row_low = image_interpolate_u8(low, next_low,
                                               (uint16_t)(row - center),
                                               (uint16_t)(next_center - center));
                row_high = image_interpolate_u8(high, next_high,
                                                (uint16_t)(row - center),
                                                (uint16_t)(next_center - center));
            }
            if ((uint16_t)row_high <= (uint16_t)row_low + 1U)
                continue;
            uint8_t *line = image + (uint32_t)row * cols;
            for (uint16_t col = 0U; col < cols; ++col)
            {
                uint16_t value = line[col];
                if (value <= row_low)
                    value = 0U;
                else if (value >= row_high)
                    value = 255U;
                else
                    value = (uint16_t)(((uint32_t)(value - row_low) * 255U +
                                        (row_high - row_low) / 2U) /
                                       (row_high - row_low));
                line[col] = (uint8_t)value;
            }
        }

        previous_center = center;
        previous_low = low;
        previous_high = high;
        first = next_first;
        count = next_count;
        center = next_center;
        low = next_low;
        high = next_high;
    }
    return 1U;
}

/*
 *@func: 二值化函数
 */
void binaryzation_process(uint8_t* _img,
                          uint16_t _rows,
                          uint16_t _cols,
                          uint32_t _threshold_value)
{
	uint32_t _size = (uint32_t)_rows * _cols;
	for (uint32_t _i = 0; _i < _size; ++_i)
	{
		if (*(_img + _i) > _threshold_value)
			*(_img + _i) = 255;
		else
			*(_img + _i) = 0;
	}
}

/*
 *@func: 获取直方图
 */
void get_hist_gram(const uint8_t* _img,
                   uint16_t _rows,
                   uint16_t _cols,
                   uint32_t* _hist_gram)
{
	memset(_hist_gram, 0, 256 * sizeof(*_hist_gram));
    uint32_t _size = (uint32_t)_rows * _cols;
    for (uint32_t _i = 0; _i < _size; ++_i)
    {
        _hist_gram[_img[_i]]++;
    }
}

/*
 *@func: 大津法求阈值
 */
uint8_t get_threshold_otsu(const uint32_t* _hist_gram)
{
    int Y;
    uint32_t amount = 0, pixel_back = 0, pixel_fore = 0;
    uint64_t pixel_integral_back = 0, pixel_integral_fore = 0, pixel_integral = 0;
    double OmegaBack, OmegaFore, MicroBack, MicroFore, SigmaB, Sigma; // 类间方差;
    int pixel_min_value, pixel_max_value;
    int threshold = 0;

    for (pixel_min_value = 0; pixel_min_value < 256 && _hist_gram[pixel_min_value] == 0; pixel_min_value++)
        ;
    if (pixel_min_value == 256)
        return 0; // 空直方图，不进行除法或访问越界灰度项
    for (pixel_max_value = 255; pixel_max_value > pixel_min_value && _hist_gram[pixel_max_value] == 0; pixel_max_value--)
        ;
    if (pixel_max_value == pixel_min_value)
        return (uint8_t)pixel_max_value; // 图像中只有一个颜色
    if (pixel_min_value + 1 == pixel_max_value)
        return (uint8_t)pixel_min_value; // 图像中只有二个颜色

    for (Y = pixel_min_value; Y <= pixel_max_value; Y++)
        amount += _hist_gram[Y]; //  像素总数

    pixel_integral = 0;
    for (Y = pixel_min_value; Y <= pixel_max_value; Y++)
        pixel_integral += (uint64_t)_hist_gram[Y] * (uint32_t)Y;
    SigmaB = -1;
    for (Y = pixel_min_value; Y < pixel_max_value; Y++)
    {
        pixel_back = pixel_back + _hist_gram[Y];
        pixel_fore = amount - pixel_back;
        pixel_integral_back += (uint64_t)_hist_gram[Y] * (uint32_t)Y;
        if (pixel_back == 0 || pixel_fore == 0)
            continue;
        OmegaBack = (double)pixel_back / amount;
        OmegaFore = (double)pixel_fore / amount;
        pixel_integral_fore = pixel_integral - pixel_integral_back;
        MicroBack = (double)pixel_integral_back / pixel_back;
        MicroFore = (double)pixel_integral_fore / pixel_fore;
        Sigma = OmegaBack * OmegaFore * (MicroBack - MicroFore) * (MicroBack - MicroFore);
        if (Sigma > SigmaB)
        {
            SigmaB = Sigma;
            threshold = Y;
        }
    }
    return (uint8_t)threshold;
}

uint8_t image_banded_otsu_binary_offset(uint8_t *image, uint16_t rows,
                                        uint16_t cols, uint16_t band_rows,
                                        int16_t threshold_offset,
                                        uint32_t *histogram)
{
    if (image == NULL || histogram == NULL || rows == 0U || cols == 0U ||
        band_rows == 0U)
        return 0U;

    uint16_t first = 0U;
    uint16_t count = band_rows < rows ? band_rows : rows;
    uint16_t center = image_band_center(first, count);
    get_hist_gram(image, count, cols, histogram);
    int32_t raw_threshold = (int32_t)get_threshold_otsu(histogram) +
                            threshold_offset;
    if (raw_threshold < 1)
        raw_threshold = 1;
    else if (raw_threshold > 254)
        raw_threshold = 254;
    uint8_t threshold = (uint8_t)raw_threshold;
    uint16_t previous_center = center;
    uint8_t previous_threshold = threshold;
    uint32_t threshold_sum = 0U;

    while (first < rows)
    {
        const uint16_t next_first = (uint16_t)(first + count);
        uint16_t next_count = 0U;
        uint16_t next_center = center;
        uint8_t next_threshold = threshold;
        if (next_first < rows)
        {
            next_count = band_rows;
            if ((uint32_t)next_first + next_count > rows)
                next_count = (uint16_t)(rows - next_first);
            next_center = image_band_center(next_first, next_count);
            get_hist_gram(image + (uint32_t)next_first * cols, next_count,
                          cols, histogram);
            raw_threshold = (int32_t)get_threshold_otsu(histogram) +
                            threshold_offset;
            if (raw_threshold < 1)
                raw_threshold = 1;
            else if (raw_threshold > 254)
                raw_threshold = 254;
            next_threshold = (uint8_t)raw_threshold;
        }

        for (uint16_t row = first; row < next_first; ++row)
        {
            uint8_t row_threshold = threshold;
            if (row < center && center > previous_center)
                row_threshold = image_interpolate_u8(
                    previous_threshold, threshold,
                    (uint16_t)(row - previous_center),
                    (uint16_t)(center - previous_center));
            else if (row > center && next_center > center)
                row_threshold = image_interpolate_u8(
                    threshold, next_threshold, (uint16_t)(row - center),
                    (uint16_t)(next_center - center));
            binaryzation_process(image + (uint32_t)row * cols, 1U, cols,
                                 row_threshold);
            threshold_sum += row_threshold;
        }

        previous_center = center;
        previous_threshold = threshold;
        first = next_first;
        count = next_count;
        center = next_center;
        threshold = next_threshold;
    }
    return (uint8_t)((threshold_sum + rows / 2U) / rows);
}

void auxiliary_process(const uint8_t* _src_pixel_mat,
                       uint16_t _src_rows, uint16_t _src_cols,
                       uint8_t _threshold_val,
                       uint16_t* _left_line, uint16_t* _mid_line, uint16_t* _right_line)
{
    if (_src_rows == 0 || _src_cols == 0)
        return;
    uint16_t mid_point = _src_cols >> 1;
    uint32_t bottom_row = (uint32_t)(_src_rows - 1U) * _src_cols;
    if (_src_pixel_mat[bottom_row + mid_point] < _threshold_val)
    {
        // 重新寻找扫线中点
        uint16_t _tmp_left_mid = 0;
        uint16_t _tmp_right_mid = _src_cols;
        for (uint16_t j = (_src_cols >> 1); (uint32_t)j + 2U < _src_cols; ++j)
        {
            if (_src_pixel_mat[bottom_row + j] > _threshold_val &&
                _src_pixel_mat[bottom_row + j + 1U] > _threshold_val &&
                _src_pixel_mat[bottom_row + j + 2U] > _threshold_val)
            {
                _tmp_right_mid = j + 2;
            }
        }

        for (uint16_t j = (_src_cols >> 1); j > 2U; --j)
        {
            if (_src_pixel_mat[bottom_row + j] > _threshold_val &&
                _src_pixel_mat[bottom_row + j - 1U] > _threshold_val &&
                _src_pixel_mat[bottom_row + j - 2U] > _threshold_val)
            {
                _tmp_left_mid = j - 2;
            }
        }
        if (_tmp_right_mid - (_src_cols >> 1) < (_src_cols >> 1) - _tmp_left_mid)
        {
            mid_point = _tmp_right_mid;
        }
        else
        {
            mid_point = _tmp_left_mid;
        }
    }
    // 未找到白色起点时仍使用合法列，避免将列数哨兵当作像素坐标。
    if (mid_point >= _src_cols)
        mid_point = _src_cols >> 1;
    for (uint16_t row = _src_rows; row > 0; --row)
    {
        uint16_t i = row - 1U;
        uint32_t row_offset = (uint32_t)i * _src_cols;
        uint16_t cur_point = mid_point;
        _left_line[i] = 0;
        _right_line[i] = _src_cols - 1U;
       // 扫描左线
        while (cur_point > 2U)
        {
            if (_src_pixel_mat[row_offset + cur_point] < _threshold_val &&
                _src_pixel_mat[row_offset + cur_point - 1U] < _threshold_val &&
                _src_pixel_mat[row_offset + cur_point - 2U] < _threshold_val)
            {
                _left_line[i] = cur_point;
                break;
            }
            --cur_point;
        }
        // 扫描右线
        cur_point = mid_point;
        while ((uint32_t)cur_point + 2U < _src_cols)
        {
            if (_src_pixel_mat[row_offset + cur_point] < _threshold_val &&
                _src_pixel_mat[row_offset + cur_point + 1U] < _threshold_val &&
                _src_pixel_mat[row_offset + cur_point + 2U] < _threshold_val)
            {
                _right_line[i] = cur_point;
                break;
            }
            ++cur_point;
        }

        _mid_line[i] = (uint16_t)(((uint32_t)_right_line[i] + _left_line[i]) >> 1);
        if (_left_line[i] != 0 || 
			_right_line[i] != _src_cols - 1)
        {
            mid_point = _mid_line[i];
        }
        else
        {
            mid_point = _src_cols >> 1;
        }
    }
}

