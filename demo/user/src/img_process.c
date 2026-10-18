/*
 *@第十九届东南大学智能车竞赛基础四轮组部分开源学习代码
 */


#include "img_process.h"
#include "string.h"

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

