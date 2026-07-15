#include "VisionPositionMap.h"

uint8_t VisionPosition_GetPlace(uint8_t vision_pos, box_t *place)
{
    static const box_t place_table[VISION_POSITION_MAX] =
    {
        BOX_PLACE_4,
        BOX_PLACE_5,
        BOX_PLACE_6,
        BOX_PLACE_7,
        BOX_PLACE_8,
    };

    if (place == 0 || vision_pos < VISION_POSITION_MIN || vision_pos > VISION_POSITION_MAX)
    {
        return 0U;
    }

    *place = place_table[vision_pos - VISION_POSITION_MIN];
    place->z_counts = Z_PLACE;  /* MATCH 只选择放置点，不能改变统一放置高度。 */
    return 1U;
}
