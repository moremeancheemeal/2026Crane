#ifndef __VISION_POSITION_MAP_H
#define __VISION_POSITION_MAP_H

#include "MapCoords.h"
#include <stdint.h>

#define VISION_POSITION_MIN   1U
#define VISION_POSITION_MAX   5U

uint8_t VisionPosition_GetPlace(uint8_t vision_pos, box_t *place);

#endif /* __VISION_POSITION_MAP_H */
