#include <stdint.h>
#include <string.h>
#include "flow_decode.h"
#include "check.h"

lilghtFlow_t coor;
UpixelsOpticalFlow up_flow_data;
//302GS使用的UPIXELS_NO_TOF纯光流协议
int16_t upnotof_parse_char(uint8_t ch)
{
	int16_t ret = -1;
    static int s  = 0, p = 0;
    static char Xor_r = 0x00, Xor_c = 0x00;

    switch(s) {
        case 0:
            if(ch == 0xFE)
                s = 1;
        break;
        case 1:
            if(ch == 0x0A) {
                s = 2;
                p = 0;
                Xor_c = 0x00;
            } else {
                s = 0;
            }
        break;
        case 2:
            ((char *)&up_flow_data)[p++] = ch;
            Xor_c ^= ch;
            if(p == 10) {
                s = 3;
                p = 0;
            }
        break;
        case 3:
            s = 4;
            Xor_r = ch;
        break;
        case 4:
            if(ch == 0x55) {
                if(Xor_r == Xor_c)
                    ret = 0;
            }
            s = 0;
        break;
        default:
        break;
    }
	return ret;
}

void setCoor(int16_t x, int16_t y)
{
	coor.target_x = x;
	coor.target_y = y;
}
void getCoor(int16_t* x, int16_t* y)
{
	*x = coor.flow_x;
	*y = coor.flow_y;
}
