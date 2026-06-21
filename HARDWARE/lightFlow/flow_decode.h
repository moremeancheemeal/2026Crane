#include <stdint.h>

#pragma pack(1)

typedef struct lilghtFlow
{
    float		flow_x; //2
    float		flow_y; //2
	float  	target_x;
	float 	target_y;
	float 	target_angle;
	float 	angle;
}lilghtFlow_t;

//UPIXELS纯光流协议
typedef struct optical_flow_data
{
    int16_t		flow_x_integral; //2
    int16_t		flow_y_integral; //2
    uint16_t   	integration_timespan; //2
    uint16_t   	ground_distance; //2
    uint8_t    	valid; //1
    uint8_t    	version; //1
}UpixelsOpticalFlow;


#pragma pack()

typedef enum {
    UPIXELS=0,
    UPIXELS_NO_TOF,
	MSP,
	MSP_NO_TOF,
	MAVLINK_APM,
	MAVLINK_PX4,
	MAVLINK_PX4_NO_TOF
}PROTOCOL;

extern UpixelsOpticalFlow up_flow_data;
extern lilghtFlow_t coor;

#ifdef __cplusplus
extern "C" {
#endif

int16_t upnotof_parse_char(uint8_t ch); //302GS使用的UPIXELS_NO_TOF纯光流协议
void setCoor(int16_t x, int16_t y);
void getCoor(int16_t* x, int16_t* y);
#ifdef __cplusplus
}
#endif
