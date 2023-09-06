#ifndef INC_IO_H_
#define INC_IO_H_

#include "stdint.h" // for some reason I need this or I can't use uint8_t

// ENUM and TYPEDEF ===============================================================================

enum _DrivingMode {
	MODE_MANUAL, MODE_AUTO
};

enum _Gear {
	GEAR_P, GEAR_R, GEAR_N, GEAR_D
};

typedef struct {
	// READ ONLY VARIABLES
	enum _DrivingMode DrivingMode;
	enum _Gear Gear;
	int8_t DrivingDirection;
	uint8_t BrakeStatus;
	float Throttle;
} IOtypedef;

// FUNCTION PROTOTYPE =============================================================================

void IO_read_write(IOtypedef *var);
void IO_init_ADC_DMA();
float map(float value, int32_t m, int32_t n, int32_t x, int32_t y);

// ================================================================================================

#endif /* INC_IO_H_ */
