#ifndef IST8310_HPP
#define IST8310_HPP

#include "bsp_ist8310.h"

class IST8310
{
public:
    IST8310():data_buffer{0}{;}
    ~IST8310(){;}

    uint8_t Init();
	void Update();
	
	uint8_t data_buffer[6];

	float m_mag_x;
	float m_mag_y;
	float m_mag_z;
};

#endif
