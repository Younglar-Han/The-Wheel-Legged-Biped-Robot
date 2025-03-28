#include "IST8310.hpp"

uint8_t IST8310::Init()
{
    if(ist8310_init() == IST8310_NO_ERROR)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

void IST8310::Update()
{
    ist8310_read_mag(&m_mag_x, &m_mag_y, &m_mag_z);
}

