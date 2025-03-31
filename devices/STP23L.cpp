#include "STP23L.hpp"

void STP23L::Init()
{
    bsp_tof_init(m_bsp_rxbuf);
}

void STP23L::Update()
{
    uint8_t _currentRear = m_RxQueueRear;
    while (m_RxQueueFront != _currentRear)
    {
        uint8_t _data = m_RxQuene[m_RxQueueFront];        
        if (state < 4)
        {
            if (_data == HEADER)
                state++;
            else
                state = 0;
        }
        else if (state < 10 && state > 3)
        {
            switch (state)
            {
            case 4:
                if (_data == device_address)
                {
                    state++;
                    crc = crc + _data;
                }
                else
                    state = 0, crc = 0;
                break;
            case 5:
                if (_data == PACK_GET_DISTANCE)
                {
                    PACK_FLAG = PACK_GET_DISTANCE;
                    state++;
                    crc = crc + _data;
                }
                else if (_data == PACK_RESET_SYSTEM)
                {
                    PACK_FLAG = PACK_RESET_SYSTEM;
                    state++;
                    crc = crc + _data;
                }
                else if (_data == PACK_STOP)
                {
                    PACK_FLAG = PACK_STOP;
                    state++;
                    crc = crc + _data;
                }
                else if (_data == PACK_ACK)
                {
                    PACK_FLAG = PACK_ACK;
                    state++;
                    crc = crc + _data;
                }
                else if (_data == PACK_VERSION)
                {
                    PACK_FLAG = PACK_VERSION,
                    state++,
                    crc = crc + _data;
                }
                else
                    state = 0, crc = 0;
                break;
            case 6:
                if (_data == chunk_offset)
                {
                    state++;
                    crc = crc + _data;
                }
                else
                    state = 0, crc = 0;
                break;
            case 7:
                if (_data == chunk_offset)
                {
                    state++;
                    crc = crc + _data;
                }
                else
                    state = 0, crc = 0;
                break;
            case 8:
                data_len = (uint16_t)_data;
                state++;
                crc = crc + _data;
                break;
            case 9:
                data_len = data_len + ((uint16_t)_data << 8);
                state++;
                crc = crc + _data;
                break;
            default:
                break;
            }
        }
        else if (state == 10)
            state_flag = 0;
        if (PACK_FLAG == PACK_GET_DISTANCE && state_flag == 0)
        {
            if (state > 9)
            {
                if (state < 190)
                {
                    static uint8_t state_num;
                    state_num = (state - 10) % 15;
                    switch (state_num)
                    {
                    case 0:
                        Pack_Data[cnt].distance = (uint16_t)_data;
                        crc = crc + _data;
                        state++;
                        break;
                    case 1:
                        Pack_Data[cnt].distance = ((uint16_t)_data << 8) + Pack_Data[cnt].distance;
                        crc = crc + _data;
                        state++;
                        break;
                    case 2:
                        Pack_Data[cnt].noise = (uint16_t)_data;
                        crc = crc + _data;
                        state++;
                        break;
                    case 3:
                        Pack_Data[cnt].noise = ((uint16_t)_data << 8) + Pack_Data[cnt].noise;
                        crc = crc + _data;
                        state++;
                        break;
                    case 4:
                        Pack_Data[cnt].peak = (uint32_t)_data;
                        crc = crc + _data;
                        state++;
                        break;
                    case 5:
                        Pack_Data[cnt].peak = ((uint32_t)_data << 8) + Pack_Data[cnt].peak;
                        crc = crc + _data;
                        state++;
                        break;
                    case 6:
                        Pack_Data[cnt].peak = ((uint32_t)_data << 16) + Pack_Data[cnt].peak;
                        crc = crc + _data;
                        state++;
                        break;
                    case 7:
                        Pack_Data[cnt].peak = ((uint32_t)_data << 24) + Pack_Data[cnt].peak;
                        crc = crc + _data;
                        state++;
                        break;
                    case 8:
                        Pack_Data[cnt].confidence = _data;
                        crc = crc + _data;
                        state++;
                        break;
                    case 9:
                        Pack_Data[cnt].intg = (uint32_t)_data;
                        crc = crc + _data;
                        state++;
                        break;
                    case 10:
                        Pack_Data[cnt].intg = ((uint32_t)_data << 8) + Pack_Data[cnt].intg;
                        crc = crc + _data;
                        state++;
                        break;
                    case 11:
                        Pack_Data[cnt].intg = ((uint32_t)_data << 16) + Pack_Data[cnt].intg;
                        crc = crc + _data;
                        state++;
                        break;
                    case 12:
                        Pack_Data[cnt].intg = ((uint32_t)_data << 24) + Pack_Data[cnt].intg;
                        crc = crc + _data;
                        state++;
                        break;
                    case 13:
                        Pack_Data[cnt].reftof = (int16_t)_data;
                        crc = crc + _data;
                        state++;
                        break;
                    case 14:
                        Pack_Data[cnt].reftof = ((int16_t)_data << 8) + Pack_Data[cnt].reftof;
                        crc = crc + _data;
                        state++;
                        cnt++;
                        break;
                    default:
                        break;
                    }
                }
                if (state == 190)
                    timestamp = _data, state++, crc = crc + _data;
                else if (state == 191)
                    timestamp = ((uint32_t)_data << 8) + timestamp, state++, crc = crc + _data;
                else if (state == 192)
                    timestamp = ((uint32_t)_data << 16) + timestamp, state++, crc = crc + _data;
                else if (state == 193)
                    timestamp = ((uint32_t)_data << 24) + timestamp, state++, crc = crc + _data;
                else if (state == 194)
                {
                    if (_data == crc)
                    {
                        DataProcess();
                        receive_cnt++;
                    }
                    distance = Pack_Data[0].distance;
                    crc = 0;
                    state = 0;
                    state_flag = 1;
                    cnt = 0;
                }
                else if (state > 194)
                {
                    crc = 0;
                    state = 0;
                    state_flag = 1;
                    cnt = 0;
                }
            }
        }
        m_RxQueueFront = (m_RxQueueFront + 1) % ST_RX_QUEUE_LEN;
    }
}


void STP23L::DataProcess(void)
{
    uint8_t i;
    static uint16_t count = 0;
    LidarPointTypedef Pack_sum;
    for (i = 0; i < 12; i++)
    {
        if (Pack_Data[i].distance != 0)
        {
            count++;
            Pack_sum.distance += Pack_Data[i].distance;
            Pack_sum.noise += Pack_Data[i].noise;
            Pack_sum.peak += Pack_Data[i].peak;
            Pack_sum.confidence += Pack_Data[i].confidence;
            Pack_sum.intg += Pack_Data[i].intg;
            Pack_sum.reftof += Pack_Data[i].reftof;
        }
    }
    if (count != 0)
    {
        last_distance = distance;
        distance = Pack_sum.distance / count;
        if (distance > 7000 || distance < 70)
            distance = last_distance;
        noise = Pack_sum.noise / count;
        peak = Pack_sum.peak / count;
        confidence = Pack_sum.confidence / count;
        intg = Pack_sum.intg / count;
        reftof = Pack_sum.reftof / count;
        Pack_sum.distance = 0;
        Pack_sum.noise = 0;
        Pack_sum.peak = 0;
        Pack_sum.confidence = 0;
        Pack_sum.intg = 0;
        Pack_sum.reftof = 0;
        count = 0;
    }

}
