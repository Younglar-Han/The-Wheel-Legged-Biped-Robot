#ifndef STP23L_HPP
#define STP23L_HPP

#include "bsp_tof.h"
#include "Time.hpp"

#define ST_RX_QUEUE_LEN 250
#define ST_FRAME_BUFFER_LEN 250

class STP23L
{
private:
    uint8_t m_RxQuene[ST_RX_QUEUE_LEN];
    uint8_t m_FrameBuffer[ST_FRAME_BUFFER_LEN];
    uint8_t m_CurrentFrame[ST_FRAME_BUFFER_LEN];
    uint8_t m_RxQueueFront;
    uint8_t m_RxQueueRear;

    typedef struct
    {
        int16_t distance;
        uint16_t noise;
        uint32_t peak;
        uint8_t confidence;
        uint32_t intg;
        int16_t reftof;
    } LidarPointTypedef;

    uint32_t receive_cnt;
    uint8_t confidence;
    uint16_t distance, noise, reftof, last_distance;
    uint32_t peak, intg;

    LidarPointTypedef Pack_Data[12];
    LidarPointTypedef Pack_sum;

    uint8_t state;
    uint8_t crc;
    uint8_t cnt;
    uint8_t PACK_FLAG;
    uint8_t data_len;
    uint32_t timestamp;
    uint8_t state_flag;
    uint32_t lastUpdateTick;

    STP23L() : m_RxQueueFront(0), m_RxQueueRear(0), state(0), crc(0), cnt(0), PACK_FLAG(0), data_len(0), timestamp(0), state_flag(1) {}

public:
    uint8_t m_bsp_rxbuf[200];

    void Init();
    void Update();
    void DataProcess();

    uint16_t GetDistance() { return distance; }
    uint32_t GetCounter() { return receive_cnt; }

    static STP23L *Instance()
    {
        static STP23L instance;
        return &instance;
    }

    void Enqueue(uint8_t *_pData, uint32_t _len)
    {
        for (int i = 0; i < _len; ++i)
        {
            m_RxQuene[m_RxQueueRear] = _pData[i];
            m_RxQueueRear = (m_RxQueueRear + 1) % ST_RX_QUEUE_LEN;
        }
        lastUpdateTick = Time::GetTick();
    }

    bool IsTimeout() { return Time::GetTick() - lastUpdateTick > 500; }
};

#endif
