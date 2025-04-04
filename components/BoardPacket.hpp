#ifndef BOARDPACKET_HPP
#define BOARDPACKET_HPP

#include "Packet.hpp"
#include "CanMsgHandler.hpp"

#define BOARD_CHANNEL_NUM 2

enum BoardPacketChannelIdType
{
    CIT_Channel0 = 0x120,
    CIT_Channel1 = 0x130,
};

class BoardPacket : public Packet
{
private:
    BoardPacketChannelIdType m_ChannelCanId;

public:
    BoardPacket(){}

    virtual void Init(BoardPacketChannelIdType _channelCanId, uint8_t _id)
    {
        Packet::Init(_id);
        m_ChannelCanId = _channelCanId;

        Registration();
    }

    virtual void Registration();
    virtual void SendPacket();
};

class TestBoardPacket : public BoardPacket
{
public:
	TestBoardPacket(){}
    uint16_t m_testData;
    float m_Vxfdb;
    float m_Vyfdb;
    uint8_t color;
    uint8_t shooterId;
    uint8_t robotLevel;
    float bulletSpeed;
    float cap_energy;
    float chassis_power;
    uint8_t shoot_freq;
    uint16_t bullet_remain_num;
    uint16_t stage_remain_time;
    uint16_t max_heat;
    uint16_t cooling_rate;

    virtual void Init(BoardPacketChannelIdType _channelCanId, uint8_t _id)
    {
        m_testData = 0;
        BoardPacket::Init(_channelCanId, _id);
    }

    virtual void OnPacketReceived();
    virtual void SendPacket();
    template<typename Stream> void SerializePacket(Stream &stream);
};

class BoardPacketManager : public PacketManager, public CanMsgHandler
{
private:
    StreamChannel m_StreamChannel[BOARD_CHANNEL_NUM];
    CAN_TypeDef *m_pCan;

    TestBoardPacket m_testPacket;

protected:
    virtual bool FlushSendBufferLow();

public:
    BoardPacketManager();

    TestBoardPacket& GetTestPacket() { return m_testPacket; }
	
    virtual void Update();
    virtual void Init();
    virtual void HandleNewCanRxMsg(CanRxMsg *_msg);

    static BoardPacketManager *Instance()
    {
        static BoardPacketManager instance;
        return &instance;
    }
};

#endif
