//#ifdef LteSATMAC
#include "stack/phy/layer/LtePhyUeD2D.h"
#include "stack/phy/packet/SidelinkControlInformation_m.h"
#include "stack/phy/packet/SatmacPacket_m.h"
#include "stack/mac/packet/LteSchedulingGrant.h"
#include "stack/mac/allocator/LteAllocationModule.h"
#include "stack/phy/layer/Subchannel.h"
#include <unordered_map>
#include <stack/mac/packet/SatmacSchedulingGrant.h>
#include "common/SatmacCommon.h"
class SatPhy : public LtePhyUeD2D
{
protected:

    // D2D Tx Power
    int m_frame_len;
    int m_channel_num;
    double d2dTxPower_;

    bool adjacencyPSCCHPSSCH_;
    int pStep_;
    int numSubchannels_;
    int subchannelSize_ ;
    int selectionWindowStartingSubframe_;
    int thresholdRSSI_;
    int cbrCountDown_;

    bool transmitting_;

    std::map<MacNodeId, simtime_t> previousTransmissionTimes_;

    std::vector<int> ThresPSSCHRSRPvector_;

    std::vector<LteAirFrame*> tbFrames_; // airframes received in the current TTI. Only one will be decoded
    cMessage* d2dDecodingTimer_; // timer for triggering decoding at the end of the TTI. Started when the first airframe is received

    std::vector<std::vector<double>> tbRsrpVectors_;
    std::vector<std::vector<double>> tbRssiVectors_;
    std::vector<std::vector<double>> tbSinrVectors_;
    std::vector<double> tbAttenuations_;

    std::vector<std::vector<Subchannel*>> sensingWindow_;
    int sensingWindowFront_;
    SatmacSchedulingGrant* sciGrant_;
    SatmacSchedulingGrant* satmacGrant_;

    std::vector<std::vector<double>> sciRsrpVectors_;
    std::vector<std::vector<double>> sciRssiVectors_;
    std::vector<std::vector<double>> sciSinrVectors_;
    std::vector<double> sciAttenuations_;
    std::vector<LteAirFrame*> sciFrames_;
    std::vector<cPacket*> scis_;

    simsignal_t cbr;
    simsignal_t sciReceived;
    simsignal_t sciDecoded;
    simsignal_t sciSent;
    simsignal_t tbSent;
    simsignal_t tbReceived;
    simsignal_t tbDecoded;
    simsignal_t tbFailedDueToNoSCI;
    simsignal_t tbFailedButSCIReceived;
    simsignal_t tbAndSCINotReceived;
    simsignal_t sciFailedHalfDuplex;
    simsignal_t tbFailedHalfDuplex;
    simsignal_t threshold;
    simsignal_t txRxDistanceSCI;
    simsignal_t txRxDistanceTB;
    simsignal_t subchannelReceived;
    simsignal_t subchannelsUsed;
    simsignal_t senderID;
    simsignal_t subchannelSent;
    simsignal_t subchannelsUsedToSend;
    simsignal_t interPacketDelay;
    simsignal_t posX;
    simsignal_t posY;

    simsignal_t tbFailedDueToProp;
    simsignal_t tbFailedDueToInterference;
    simsignal_t sciFailedDueToProp;
    simsignal_t sciFailedDueToInterference;
    simsignal_t sciUnsensed;

    int sciReceived_;
    int sciDecoded_;
    int sciFailedHalfDuplex_;
    int tbReceived_;
    int tbDecoded_;
    int tbFailedDueToNoSCI_;
    int tbFailedButSCIReceived_;
    int tbAndSCINotReceived_;
    int tbFailedHalfDuplex_;
    int subchannelReceived_;
    int subchannelsUsed_;

    int tbFailedDueToProp_;
    int tbFailedDueToInterference_;
    int sciFailedDueToProp_;
    int sciFailedDueToInterference_;

    int sciUnsensed_;

    RbMap availableRBs_;
    RbMap fiRbs_;
    LteAllocationModule* allocator_;

    void storeAirFrame(LteAirFrame* newFrame);
    LteAirFrame* extractAirFrame();
    //void decodeAirFrame(LteAirFrame* frame, UserControlInfo* lteInfo, std::vector<double> &rsrpVector, std::vector<double> &rssiVector, std::vector<double> &sinrVector, double &attenuation);
    // ---------------------------------------------------------------- //

    virtual void initialize(int stage);
    virtual void finish();
    virtual void handleAirFrame(cMessage* msg);
    virtual void handleUpperMessage(cMessage* msg);
    virtual void handleSelfMessage(cMessage *msg);

    void decodeAirFrame(LteAirFrame* frame, UserControlInfo* lteInfo, std::vector<double> &rsrpVector, std::vector<double> &rssiVector, std::vector<double> &sinrVector, double &attenuation);

    // Helper function which prepares a frame for sending
    virtual LteAirFrame* prepareAirFrame(cMessage* msg, UserControlInfo* lteInfo);

    // Generate an SCI message corresponding to a Grant
    virtual SidelinkControlInformation* createSCIMessage();

    virtual void updateSubframe();

    // Compute Candidate Single Subframe Resources which the MAC layer can use for transmission

    //virtual std::vector<std::tuple<double, int, int>> selectBestRSSIs(std::unordered_map<int, std::set<int>> possibleCSRs, LteMode4SchedulingGrant* &grant, int totalPossibleCSRs);

    virtual std::tuple<int,int> decodeRivValue(SidelinkControlInformation* sci, UserControlInfo* sciInfo);

    virtual RbMap sendSciMessage(cMessage* sci, UserControlInfo* lteInfo);

    virtual void initialiseSensingWindow();

    virtual int translateIndex(int index);

  public:
    SatPhy();
    virtual ~SatPhy();

    virtual double getTxPwr(Direction dir = UNKNOWN_DIRECTION)
    {
        if (dir == D2D)
            return d2dTxPower_;
        return txPower_;
    }

};
//#endif
