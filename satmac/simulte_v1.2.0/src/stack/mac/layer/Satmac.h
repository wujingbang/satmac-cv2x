//#ifdef SATMAC

//#include "ns3/data-rate.h"
//#include "ns3/nstime.h"
//#include "tdma-mac.h"
//#include "tdma-mac-low.h"
//#include "tdma-mac-queue.h"
#include "common/SatmacCommon.h"
#include <string>
//#include "ns3/vector.h"
#include "stack/mac/layer/LteMacUeRealisticD2D.h"
#include "stack/mac/packet/SatmacSchedulingGrant.h"
#include "stack/mac/packet/AskCoordinate_m.h"
#include "stack/phy/packet/Coordinate_m.h"
#include "corenetwork/deployer/LteDeployer.h"
#include <unordered_map>

#include "common/geohash/geohash.h"
#include "common/geohash/mbr_sumomap.h"
#include "stack/phy/packet/SatmacPacket_m.h"

//#include "ns3/output-stream-wrapper.h"
//#include "ns3/trace-helper.h"

class Satmac : public LteMacUeRealisticD2D
{
public:
  Satmac ();
  ~Satmac ();

  /**
   * \param packet packet to send
   * \param hdr header of packet to send.
   *
   * Store the packet in the internal queue until it
   * can be sent safely.
   */
  /**
   * \param slotTime the duration of a slot.
   *
   * It is a bad idea to call this method after RequestAccess or
   * one of the Notify methods has been invoked.
   */

protected:

    virtual void handleMessage(cMessage *msg);

    virtual void handleSelfMessage();

    virtual void initialize(int stage);

    virtual void macPduMake();

    virtual void flushHarqBuffers();

    //void parseUeTxConfig(cXMLElement* xmlConfig);

    //void parseCbrTxConfig(cXMLElement* xmlConfig);

    //virtual double calculateChannelOccupancyRatio(int period);

    //virtual int getNumAntennas();

    //virtual void handleHarq();

    void finish();
    double get_channel_utilization();
    /*
    * slot_tag and fi handle functions
    */
    void slotHandler ();
    /* Determining which slot will be selected as BCH. */
    int determine_BCH(bool strict); //修改
    pair<int,int> determine_BCH_GeoHash(bool strict);
    void show_slot_occupation(void);
    void recvFI(cMessage *msg);

    //void clear_Local_FI(int begin_slot, int end_slot, int slot_num);
    /* Translating the fi_local_ to a FI_packet transmitted */
    unsigned char* generate_Slot_Tag_Code(slot_tag *st);
    //void update_slot_tag(unsigned char* buffer,unsigned int &byte_pos,unsigned int &bit_pos, int slot_pos, unsigned long long recv_sti);
    //void set_cetain_slot_tag(int index, unsigned char busy,unsigned long long sti, unsigned char psf, unsigned char ptp);
    void generate_send_FI_packet(int channelNum);

    void send_Fi();

    //void update_slot_tag(unsigned char* buffer,unsigned int &byte_pos,unsigned int &bit_pos, int slot_pos, unsigned int recv_sti);
    Frame_info * get_new_FI(int slot_count);
    void fade_received_fi_list(int time);
    bool isNewNeighbor(int sid);
    bool isSingle(void);
    void synthesize_fi_list();
    void merge_fi(Frame_info* base, Frame_info* append, Frame_info* decision);
    void clear_FI(Frame_info *fi);
    void clear_others_slot_status();
    bool adjust_is_needed(int slot_num, int channel_num);
    void adjFrameLen();
    void print_slot_status(void);

    int selectSubChannel();

    int grantFlag = 0;

    MacCid macCid;
    LteMacQueue* m_queue;

    bool geoHash_NQueens;

    const char *positionX = NULL;
    const char *positionY = NULL;
    double x = 0.0;
    double y = 0.0;

    int minMCSPSSCH_;
    int maxMCSPSSCH_;
    int minSubchannelNumberPSSCH_;
    int maxSubchannelNumberPSSCH_;
    int allowedRetxNumberPSSCH_;
    int m_wifimaclow_flag;

    bool packetDropping_;

    std::vector<std::unordered_map<std::string, double>> cbrPSSCHTxConfigList_;
    std::vector<std::unordered_map<std::string, double>> cbrLevels_;

    simtime_t m_slotTime;
    simtime_t m_slotRemainTime;
    uint64_t m_lastpktUsedTime;

    UeInfo* ueInfo_;

    // if true, use the preconfigured TX params for transmission, else use that signaled by the eNB
    bool usePreconfiguredTxParams_;
    UserTxParams* preconfiguredTxParams_;
    UserTxParams* getPreconfiguredTxParams();  // build and return new user tx params
    void parseUeTxConfig(cXMLElement* xmlConfig);
    virtual double calculateChannelOccupancyRatio(int period);

    int numAntennas_;
    int missedTransmissions_;
    double mcsScaleD2D_;
    McsTable d2dMcsTable_;
    /// Local LteDeployer
    LteDeployer *deployer_;
    bool expiredGrant_;
    int reselectAfter_;
    int currentCbrIndex_;
    int defaultCbrIndex_;
    bool requestSdu;

    simsignal_t macPduCount;
    simsignal_t macRcvdSatmacPkt;
    simsignal_t macPduMakeCount;
    simsignal_t macPduCount0;
    simsignal_t remainPktInTheEnd;
    simsignal_t flushHarqCount1;
    simsignal_t flushHarqCount2;
    simsignal_t flushHarqCount3;
    simsignal_t flushHarqCount4;
    simsignal_t mcsCapacityRecord;
    simsignal_t pduLengthRecord;

    simsignal_t macpktSent;
    simsignal_t macRcvdDataPkt;
    simsignal_t grantStartTime;
    simsignal_t grantBreak;
    simsignal_t grantBreakTiming;
    simsignal_t grantBreakSize;
    simsignal_t droppedTimeout;
    simsignal_t grantBreakMissedTrans;
    simsignal_t missedTransmission;
    simsignal_t selectedMCS;
    simsignal_t selectedNumSubchannels;
    simsignal_t selectedSubchannelIndex;
    simsignal_t maximumCapacity;
    simsignal_t grantRequests;
    simsignal_t packetDropDCC;
    simsignal_t macNodeID;

    double remainingTime_;
    simtime_t receivedTime_;

    std::unordered_map<double, int> previousTransmissions_;

    ///////////////////////////////////////////////
    // SATMAC
    ///////////////////////////////////////////////
    // life time (frames) of a slot
    int slot_lifetime_frame_;
    // slot candidate count_3hop threshold
    int c3hop_threshold_;

    int delay_init_frame_num_;
    int random_bch_if_single_switch_;
    int choose_bch_random_switch_;

    int maximumCapacity_;
    Codeword currentCw_;

    bool fistAskPos = true;
    int askPosCounter = 10;
    int slot_num_;                      // The slot number it's allocated.
    int channel_num_;                   // 被分配到信道编号

    int slot_adj_candidate_;
    int channelNumber_adj;              // 当slot重新调整时，子信道也要根据当前位置调整
    int bch_slot_lock_;

    int slot_count_;
    long long total_slot_count_;
    bool dataPktExist;

    // How many packets has been sent out?
    static int tdma_ps_;
    static int tdma_pr_;

    //added variables
    int m_frame_len;
    int m_channel_num;
    int subchannelSize_;
    int numSubchannels_;
    double channelOccupancyRatio_;
    //int max_frame_len_;
    int global_sti;
    int global_psf;

    Frame_info *decision_fi_;
    Frame_info *collected_fi_;
    Frame_info *received_fi_list_;

    NodeState node_state_;
    SlotState slot_state_;
    int backoff_frame_num_;
    int enable;
    int vemac_mode_;
    int adj_ena_;
    int adj_free_threshold_;
    int adj_single_slot_ena_;
    int adj_frame_ena_;
    int adj_frame_lower_bound_;
    int adj_frame_upper_bound_;
    int slot_memory_;
    bool testmode_init_flag_;

    simtime_t last_log_time_;
    int collision_count_;
    int localmerge_collision_count_;
    int adj_count_total_;
    int adj_count_success_;
    int request_fail_times;
    int no_avalible_count_;
    int waiting_frame_count;
    int packet_sended;
    int packet_received;
    int frame_count_;
    int continuous_work_fi_;
    int continuous_work_fi_max_;

    int recv_fi_count_;
    int send_fi_count_;

    int m_start_delay_frames;

    double frameadj_cut_ratio_ths_;
    double frameadj_cut_ratio_ehs_;
    double frameadj_exp_ratio_;
    //#define FRAMEADJ_CUT_RATIO_THS 0.4
    //#define FRAMEADJ_CUT_RATIO_EHS 0.6
    //#define frameadj_exp_ratio_ 0.9

    bool location_initialed_;
    bool direction_initialed_;
    inet::Coord last_location_;
    inet::Coord curr_location_;
    int direction_;

    std::string m_traceOutFile;

};

//#endif




