#include "stack/mac/packet/LteSchedulingGrant.h"
#include "common/LteCommon.h"
#include "stack/mac/amc/UserTxParams.h"
#include "common/SatmacCommon.h"

class SatmacSchedulingGrant : public LteSchedulingGrant
{
protected:
    unsigned int numSubchannels;
    unsigned int startingSubchannel;
    unsigned int retransSubchannel;
    bool retransmission;
    bool firstTransmission;
    unsigned int timeGapTransRetrans;
    unsigned int mcs;

    int frameLength;
    int channelNumber;
    int globleSti;
    std::vector<std::vector<SlotTagForSending>> slotTag;

public:

    SatmacSchedulingGrant(const char *name = NULL, int kind = 0) :
        LteSchedulingGrant(name, kind)
    {
        numSubchannels = 0;
        timeGapTransRetrans = 0;
        startingSubchannel = 0;
        mcs = 0;
        retransSubchannel = 0;
        firstTransmission = true;
        frameLength = 0;
        channelNumber = 0;
        globleSti = 0;
    }


    ~SatmacSchedulingGrant()
    {
    }

    SatmacSchedulingGrant(const SatmacSchedulingGrant& other) :
        SatmacSchedulingGrant(other.getName())
    {
        operator=(other);
    }

    SatmacSchedulingGrant& operator=(const SatmacSchedulingGrant& other)
    {
        numSubchannels = other.numSubchannels;
        timeGapTransRetrans = other.timeGapTransRetrans;
        startingSubchannel = other.startingSubchannel;
        mcs = other.mcs;
        retransSubchannel = other.retransSubchannel;

        frameLength = other.frameLength;
        channelNumber = other.channelNumber;
        globleSti = other.globleSti;

        // slotTag
        slotTag = other.slotTag;
        LteSchedulingGrant::operator=(other);
        return *this;
    }

    virtual SatmacSchedulingGrant *dup() const
    {
        return new SatmacSchedulingGrant(*this);
    }

    void setNumberSubchannels(unsigned int subchannels)
    {
        numSubchannels = subchannels;
    }
    unsigned int getNumSubchannels() const
    {
        return numSubchannels;
    }

    void setTimeGapTransRetrans(unsigned int timeGapTransRetrans)
    {
        this->timeGapTransRetrans = timeGapTransRetrans;
    }
    unsigned int getTimeGapTransRetrans() const
    {
        return timeGapTransRetrans;
    }
    void setStartingSubchannel(unsigned int subchannelIndex)
    {
        this->startingSubchannel = subchannelIndex;
    }
    unsigned int getStartingSubchannel() const
    {
        return startingSubchannel;
    }
    void setMcs(unsigned int mcs)
    {
        this->mcs = mcs;
    }
    unsigned int getMcs() const
    {
        return mcs;
    }
    void setRetransSubchannel(unsigned int retransSubchannel)
    {
        this->retransSubchannel = retransSubchannel;
    }
    unsigned int getRetransSubchannel() const
    {
        return retransSubchannel;
    }
    void setRetransmission(bool retransmission)
    {
        this->retransmission = retransmission;
    }
    bool getRetransmission() const
    {
        return retransmission;
    }
    bool getFirstTransmission() const
    {
        return firstTransmission;
    }
    void setFirstTransmission(bool firstTransmission)
    {
        this->firstTransmission = firstTransmission;
    }

    int getFrameLength() const
    {
        return this->frameLength;
    }

    void setFrameLength(int frameLength)
    {
        this->frameLength = frameLength;
    }

    int getChannelNumber() const
    {
        return this->channelNumber;
    }

    void setChannelNumber(int channelNumber)
    {
        this->channelNumber = channelNumber;
    }

    int getGlobleSti() const
    {
        return this->globleSti;
    }

    void setGlobleSti(int globleSti)
    {
        this->globleSti = globleSti;
    }

    std::vector<std::vector<SlotTagForSending>>& getSlotTag()
    {
        return this->slotTag;
    }

    void setSlotTag(const std::vector<std::vector<slot_tag>>& slotTag)
    {
        //  std::vector<std::vector<SlotTagForSending>> SlotTag;
        //  SlotTag slotTag

        for(int i = 0;i < frameLength; i++)
        {
            std::vector<SlotTagForSending> v;
            for(int j = 0;j < channelNumber; j++)
            {
                SlotTagForSending stfs;
                stfs.busy = slotTag[i][j].busy;
                stfs.sti = slotTag[i][j].sti;
                stfs.count_2hop = slotTag[i][j].count_2hop;
                v.push_back(stfs);
            }
            this->slotTag.push_back(v);
        }
    }
};
