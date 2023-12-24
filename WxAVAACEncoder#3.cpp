////////////////////////////////////////////////////////////////////////////////////////////////////
//  COPYRIGHT © 2023 by WSI Corporation
////////////////////////////////////////////////////////////////////////////////////////////////////
/// \author 

#include "WxKernelAfx.h"
#include "Shared/Video/WxAVAACEncoder.h"

#include "Renderer/Movie/WxrFFMPEGEncoder.h"

namespace
{
    const std::array<wxuint, 13> sSamplingRateList = {
        96000, 88200, 64000, 48000, 44100, 32000, 24000, 22050, 16000, 12000, 11025, 8000, 7350
    };
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// WxBitWriter
////////////////////////////////////////////////////////////////////////////////////////////////////

struct WxBitWriter
{
    WxBitWriter(wxbyte* pData) : m_pData(pData)
    {}

    void PushBits(const wxuchar8 bitCnt, const wxuint32 value)
    {
        constexpr wxuchar8 bitsPerByte = 8;
        constexpr wxuchar8 bitsPerInt32 = 8*4;

        // Not enough bits in this byte to write all the bits required.
        if (m_bitIndex + bitCnt > bitsPerByte)
        {
            const wxuchar8 remainderBitCnt = m_bitIndex + bitCnt - bitsPerByte;
            const wxuint32 remainderValue = value & (0xFFFFFFFFul >> (bitsPerInt32 - remainderBitCnt));

            m_pData[m_pos] |= (value >> remainderBitCnt);

            m_bitIndex = 0;
            m_pos++;

            // Process the remaining value in the next byte.
            PushBits(remainderBitCnt, remainderValue);

            return;
        }

        const wxuchar8 offset = bitsPerByte - m_bitIndex - bitCnt;
        m_pData[m_pos] |= (value << offset);
        m_bitIndex += bitCnt;

        if (m_bitIndex == bitsPerByte) {
            m_bitIndex = 0;
            m_pos++;
        }
    }

private:
    wxuint32 m_pos = 0;
    wxuchar8 m_bitIndex = 0;
    wxbyte* m_pData;
};

////////////////////////////////////////////////////////////////////////////////////////////////////
// WxAVAACEncoder
////////////////////////////////////////////////////////////////////////////////////////////////////

WxAVAACEncoder::WxAVAACEncoder(const wxuint targetBitRate) :
    m_sampleRate(48000),
    m_channelCnt(2),
    m_ADTSHeader(
            WxAVADTSHeader::eAAC_LC,
            WxAVADTSHeader::eStereo,
            m_sampleRate)
{
    m_rAudioEncoder = std::make_unique<WxrFFMPEGAudioEncoder>(
            WxrFFMPEGAudioEncoder::eAAC,
            WxrFFMPEGAudioEncoder::eS32Interleaved,
            WxrFFMPEGAudioEncoder::eStereo,
            m_sampleRate,
            targetBitRate);
}

////////////////////////////////////////////////////////////////////////////////////////////////////

WxAVAACEncoder::~WxAVAACEncoder()
{}

////////////////////////////////////////////////////////////////////////////////////////////////////

wxfloat WxAVAACEncoder::GetPPS() const
{
    if (m_rAudioEncoder == nullptr) return 0;

    return (wxfloat)(m_sampleRate)/(wxfloat)(GetEncodingFrameSampleSetCount());
}

////////////////////////////////////////////////////////////////////////////////////////////////////

wxuint WxAVAACEncoder::GetEncodingFrameSampleSetCount() const
{
    if (m_rAudioEncoder == nullptr) return 0;

    return m_rAudioEncoder->GetEncodingFrameSampleSetCount();
}

////////////////////////////////////////////////////////////////////////////////////////////////////

bool WxAVAACEncoder::Encode(
        const WxAVAudioSample* pAudioSamples)
{
    m_aacBuffer.clear();
    if (m_rAudioEncoder == nullptr || !m_rAudioEncoder->IsValid()) return false;

    const WxrFFMPEGAudioEncoder::EncodedAudioPacket encoded = 
            m_rAudioEncoder->Encode((const wxuchar8*)pAudioSamples);

    if (encoded.m_size == 0 || encoded.m_pData == nullptr)
    {
        return m_rAudioEncoder->IsValid();
    }

    const wxuint adtsFrameSize = WxAVADTSHeader::Size() + encoded.m_size;
    m_aacBuffer.resize(adtsFrameSize);
    WxBitWriter adtsBits(m_aacBuffer.data());
    m_ADTSHeader.Encode(adtsBits, adtsFrameSize);
    WxMemCopy(m_aacBuffer.data() + WxAVADTSHeader::Size(), encoded.m_pData, encoded.m_size);

    return m_rAudioEncoder->IsValid();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// WxAVADTSHeader
////////////////////////////////////////////////////////////////////////////////////////////////////

WxAVADTSHeader::WxAVADTSHeader(
        const Profile profile,
        const ChannelLayout channels,
        const wxuint samplingRate)
{
    if (profile == eAAC_LC) m_profile = 1;
    if (channels == eStereo) m_channelConfiguration = 2;

    m_samplingFrequencyIndex = std::distance(
            sSamplingRateList.begin(),
            std::find(sSamplingRateList.begin(), sSamplingRateList.end(), samplingRate));
}

////////////////////////////////////////////////////////////////////////////////////////////////////

wxuint WxAVADTSHeader::Size()
{ return 7; }

////////////////////////////////////////////////////////////////////////////////////////////////////

void WxAVADTSHeader::Encode(WxBitWriter& bitWriter, const wxuint audioFrameSize) const
{
    bitWriter.PushBits(12, 0xFFF); // Sync word
    bitWriter.PushBits(1, 0); // MPEG Version
    bitWriter.PushBits(2, 0); // Layer
    bitWriter.PushBits(1, 1); // Protection absent (no CRC)
    bitWriter.PushBits(2, m_profile); // Profile, the MPEG-4 Audio Object Type minus 1.
    bitWriter.PushBits(4, m_samplingFrequencyIndex); // MPEG-4 Sampling Frequency Index 
    bitWriter.PushBits(1, 0); // Private bit
    bitWriter.PushBits(3, m_channelConfiguration); // MPEG-4 Channel Configuration
    bitWriter.PushBits(1, 0); // Originality
    bitWriter.PushBits(1, 0); // Home
    bitWriter.PushBits(1, 0); // Copyright ID bit
    bitWriter.PushBits(1, 0); // Copyright ID start
    bitWriter.PushBits(13, audioFrameSize); // Frame length
    bitWriter.PushBits(11, 0x7FF); // Buffer fullness
    bitWriter.PushBits(2, 0); // Number of AAC frames in ADTS frame minus 1
}