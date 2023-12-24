////////////////////////////////////////////////////////////////////////////////////////////////////
//  COPYRIGHT © 2023 by WSI Corporation
////////////////////////////////////////////////////////////////////////////////////////////////////
/// \author 
#pragma once

// Wx includes.
#include "WxKernelDecl.h"
#include "Shared/Video/WxAVAudioStream.h"

// Wx includes.
#include "WxKernelDecl.h"

// Forward declarations.
class WxrFFMPEGAudioEncoder;
class WxBitWriter;

////////////////////////////////////////////////////////////////////////////////////////////////////
// WxAVADTSHeader
////////////////////////////////////////////////////////////////////////////////////////////////////

class WX_EXPORT_KERNEL WxAVADTSHeader
{
public:
    enum Profile
    {
        eAAC_LC
    };

    enum ChannelLayout
    {
        eStereo
    };

    WxAVADTSHeader(
            const Profile profile,
            const ChannelLayout channels,
            const wxuint samplingRate);
    ~WxAVADTSHeader()
    {}

    static wxuint Size();

    void Encode(WxBitWriter& bitWriter, const wxuint audioFrameSize) const;

private:
    wxbyte m_profile = 0;
    wxbyte m_samplingFrequencyIndex = 0;
    wxbyte m_channelConfiguration = 0;
};

////////////////////////////////////////////////////////////////////////////////////////////////////
// WxAVAACEncoder
////////////////////////////////////////////////////////////////////////////////////////////////////

class WX_EXPORT_KERNEL WxAVAACEncoder
{
public:
    WxAVAACEncoder(const wxuint targetBitRate);
    ~WxAVAACEncoder();

    bool IsValid() const 
    { return m_rAudioEncoder != nullptr; }

    // Encoded audio frames per second.
    wxfloat GetPPS() const;

    // It is mandatory to send this exact sample set count for encoding.
    wxuint GetEncodingFrameSampleSetCount() const;

    wxuint GetChannelCnt() const
    { return m_channelCnt; }

    bool Encode(const WxAVAudioSample* pAudioSamples);

    const std::vector<wxbyte>& GetEncodedFrame() const
    { return m_aacBuffer; }

private:
    wxuint m_channelCnt;
    wxuint m_sampleRate;

    std::unique_ptr<WxrFFMPEGAudioEncoder> m_rAudioEncoder;
    WxAVADTSHeader m_ADTSHeader;
    std::vector<wxbyte> m_aacBuffer;
};