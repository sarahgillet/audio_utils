#include "AlsaPcmDevice.h"

#include <audio_utils/AudioFrame.h>

#include <ros/ros.h>

#include <cstdlib>
#include <iostream>
#include <sstream>
// example to run: rosrun audio_utils alsa_capture_node _device:="hw:CARD=0,DEV=0" _format:="signed_16" _channel_count:=2 _sampling_frequency:=44100 _frame_sample_count:=1000

using namespace introlab;
using namespace std;

void mergeChannels(const PcmAudioFrame& pcmInput, PcmAudioFrame& pcmOutput, float gain)
{
    AudioFrame<float> input = pcmInput;
    AudioFrame<float> output(1, input.sampleCount());

    for (size_t sample = 0; sample < input.sampleCount(); sample++)
    {
        output[sample] = 0;
        for (size_t channel = 0; channel < input.channelCount(); channel++)
        {
            output[sample] += input[channel * input.sampleCount() + sample];
        }

        output[sample] /= input.channelCount();
        output[sample] *= gain;
    }

    pcmOutput = PcmAudioFrame(output, pcmInput.format());
}

int main(int argc, char **argv)
{
    srand((unsigned) time(0));
    int randomNumber = rand();
    string node_name;
    stringstream ss;
    ss << "alsa_capture_node_";
    ss << randomNumber;
    ss >> node_name;
    ros::init(argc, argv, node_name);
    
    ros::NodeHandle nodeHandle;
    ros::NodeHandle privateNodeHandle("~");

    string device;
    string formatString;
    PcmAudioFrameFormat format;
    int channelCount;
    int samplingFrequency;
    int frameSampleCount;
    string member_id;


    bool merge;
    float merge_gain;

    privateNodeHandle.getParam("member_id", member_id);
    privateNodeHandle.getParam("device", device);
    privateNodeHandle.getParam("format", formatString);
    format = parseFormat(formatString);

    privateNodeHandle.getParam("channel_count", channelCount);
    privateNodeHandle.getParam("sampling_frequency", samplingFrequency);
    privateNodeHandle.getParam("frame_sample_count", frameSampleCount);
    

    privateNodeHandle.getParam("merge", merge);
    privateNodeHandle.getParam("merge_gain", merge_gain);

    string topic_pub;
    stringstream ss_pub;
    ss_pub << member_id;
    ss_pub << "/audio_out";
    ss_pub >> topic_pub;

    ros::Publisher audioPub = nodeHandle.advertise<audio_utils::AudioFrame>(topic_pub, 10);

    audio_utils::AudioFrame audioFrameMsg;
    audioFrameMsg.format = formatString;
    audioFrameMsg.channel_count = merge ? 1 : channelCount;
    audioFrameMsg.sampling_frequency = samplingFrequency;
    audioFrameMsg.frame_sample_count = frameSampleCount;

    try
    {
        AlsaPcmDevice captureDevice(device, AlsaPcmDevice::Stream::Capture, format, channelCount, frameSampleCount, samplingFrequency);
        PcmAudioFrame manyChannelFrame(format, channelCount, frameSampleCount);
        PcmAudioFrame oneChannelFrame(format, 1, frameSampleCount);
        while(ros::ok())
        {
            captureDevice.read(manyChannelFrame);

            if (merge)
            {                
                mergeChannels(manyChannelFrame, oneChannelFrame, merge_gain);
                audioFrameMsg.data = vector<uint8_t>(oneChannelFrame.data(), oneChannelFrame.data() + oneChannelFrame.size());
            }
            else
            {
                audioFrameMsg.data = vector<uint8_t>(manyChannelFrame.data(), manyChannelFrame.data() + manyChannelFrame.size());
            }

            audioPub.publish(audioFrameMsg);
        }
    }
    catch (const std::exception& e)
    {
        ROS_ERROR(e.what());
    }

    return 0;
}
