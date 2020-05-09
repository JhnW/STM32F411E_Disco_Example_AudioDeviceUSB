#include "audio_device.h"

static volatile AudioSpeakerData_t spekerData;
static volatile AudioMicrophoneData_t microphoneData;
static uint8_t emptyData[EMPTY_DATA_SIZE];
static volatile AudioMode_t audioMode;

void InitAudioDevice()
{
	spekerData.pbuff = NULL;
	spekerData.size = 0;
	spekerData.isFirstFrame = true;
	spekerData.isFrameConsumed = true;
	microphoneData.frameID = 1;
	microphoneData.isFrameToConsume = false;
	microphoneData.isFrameConsumed = false;
	microphoneData.frameToRecord = 1;
	audioMode = AUDIO_MODE_SPEAKER;
	//audioMode = AUDIO_MODE_MICROPHONE;
	int i = 0;
	while(i++ < EMPTY_DATA_SIZE) emptyData[i] = 0x0;
}

inline volatile AudioSpeakerData_t* GetAudioSpeakerData(void)
{
	return &spekerData;
}

inline volatile AudioMicrophoneData_t* GetAudioMicrophoneData(void)
{
	return &microphoneData;
}

volatile AudioMode_t GetAudioMode(void)
{
	return audioMode;
}

void SetAudioMode(AudioMode_t mode)
{
	audioMode = mode;
}

uint8_t* GetZeroData(void)
{
	return emptyData;
}
