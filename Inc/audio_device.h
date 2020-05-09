#ifndef AUDIO_DEVICE_H
#define AUDIO_DEVICE_H
#include "stdbool.h"
#include "stdint.h"
#include "stddef.h"

#define AUDIO_BYTES 16
#define EMPTY_DATA_SIZE (48*4)
#define AUDIO_IN_SIZE (2*2*16)

typedef enum AudioMode_t
{
	AUDIO_MODE_SPEAKER,
	AUDIO_MODE_MICROPHONE
}AudioMode_t;

typedef struct AudioSpeakerData_t
{
	uint8_t *pbuff;
	size_t size;
	bool isFirstFrame;
	bool isFrameConsumed;
}AudioSpeakerData_t;

typedef struct AudioMicrophoneData_t
{
	uint16_t pdm1[4 * AUDIO_BYTES];
	uint16_t pdm2[4 * AUDIO_BYTES];
	uint16_t pcm1[2 * AUDIO_BYTES * 4];
	uint16_t pcm2[2 * AUDIO_BYTES * 4];
	bool isFrameToConsume;
	bool isFrameConsumed;
	int frameID;
	int frameToRecord;
	uint16_t buffer[128];
}AudioMicrophoneData_t;

void InitAudioDevice();
volatile AudioSpeakerData_t* GetAudioSpeakerData(void);
volatile AudioMicrophoneData_t* GetAudioMicrophoneData(void);
AudioMode_t GetAudioMode(void);
void SetAudioMode(AudioMode_t mode);

uint8_t *GetZeroData(void);

#endif /* AUDIO_DEVICE_H*/
