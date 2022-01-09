#pragma once
#include "settings.h"
#include "FS.h"

#ifdef SD_MMC_1BIT_MODE
    #include "SD_MMC.h"
    extern fs::SDMMCFS gFSystem;
#else
    #include "SD.h"
    extern fs::SDFS gFSystem;
#endif

void SdCard_Init(void);
void SdCard_Exit(void);
sdcard_type_t SdCard_GetType(void);
char **SdCard_ReturnPlaylist(const char *fileName, const uint32_t _playMode);
void TakeSDSemaphore(void);
void GiveSDSemaphore(void);
