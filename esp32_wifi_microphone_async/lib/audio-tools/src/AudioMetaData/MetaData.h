#pragma once

#include <stdlib.h>
#include "AudioTools/AudioTypes.h"
#include "AudioTools/AudioStreams.h"
#include "AudioMetaData/MetaDataICY.h"
#include "AudioMetaData/MetaDataID3.h"
#include "AudioHttp/HttpRequest.h"

namespace audio_tools {

/**
 * @brief ID3 and Icecast/Shoutcast metadata output support
 * @author Phil Schatzmann
 * @copyright GPLv3
 * 
 */
class MetaDataPrint : public AudioPrint {
  public:

    MetaDataPrint() = default;

    ~MetaDataPrint(){
        end();
        if (meta!=nullptr) delete meta;
    }

    /// Defines the callback
    virtual void setCallback(void (*fn)(MetaDataType info, const char* str, int len)) {
        LOGD(LOG_METHOD);
        callback = fn; 
    }

    /// Starts the processing - iceMetaint is determined from the HttpRequest
    virtual void begin(HttpRequest &http) {
        LOGD(LOG_METHOD);
        ICYUrlSetup icySetup;
        int metaInt = icySetup.setup(http);
        icySetup.executeCallback(callback);
        begin(metaInt);
    }

    /// Starts the processing - if iceMetaint is defined we use icecast
    virtual void begin(int iceMetaint=0) {
        LOGD("%s: %d", LOG_METHOD, iceMetaint);
        if (callback!=nullptr){
            if (meta == nullptr) {
                meta = (iceMetaint > 0) ? new MetaDataICY() : (AbstractMetaData *)  new MetaDataID3();
            }
            meta->setCallback(callback);    
            meta->setIcyMetaInt(iceMetaint);
            meta->begin();
        } else {
            LOGI("callback not defined -> not Metadata processing")
        }
    }

    virtual void end() {
        if (callback!=nullptr && meta != nullptr) {
            LOGD(LOG_METHOD);
            meta->end();
        }
    }

    /// Provide tha audio data to the API to parse for Meta Data
    virtual size_t write(const uint8_t *data, size_t length){
        LOGD("%s: %d", LOG_METHOD, length);

        if (callback!=nullptr){
            if (meta!=nullptr){
                CHECK_MEMORY();
                if (meta->write(data, length)!=length){
                    LOGE("Did not write all data");
                }
                CHECK_MEMORY();
            } else {
                LOGW("meta is null");
            }
        }
        return length;
    }

     virtual size_t write(uint8_t c) {
         LOGE("Not Supported");
         return 0;
     }

  protected:
    AbstractMetaData *meta=nullptr;
    void (*callback)(MetaDataType info, const char* str, int len)=nullptr;

};


}