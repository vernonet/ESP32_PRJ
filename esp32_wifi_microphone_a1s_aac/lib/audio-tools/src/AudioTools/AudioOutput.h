#pragma once
#include "Arduino.h"
#include "AudioConfig.h"
#include "AudioTools/AudioTypes.h"
#include "AudioTools/Buffers.h"
#include "AudioTools/int24.h"
#include "AudioTools/VolumeControl.h"

#define MAX_SINGLE_CHARS 8

extern bool client_con;

namespace audio_tools {

/**
 * @brief Abstract Audio Ouptut class
 * @author Phil Schatzmann
 * @copyright GPLv3
 */
class AudioPrint : public Print {
    public:
        virtual size_t write(const uint8_t *buffer, size_t size) = 0;

        virtual size_t write(uint8_t ch) {
            tmp[tmpPos++] = ch;
            if (tmpPos>MAX_SINGLE_CHARS){
                flush();
            } 
            return 1;
        }

        void flush() {
            write((const uint8_t*)tmp, tmpPos-1);
            tmpPos=0;
        }

        // overwrite to do something useful
        virtual void setAudioInfo(AudioBaseInfo info) {
	 		LOGD(LOG_METHOD);
            info.logInfo();
        }

    protected:
        uint8_t tmp[MAX_SINGLE_CHARS];
        int tmpPos=0;

};



/**
 * @brief Stream Wrapper which can be used to print the values as readable ASCII to the screen to be analyzed in the Serial Plotter
 * The frames are separated by a new line. The channels in one frame are separated by a ,
 * @tparam T 
 * @author Phil Schatzmann
 * @copyright GPLv3
*/
template<typename T>
class CsvStream : public AudioPrint {

    public:
        CsvStream(int buffer_size=DEFAULT_BUFFER_SIZE, bool active=true) {
            this->active = active;
        }

        /// Constructor
        CsvStream(Print &out, int channels, int buffer_size=DEFAULT_BUFFER_SIZE, bool active=true) {
            this->channels = channels;
            this->out_ptr = &out;
            this->active = active;
        }

        void begin(){
	 		LOGD(LOG_METHOD);
            this->active = true;
        }

        void begin(AudioBaseInfo info){
	 		LOGD(LOG_METHOD);
            this->active = true;
            this->channels = info.channels;
        }

        void begin(int channels, Print &out=Serial){
	 		LOGD(LOG_METHOD);
            this->channels = channels;
            this->out_ptr = &out;
            this->active = true;
        }

        /// Sets the CsvStream as inactive 
        void end() {
	 		LOGD(LOG_METHOD);
            active = false;
        }

        /// defines the number of channels
        virtual void setAudioInfo(AudioBaseInfo info) {
	 		LOGD(LOG_METHOD);
            this->channels = info.channels;
        };

        virtual size_t write(const uint8_t* data, size_t len) {   
            if (!active) return 0;
	 		LOGD(LOG_METHOD);
            size_t lenChannels = len / (sizeof(T)*channels); 
            data_ptr = (T*)data;
            for (int j=0;j<lenChannels;j++){
                for (int ch=0;ch<channels;ch++){
                    out_ptr->print(*data_ptr);
                    data_ptr++;
                    if (ch<channels-1) Serial.print(", ");
                }
                Serial.println();
            }
            return len;
        }

    protected:
        T *data_ptr;
        Print *out_ptr = &Serial;
        int channels = 1;
        bool active = false;

};

/**
 * @brief Creates a Hex Dump
 * @author Phil Schatzmann
 * @copyright GPLv3
 */
class HexDumpStream : public AudioPrint {

    public:
        HexDumpStream(int buffer_size=DEFAULT_BUFFER_SIZE, bool active=true) {
            this->active = active;
        }

        /// Constructor
        HexDumpStream(Print &out, int buffer_size=DEFAULT_BUFFER_SIZE, bool active=true) {
            this->out_ptr = &out;
            this->active = active;
        }

        void begin(){
	 		LOGD(LOG_METHOD);
            this->active = true;
            pos = 0;
        }

        /// Sets the CsvStream as inactive 
        void end() {
	 		LOGD(LOG_METHOD);
            active = false;
        }

        void flush(){
            Serial.println();
            pos = 0;
        }

        virtual size_t write(const uint8_t* data, size_t len) {   
            if (!active) return 0;
	 		LOGD(LOG_METHOD);
            for (size_t j=0;j<len;j++){
                out_ptr->print(data[j], HEX);
                out_ptr->print(" ");
                pos++;
                if (pos == 8){
                    Serial.print(" - ");
                }
                if (pos == 16){
                    Serial.println();
                    pos = 0;
                }
            }
            return len;
        }

    protected:
        Print *out_ptr = &Serial;
        int pos = 0;
        bool active = false;
};


/**
 * @brief A more natural Stream class to process encoded data (aac, wav, mp3...).
 * @author Phil Schatzmann
 * @copyright GPLv3
 */
class EncodedAudioStream : public AudioPrint, public AudioBaseInfoSource { 
    public: 
        /**
         * @brief Construct a new Encoded Stream object - used for decoding
         * 
         * @param outputStream 
         * @param decoder 
         */
        EncodedAudioStream(Print &outputStream, AudioDecoder &decoder) {
	 		LOGD(LOG_METHOD);
            decoder_ptr = &decoder;
            decoder_ptr->setOutputStream(outputStream);
            writer_ptr = decoder_ptr;
            active = false;
        }


        /**
         * @brief Construct a new Encoded Audio Stream object - used for decoding
         * 
         * @param outputStream 
         * @param decoder 
         */
        EncodedAudioStream(Print *outputStream, AudioDecoder *decoder) {
	 		LOGD(LOG_METHOD);
            decoder_ptr = decoder;
            decoder_ptr->setOutputStream(*outputStream);
            writer_ptr = decoder_ptr;
            active = false;
        }

        /**
         * @brief Construct a new Encoded Audio Stream object - used for encoding
         * 
         * @param outputStream 
         * @param encoder 
         */
        EncodedAudioStream(Print &outputStream, AudioEncoder &encoder) {
	 		LOGD(LOG_METHOD);
            encoder_ptr = &encoder;
            encoder_ptr->setOutputStream(outputStream);
            writer_ptr = encoder_ptr;
            active = false;
        }

        /**
         * @brief Construct a new Encoded Audio Stream object - used for encoding
         * 
         * @param outputStream 
         * @param encoder 
         */
        EncodedAudioStream(Print *outputStream, AudioEncoder *encoder) {
	 		LOGD(LOG_METHOD);
            encoder_ptr = encoder;
            encoder_ptr->setOutputStream(*outputStream);
            writer_ptr = encoder_ptr;
            active = false;
        }

        /**
         * @brief Construct a new Encoded Audio Stream object - the Output and Encoder/Decoder needs to be defined with the begin method
         * 
         */
        EncodedAudioStream(){
	 		LOGD(LOG_METHOD);
            active = false;
        }

        /**
         * @brief Destroy the Encoded Audio Stream object
         * 
         */
        ~EncodedAudioStream(){
            if (write_buffer!=nullptr){
                delete [] write_buffer;
            }
        }

        /// Define object which need to be notified if the basinfo is changing
        void setNotifyAudioChange(AudioBaseInfoDependent &bi) {
	 		LOGD(LOG_METHOD);
            decoder_ptr->setNotifyAudioChange(bi);
        }


        /// Starts the processing - sets the status to active
        void begin(Print *outputStream, AudioEncoder *encoder) {
	 		LOGD(LOG_METHOD);
            encoder_ptr = encoder;
            encoder_ptr->setOutputStream(*outputStream);
            writer_ptr = encoder_ptr;
            if (!client_con) begin();
        }

        /// Starts the processing - sets the status to active
        void begin(Print *outputStream, AudioDecoder *decoder) {
	 		LOGD(LOG_METHOD);
            decoder_ptr = decoder;
            decoder_ptr->setOutputStream(*outputStream);
            writer_ptr = decoder_ptr;
            begin();
        }

        /// Starts the processing - sets the status to active
        void begin() {
	 		LOGD(LOG_METHOD);
            const CodecNOP *nop =  CodecNOP::instance();
            if (decoder_ptr != nop || encoder_ptr != nop){
                decoder_ptr->begin();
                encoder_ptr->begin();
                active = true;
            } else {
                LOGW("no decoder or encoder defined");
            }
        }

        /// Ends the processing
        void end() {
	 		LOGD(LOG_METHOD);
            decoder_ptr->end();
            encoder_ptr->end();
            active = false;
        }
        
        /// encode the data
        virtual size_t write(const uint8_t *data, size_t len){
	 		LOGD(LOG_METHOD);
            if(writer_ptr==nullptr || data==nullptr){
                LOGE("NPE");
                return 0;
            }
            return writer_ptr->write(data, len);
        }
        

        /// Returns true if status is active and we still have data to be processed
        operator bool() {
            return active;
        }

        /// Provides the initialized decoder
        AudioDecoder &decoder() {
            return *decoder_ptr;
        }

        /// Provides the initialized encoder
        AudioEncoder &encoder() {
            return *encoder_ptr;
        }

    protected:
        //ExternalBufferStream ext_buffer; 
        AudioDecoder *decoder_ptr = CodecNOP::instance();  // decoder
        AudioEncoder *encoder_ptr = CodecNOP::instance();  // decoder
        AudioWriter *writer_ptr = nullptr ;

        Stream *input_ptr; // data source for encoded data
        uint8_t *write_buffer = nullptr;
        int write_buffer_pos = 0;
        const int write_buffer_size = 256;
        bool active;        
};


/**
 * @brief Output PWM object on which we can apply some volume settings. To work properly the class needs to know the 
 * bits per sample. If nothing is defined we assume 16 bits!
 * @author Phil Schatzmann
 * @copyright GPLv3
 */
class VolumeOutput : public AudioPrint {
    public:

        /// Default Constructor
        VolumeOutput() = default;

        /// Constructor which automatically calls begin(Print out)!
        VolumeOutput(Print &out) {
            begin(out);
        }

        /// Assigns the final output 
        void begin(Print &out){
	 		LOGD(LOG_METHOD);
            p_out = &out;
        }

        /// Defines the volume control logic
        void setVolumeControl(VolumeControl &vc){
            cached_volume.setVolumeControl(vc);
        }

        /// Resets the volume control to use the standard logic
        void resetVolumeControl(){
            cached_volume.setVolumeControl(default_volume);
        }

        /// Writes raw PCM audio data, which will be the input for the volume control 
        virtual size_t write(const uint8_t *buffer, size_t size){
	 		LOGD(LOG_METHOD);
            if (buffer==nullptr || p_out==nullptr){
                LOGE("NPE");
                return 0;
            }
            if (volume_value < 0.99) applyVolume(buffer,size);
            return p_out->write(buffer, size);
        }

        /// Provides the nubmer of bytes we can write
        virtual int availableForWrite() { 
            return p_out==nullptr? 0 : p_out->availableForWrite();
        }

        /// Detines the Audio info - The bits_per_sample are critical to work properly!
        void setAudioInfo(AudioBaseInfo info){
	 		LOGD(LOG_METHOD);
            this->info = info;
        }

        /// Shortcut method to define the sample size (alternative to setAudioInfo())
        void setBitsPerSample(int bits_per_sample){
            info.bits_per_sample = bits_per_sample;
        }

        /// Shortcut method to define the sample size (alternative to setAudioInfo())
        void setBytesPerSample(int bytes_per_sample){
            info.bits_per_sample = bytes_per_sample*8;
        }

        /// Decreases the volume:  needs to be in the range of 0 to 1.0
        void setVolume(float vol){
            if (vol>1.0) vol = 1.0;
            if (vol<0.0) vol = 0.0;

            // round to 2 digits
            float value = (int)(vol * 100 + .5);
            volume_value = (float)value / 100;;
	 		LOGI("setVolume: %f", volume_value);
        }

        /// Provides the current volume setting
        float volume() {
            return volume_value;
        }

    protected:
        Print *p_out=nullptr;
        AudioBaseInfo info;
        float volume_value=1.0;
        SimulatedAudioPot default_volume;
        CachedVolumeControl cached_volume = CachedVolumeControl(default_volume);

        VolumeControl &volumeControl(){
            return cached_volume;
        }

        void applyVolume(const uint8_t *buffer, size_t size){
            switch(info.bits_per_sample){
                case 16:
                    applyVolume16((int16_t*)buffer, size/2);
                    break;
                case 24:
                    applyVolume24((int24_t*)buffer, size/3);
                    break;
                case 32:
                    applyVolume32((int32_t*)buffer, size/4);
                    break;
                default:
                    LOGE("Unsupported bits_per_sample: %d", info.bits_per_sample);
            }
        }

        void applyVolume16(int16_t* data, size_t size){
            float factor = volumeControl().getVolumeFactor(volume_value);
            for (size_t j=0;j<size;j++){
                data[j]= static_cast<int16_t>(factor * data[j]);
            }
        }

        void applyVolume24(int24_t* data, size_t size) {
            float factor = volumeControl().getVolumeFactor(volume_value);
            for (size_t j=0;j<size;j++){
                int32_t v = static_cast<int32_t>(data[j]);
                int32_t v1 = factor * v;
                data[j] = v1;
            }
        }

        void applyVolume32(int32_t* data, size_t size) {
            float factor = volumeControl().getVolumeFactor(volume_value);
            for (size_t j=0;j<size;j++){
                data[j]= static_cast<int32_t>(factor * data[j]);
            }
        }

};

} //n namespace