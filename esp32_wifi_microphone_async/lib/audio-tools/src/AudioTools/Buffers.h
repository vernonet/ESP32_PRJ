
#pragma once

#include "AudioTools/AudioLogger.h"

#undef MIN
#define MIN(A,B) ((A) < (B) ? (A) : (B))

namespace audio_tools {

// forward declaration
template<typename T> class NBuffer;


/**
 * @brief Shared functionality of all buffers
 * @author Phil Schatzmann
 * @copyright GPLv3
 */
template<typename T>
class BaseBuffer {
public:

    virtual ~BaseBuffer(){
    }

    // reads a single value
    virtual T read() = 0;
    
    // reads multiple values
    int readArray(T data[], int len){
        int lenResult = MIN(len, available());
        for (int j=0;j<lenResult;j++){
            data[j] = read();
        }
        return lenResult;
    }

    int writeArray(const T data[], int len){
        LOGD("%s: %d", LOG_METHOD, len);
        CHECK_MEMORY();

        int result = 0;
        for (int j=0;j<len;j++){
            if (write(data[j])==0){
                break;
            }
            result = j+1;
        }
        CHECK_MEMORY();
        return result;
    }

    // reads multiple values for array of 2 dimensional frames
    int readFrames(T data[][2], int len) {
        LOGD("%s: %d", LOG_METHOD, len);
        CHECK_MEMORY();
        int result = MIN(len, available());
        for (int j=0;j<result;j++){
            T sample = read();
            data[j][0] = sample;
            data[j][1] = sample;
        }
        CHECK_MEMORY();
        return result;
    }

    template <int rows, int channels>
    int readFrames(T (&data)[rows][channels]){
        int lenResult = MIN(rows, available());
        for (int j=0;j<lenResult;j++){
            T sample = read();
            for (int i=0;i<channels;i++){
                //data[j][i] = htons(sample);
                data[j][i] = sample;
            }
        }
        return lenResult;
    }

    // peeks the actual entry from the buffer
    virtual T peek() = 0;

    // checks if the buffer is full
    virtual bool isFull() = 0;

    bool isEmpty() {
        return available() == 0;
    }
    
    // write add an entry to the buffer
    virtual bool write(T data) = 0;
    
    // clears the buffer
    virtual void reset() = 0;
    
    // provides the number of entries that are available to read
    virtual int available() = 0;
    
    // provides the number of entries that are available to write
    virtual int availableForWrite() = 0;
    
    // returns the address of the start of the physical read buffer
    virtual T* address() = 0;

  protected:
    void setWritePos(int pos) {};

    friend NBuffer<T>;

};


/**
 * @brief A simple Buffer implementation which just uses a (dynamically sized) array
 * @author Phil Schatzmann
 * @copyright GPLv3
 */

template<typename T>
class SingleBuffer : public BaseBuffer<T> {
public:
    /**
     * @brief Construct a new Single Buffer object
     * 
     * @param size 
     */
    SingleBuffer(int size){
        this->max_size = size;
        buffer = new T[max_size];
        reset();
    }

    /**
     * @brief Construct a new Single Buffer w/o allocating any memory
     * 
     * @param data 
     * @param size 
     * @param len 
     */
    SingleBuffer(){
        reset();
    }

    /// notifies that the external buffer has been refilled
    void onExternalBufferRefilled(void *data, int len){
        this->owns_buffer = false;
        this->buffer = (uint8_t*)data;
        this->current_read_pos = 0;
        this->current_write_pos = len;
    }

    virtual ~SingleBuffer() {
        if (owns_buffer && buffer!=nullptr){
            delete[] buffer;
        }
    }

    bool write(T sample){
        bool result = false;
        if (buffer != nullptr &&  current_write_pos<max_size){
            buffer[current_write_pos++] = sample;
            result = true;
        }
        return result;
    }
    
    T read() {
        T result = 0;
        if (buffer != nullptr && current_read_pos < current_write_pos){
            result = buffer[current_read_pos++];
        }
        return result;
    }

    T peek() {
        T result = 0;
        if (buffer != nullptr && current_read_pos < current_write_pos){
            result = buffer[current_read_pos];
        }
        return result;
    }
    
    int available() {
        int result = current_write_pos - current_read_pos;
        return max(result, 0);
    }
    
    int availableForWrite() {
        return max_size - current_write_pos;
    }
    
    bool isFull(){
        return availableForWrite()<=0;
    }
    
    T* address() {
        return buffer;
    }
    
    void reset(){
        current_read_pos = 0;
        current_write_pos = 0;
    }

    /// If we load values directly into the address we need to set the avialeble size
    void setAvailable(size_t available_size){
        current_read_pos = 0;
        current_write_pos = available_size;
    }

    size_t size() {
        return max_size;
    }
    
protected:
    int max_size=0;
    int current_read_pos=0;
    int current_write_pos=0;
    bool owns_buffer = true;
    T *buffer = nullptr;

    void setWritePos(int pos) {
        current_write_pos = pos;
    }
};

/**
 * @brief Implements a typed Ringbuffer
 * 
 * @tparam T 
 */
template<typename T>
class RingBuffer : public BaseBuffer<T> {
    public:
        RingBuffer(int size){
            this->max_size = size;
            _aucBuffer = new T[max_size];
            reset();
        }
        
        ~RingBuffer() {
            delete[] _aucBuffer;
        }

        virtual T read(){
            if (isEmpty())
                return -1;

            uint8_t value = _aucBuffer[_iTail];
            _iTail = nextIndex(_iTail);
            _numElems--;

            return value;
        }

        // peeks the actual entry from the buffer
        virtual T peek() {
            if (isEmpty())
                return -1;

            return _aucBuffer[_iTail];
        }

        // checks if the buffer is full
        virtual bool isFull() {
            return available()==max_size;
        }

        bool isEmpty() {
            return available() == 0;
        }
        
        // write add an entry to the buffer
        virtual bool write(T data) {
            bool result = false;
            if (!isFull()){
                _aucBuffer[_iHead] = data ;
                _iHead = nextIndex(_iHead);
                _numElems++;
                result = true;
            }
            return result;
        }
        
        // clears the buffer
        virtual void reset() {
            _iHead = 0;
            _iTail = 0;
            _numElems = 0;
        }
        
        // provides the number of entries that are available to read
        virtual int available() {
            return _numElems;
        }
        
        // provides the number of entries that are available to write
        virtual int availableForWrite() {
            return (max_size - _numElems);
        }
        
        // returns the address of the start of the physical read buffer
        virtual T* address() {
            return _aucBuffer;
        }
        

    protected:
        T *_aucBuffer ;
        int _iHead ;
        int _iTail ;
        int _numElems;
        int max_size;

        int nextIndex(int index){
            return (uint32_t)(index + 1) % max_size;
        }

};

/**
 * @brief A lock free N buffer. If count=2 we create a DoubleBuffer, if count=3 a TripleBuffer etc.
 * @author Phil Schatzmann
 * @copyright GPLv3
 */
template<typename T>
class NBuffer : public BaseBuffer<T> {
public:

  NBuffer(int size=512, int count=2){
      filled_buffers = new BaseBuffer<T>*[count];
      avaliable_buffers = new BaseBuffer<T>*[count];

      write_buffer_count = 0;
      buffer_count = count;
      buffer_size = size;
      for (int j=0;j<count;j++){
        avaliable_buffers[j] = new SingleBuffer<T>(size);
        if (avaliable_buffers[j]==nullptr){
            LOGE("Not Enough Memory for buffer %d",j);
        }
      }
  }

  virtual ~NBuffer() {
      delete actual_write_buffer;
      delete actual_read_buffer;

      BaseBuffer<T> *ptr = getNextAvailableBuffer();
      while(ptr!=nullptr){
          delete ptr;
          ptr = getNextAvailableBuffer();
      }
      
      ptr = getNextFilledBuffer();
      while(ptr!=nullptr){
          delete ptr;
          ptr = getNextFilledBuffer();
      }
  }


  // reads an entry from the buffer 
  T read() {
      T result = 0;
      if (available()>0){
          result = actual_read_buffer->read();
      }
      return result;
  }

  // peeks the actual entry from the buffer 
  T peek() {
      T result = 0;
      if (available()>0){
          result = actual_read_buffer->peek();
      }
      return result;
  }

  // checks if the buffer is full
  bool isFull() {
    return availableForWrite()==0;
  }

  // write add an entry to the buffer
  bool write(T data){
      bool result = false;
      if (actual_write_buffer==nullptr){
          actual_write_buffer = getNextAvailableBuffer();
      }
      if (actual_write_buffer!=nullptr){
          result = actual_write_buffer->write(data);
          // if buffer is full move to next available
          if (actual_write_buffer->isFull()){
              addFilledBuffer(actual_write_buffer);
              actual_write_buffer = getNextAvailableBuffer();
          }
      } else {
          //Logger.debug("actual_write_buffer is full");
      }
      
      if (start_time==0l){
          start_time = millis();
      }
      sample_count++;
      
      return result;
  }

  // determines the available entries for the current read buffer
  int available() {
      if (actual_read_buffer==nullptr){
          actual_read_buffer = getNextFilledBuffer();
      }
      if (actual_read_buffer==nullptr){
          return 0;
      }
      int result = actual_read_buffer->available();
      if (result == 0){
          // make current read buffer available again
          resetCurrent();
          result = actual_read_buffer==nullptr? 0 : actual_read_buffer->available();
      }
      return result;
  }

  // deterMINes the available entries for the write buffer
  int availableForWrite() {
      if (actual_write_buffer==nullptr){
          actual_write_buffer = getNextAvailableBuffer();
      }
      // if we used up all buffers - there is nothing available any more
      if (actual_write_buffer==nullptr){
          return 0;
      }
      // check on actual buffer
      if (actual_write_buffer->isFull()){
          // if buffer is full we move it to filled buffers ang get the next available
          addFilledBuffer(actual_write_buffer);
          actual_write_buffer = getNextAvailableBuffer();
      }
      return actual_write_buffer->availableForWrite();
  }
  
  // resets all buffers
  void reset(){
      LOGD(LOG_METHOD);
      while (actual_read_buffer!=nullptr){
          actual_read_buffer->reset();
          addAvailableBuffer(actual_read_buffer);
          // get next read buffer
          actual_read_buffer = getNextFilledBuffer();
      }
  }
  
  // provides the actual sample rate
  unsigned long sampleRate() {
      unsigned long run_time = (millis() - start_time);
      return run_time == 0 ? 0 : sample_count * 1000 / run_time;
  }

  // returns the address of the start of the phsical read buffer
  T* address() {
      return actual_read_buffer==nullptr ? nullptr : actual_read_buffer->address();
  }

  // Alternative interface using address: the current buffer has been filled
  BaseBuffer<T> &writeEnd(){
    if (actual_write_buffer!=nullptr){
        actual_write_buffer->setWritePos(buffer_size);
        addFilledBuffer(actual_write_buffer);
    }
    actual_write_buffer = getNextAvailableBuffer();
    return *actual_write_buffer;
  }

  // Alternative interface using address: marks actual buffer as processed and provides access to next read buffer
  BaseBuffer<T> &readEnd() {
    // make current read buffer available again
    resetCurrent();
    return *actual_read_buffer;
  }
    
    
protected:
    int buffer_size = 0;
    uint16_t buffer_count = 0;
    uint16_t write_buffer_count = 0;
    BaseBuffer<T> *actual_read_buffer = nullptr;
    BaseBuffer<T> *actual_write_buffer = nullptr;
    BaseBuffer<T> **avaliable_buffers = nullptr;
    BaseBuffer<T> **filled_buffers = nullptr;
    unsigned long start_time = 0;
    unsigned long sample_count = 0;


    void resetCurrent(){
      if (actual_read_buffer!=nullptr){
          actual_read_buffer->reset();
          addAvailableBuffer(actual_read_buffer);
      }
      // get next read buffer
      actual_read_buffer = getNextFilledBuffer();
    }

    BaseBuffer<T> *getNextAvailableBuffer() {
        BaseBuffer<T> *result = nullptr;
        for (int j=0;j<buffer_count;j++){
            result = avaliable_buffers[j];
            if (result!=nullptr){
                avaliable_buffers[j] = nullptr;
                break;
            }
        }
        return result;
    }


    bool addAvailableBuffer(BaseBuffer<T> *buffer){
        bool result = false;
        for (int j=0;j<buffer_count;j++){
            if (avaliable_buffers[j]==nullptr){
                avaliable_buffers[j] = buffer;
                result = true;
                break;
            }
        }
        return result;
    }
    
    BaseBuffer<T> *getNextFilledBuffer() {
        BaseBuffer<T> *result = nullptr;
        if (write_buffer_count > 0){
            // get oldest entry
            result = filled_buffers[0];
            // move data by 1 entry to the left
            for (int j=0;j<write_buffer_count-1;j++)
                filled_buffers[j]=filled_buffers[j+1];
            // clear last enty
            filled_buffers[write_buffer_count-1]=nullptr;
            write_buffer_count--;
        }
        return result;
    }
    
    bool addFilledBuffer(BaseBuffer<T> *buffer){
        bool result = false;
        if (write_buffer_count<buffer_count){
            filled_buffers[write_buffer_count++] = buffer;
            result = true;
        }
        return result;
    }

};

} // namespace
