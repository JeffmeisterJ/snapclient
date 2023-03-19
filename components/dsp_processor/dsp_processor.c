

#include <stdint.h>
#include <sys/time.h>

#include "driver/i2s.h"
#include "dsps_biquad.h"
#include "dsps_biquad_gen.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/queue.h"
#include "freertos/task.h"
//#include "websocket_if.h"
#include "driver/dac.h"
#include "driver/i2s.h"
#include "dsp_processor.h"
#include "hal/i2s_hal.h"
//#include "adc1_i2s_private.h"
#include "board_pins_config.h"
#include "soc/rtc.h"

#ifdef CONFIG_USE_BIQUAD_ASM
#define BIQUAD dsps_biquad_f32_ae32
#else
#define BIQUAD dsps_biquad_f32
#endif

// #define BUFFER_SIZE (2*((32*44100)/8) //2 channels *((32bits * 44100) samples divided by 8 bits)
// #define BUFFER_SIZE 2*(192 * (3528 + 12)) //44.1khz
// #define BUFFER_SIZE  192 * (3840 + 12)
#define BUFFER_SIZE  384 * (1920 + 12)

uint32_t bits_per_sample = CONFIG_BITS_PER_SAMPLE;

static xTaskHandle s_dsp_i2s_task_handle = NULL;
// static RingbufHandle_t audio_sample_queue = NULL;

extern xQueueHandle i2s_queue;
extern xQueueHandle flow_queue;
extern xQueueHandle snap_ctrl_queue;
xQueueHandle audio_sample_queue = NULL;

extern struct timeval tdif;

extern uint32_t buffer_ms;
extern uint8_t muteCH[4];
uint8_t volume;


uint8_t dspFlow =
    dspfStereo; // dspfStereo; //dspfBassBoost; //dspfStereo;

ptype_t bq[8];

void setup_dsp_i2s(uint32_t sample_rate, bool slave_i2s) {
  i2s_config_t i2s_config0 = {
      .mode = I2S_MODE_MASTER | I2S_MODE_TX,  // Only TX
      .sample_rate = sample_rate,
      .bits_per_sample = bits_per_sample,
      .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,  // 2-channels
      .communication_format = I2S_COMM_FORMAT_I2S,
      .dma_buf_count = 8,
      .dma_buf_len = 480,
      .intr_alloc_flags = 1,  // Default interrupt priority
      .use_apll = true,
      .fixed_mclk = 0,
      .tx_desc_auto_clear = true  // Auto clear tx descriptor on underflow
  };

  i2s_pin_config_t pin_config0;
  get_i2s_pins(I2S_NUM_0, &pin_config0);

  i2s_driver_install(0, &i2s_config0, 7, &i2s_queue);
  i2s_zero_dma_buffer(0);
  i2s_set_pin(0, &pin_config0);
  //gpio_set_drive_capability(CONFIG_MASTER_I2S_BCK_PIN, 0);
  //gpio_set_drive_capability(CONFIG_MASTER_I2S_LRCK_PIN, 0);
  //gpio_set_drive_capability(CONFIG_MASTER_I2S_DATAOUT_PIN, 0);
  ESP_LOGI("I2S", "I2S interface master setup");
  if (slave_i2s) {
    ESP_LOGI("I2S", "Config slave I2S channel");
  
    i2s_config_t i2s_config1 = {
        .mode = I2S_MODE_SLAVE | I2S_MODE_TX,  // Only TX - Slave channel
        .sample_rate = sample_rate,
        .bits_per_sample = bits_per_sample,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,  // 2-channels
        .communication_format = I2S_COMM_FORMAT_I2S,
        .dma_buf_count = 8,
        .dma_buf_len = 480,
        .use_apll = true,
        .fixed_mclk = 0,
        .tx_desc_auto_clear = true  // Auto clear tx descriptor on underflow
    };
    i2s_pin_config_t pin_config1;
    get_i2s_pins(I2S_NUM_1, &pin_config1);
    i2s_driver_install(I2S_NUM_1, &i2s_config1, 7, &i2s_queue);
    i2s_zero_dma_buffer(1);
    i2s_set_pin(1, &pin_config1);
  }
}

static void dsp_i2s_task_handler(void *arg) {
  struct timeval now, tv1;
  uint32_t cnt = 0;
  uint8_t *audio = NULL;
  uint8_t *ax = NULL;
  uint8_t *drainPtr = NULL;
  audio_pkt_element_t audio_pkt;
  float sbuffer0[1024];
  float sbuffer1[1024];
  float sbuffer2[1024];
  float sbufout0[1024];
  float sbufout1[1024];
  float sbufout2[1024];
  float sbuftmp0[1024];

  uint8_t dsp_audio[4 * 1024];
  uint8_t dsp_audio1[4 * 1024];
  size_t n_byte_read = 0;
  size_t chunk_size = 0;
  size_t bytes_written = 0;
  muteCH[0] = 0;
  muteCH[1] = 0;
  muteCH[2] = 0;
  muteCH[3] = 0;
  // uint32_t inBuffer,freeBuffer,wbuf,rbuf ;

  // static int32_t avgcnt = 0;
  // uint32_t avgcntlen = 64;  // x 960/4*1/fs = 320ms @48000 kHz
  // uint32_t avgarray[128] = {0};
  // uint32_t sum;
  // float avg = 0.0;
  uint32_t clk_adj = 210;
  int32_t age = 0;
  int32_t agesec;
  int32_t ageusec;
  int playback_synced = 0;
  uint32_t flow_que_msg = 0;
  int flow_state = 0;
  int flow_drain_counter = 0;
  double dynamic_vol = 1.0;
  snap_ctrl_element_t ctrl_element;

  rtc_clk_apll_enable(1, 0, clk_adj, 5, 6);

  for (;;) {
    // 23.12.2020 - JKJ : Buffer protocol major change
    //   - Audio data prefaced with timestamp and size tag server timesamp
    //   (sec/usec (uint32_t)) and number of bytes (uint32_t)
    //   - DSP processor is now last in line to quanity if data must be passed
    //   on to hw i2s interface buffer or delayed based on
    //     timestamp
    //   - Timestamp vs now must be use to schedul if pack must be delay,
    //   dropped or played

    // Old scheem
    // Condition                                     state   Action
    // Buffer is empty - because not being filled    Stopped  Wait
    // Buffer is increasing and below target         Filling  Wait
    // Buffer above target                           Playing  Consume from
    // buffer Buffer is below target                        Playing  Short delay

    // Snap client process has a realtime queue to signal change in sample flow.
    // If client is muted from server - DSP output render will be signaled and
    // allowed to take action.

    cnt++;

    if (xQueueReceive(flow_queue, &flow_que_msg, 0)) {
      ESP_LOGI("I2S", "FLOW Queue message: %d ", flow_que_msg);
      if (flow_state != flow_que_msg) {
        switch (flow_que_msg) {
          case 2:  // front end timed out -- network congestion more then 200
                   // msec or source off
            if (flow_state == 1) {
              flow_state = 1;
            } else {
              flow_state = 2;
              flow_drain_counter = 20;
              playback_synced = 0;
            }
            break;
          case 1:  // Server has muted this channel. Turn down the volume over
                   // next 10 packages
            flow_state = 1;
            flow_drain_counter = 20;
            playback_synced = 0;
            break;
          case 0:  // Reset sync counter and
            flow_state = 0;
            playback_synced = 0;
            dynamic_vol = 1.0;
            break;
        }
      }
    }

    if (xQueueReceive(snap_ctrl_queue, &ctrl_element, 0)) {
      ESP_LOGI("I2S", "Snap control queue message: type: %d, value: %d ", ctrl_element.type, ctrl_element.value);
      switch(ctrl_element.type) {
        case VOLUME:
          volume = (uint8_t) (ctrl_element.value&0xff);
          break;
        case LATENCY:
          break;
        case MUTE:
          break;
      }
    };

    BaseType_t resp = xQueueReceive(audio_sample_queue, &audio_pkt, 40);


    if (!resp) {
      ESP_LOGI("I2S", "Wait: no data in buffer %d %d", cnt, n_byte_read);
      continue;
    }

    // ESP_LOGI("RBUF", "Got item from queue! sample_buf size: %d", audio_pkt.samplebuf_sz);

    gettimeofday(&now, NULL);

    timersub(&now, &tdif, &tv1);
    // ESP_LOGI("log", "diff :% 11ld.%03ld ", tdif.tv_sec ,
    // tdif.tv_usec/1000); ESP_LOGI("log", "head :% 11ld.%03ld ", tv1.tv_sec ,
    // tv1.tv_usec/1000); ESP_LOGI("log", "tsamp :% 11d.%03d ", audio_pkt.timestamp_sec ,
    // audio_pkt.timestamp_usec/1000);

    ageusec = tv1.tv_usec - audio_pkt.timestamp_usec;
    agesec = tv1.tv_sec - audio_pkt.timestamp_sec;
    if (ageusec < 0) {
      ageusec += 1000000;
      agesec -= 1;
    }
    age = agesec * 1000 + ageusec / 1000;
    if ( age < 0 ) { age = 0;  }

    // rtc_clk_apll_enable(1, 0, 250, 5, 6);

    if (playback_synced == 1) {
      if (cnt % 100 == 2) {
        if (age > (buffer_ms-100)) {  // Too slow speedup playback
          // rtc_clk_apll_enable(1, sdm0, sdm1, sdm2, odir);
          if (clk_adj != 220) {
            clk_adj+=1;
            ESP_LOGI("APLL", "Adjusting clock faster: %d", clk_adj);
            rtc_clk_apll_enable(1, 0, clk_adj, 5, 6);
          }
        }  // ESP_LOGI("i2s", "%d %d", buffer_ms, age );
        else if (age < 100){
          if (clk_adj != 200) {
            clk_adj-=1;
            ESP_LOGI("APLL", "Adjusting clock slower: %d", clk_adj);
            rtc_clk_apll_enable(1, 0, clk_adj, 5, 6);
          }
        }
      }
      // ESP_LOGI("i2s", "%d %d %1.2f ", buffer_ms, age, dynamic_vol );
    } else {
      while (age < buffer_ms) {
        vTaskDelay(2);
        gettimeofday(&now, NULL);
        timersub(&now, &tdif, &tv1);
        ageusec = tv1.tv_usec - audio_pkt.timestamp_usec;
        agesec = tv1.tv_sec - audio_pkt.timestamp_sec;
        if (ageusec < 0) {
          ageusec += 1000000;
          agesec -= 1;
        }
        age = agesec * 1000 + ageusec / 1000;

        ESP_LOGI("I2S", "%d %d syncing ... ", buffer_ms, age);
      }
      ESP_LOGI("I2S", "%d %d SYNCED ", buffer_ms, age);

      playback_synced = 1;
    }
    chunk_size = audio_pkt.samplebuf_sz;
    audio = audio_pkt.samplebuf;

    // audio = xRingbufferReceive(s_ringbuf_i2s, &chunk_size, 20);
    // if (audio == NULL) { 
      // ESP_LOGI("DSP", "Failed to receive audio chunk!");
    // };
    /* audio = (uint8_t *)xRingbufferReceiveUpTo(s_ringbuf_i2s, &chunk_size,
    //                                         (portTickType)20, ts_size);
    if (chunk_size != ts_size) {
      uint32_t missing = ts_size - chunk_size; 
      ESP_LOGI("I2S", "Error readding audio from ring buf : read %d of %d , missing %d", chunk_size,ts_size, missing);
      vRingbufferReturnItem(s_ringbuf_i2s, (void *)audio);
      ax = (uint8_t *)(audio +chunk_size); 
      ax = (uint8_t *)xRingbufferReceiveUpTo(s_ringbuf_i2s, &chunk_size,
                                              (portTickType)20, missing);
      vRingbufferReturnItem(s_ringbuf_i2s, (void *)ax);
      ESP_LOGI("I2S", "Read the next %d ", chunk_size );
    }
    */
    // vRingbufferReturnItem(s_ringbuf_i2s, (void *)audio);
    // printf("Read data   : %d \n",chunk_size );

    // vRingbufferGetInfo(s_ringbuf_i2s, &freeBuffer, &rbuf, &wbuf, NULL,
    // &inBuffer );
    /*
    if (avgcnt >= avgcntlen) { avgcnt = 0; }
    avgarray[avgcnt++] = inBuffer;
    sum = 0;
    for (int n = 0; n < avgcntlen ; n++)
    { sum = sum + avgarray[n];
    }
    avg = sum / avgcntlen;

    #ifndef CONFIG_USE_PSRAM
      buffer_ms = 150;
    #endif

    if (inBuffer < (buffer_ms*48*4)) {vTaskDelay(1); }
    */
    // audio = (uint8_t *)xRingbufferReceiveUpTo(s_ringbuf_i2s,
    // &chunk_size,(portTickType) 20 ,960);  // 200 ms timeout

    // audio = (uint8_t *)xRingbufferReceive(s_ringbuf_i2s, &chunk_size,
    // (portTickType)portMAX_DELAY);
    /*if (chunk_size == 0)
    { printf("no data in buffer Error\n",inBuffer);

    }
    */
    // else if (inBuffer < (buffer_ms*48*4))
    //{ printf("Buffering ...  buffer : %d\n",inBuffer);
    //}
    // if ((flow_state >= 1) & (flow_drain_counter > 0)) {
    //   flow_drain_counter--;
    //   dynamic_vol = 1.0 / (20 - flow_drain_counter);
    //   if (flow_drain_counter == 0) {
    //     // Drain buffer
    //     vRingbufferReturnItem(audio_sample_queue, (void *)audio);
    //     xRingbufferPrintInfo(audio_sample_queue);

    //     uint32_t drainSize;
    //     drainPtr = (uint8_t *)xRingbufferReceive(audio_sample_queue, &drainSize,
    //                                              (portTickType)0);
    //     ESP_LOGI("I2S", "Drained Ringbuffer (bytes):%d ", drainSize);
    //     if (drainPtr != NULL) {
    //       vRingbufferReturnItem(audio_sample_queue, (void *)drainPtr);
    //     }
    //     xRingbufferPrintInfo(audio_sample_queue);
    //     playback_synced = 0;
    //     dynamic_vol = 1.0;

    //     continue;
    //   }
    // }

    {
      int16_t len = chunk_size / 2;
      if (cnt % 100 == 2) {
        ESP_LOGI("I2S", "Chunk: %d, Age: %d ms, Buffer: %d ms", chunk_size, age, ((uxQueueMessagesWaiting(audio_sample_queue) *(1920/2/2))*1000)/44100);
        // xRingbufferPrintInfo(s_ringbuf_i2s);
        uint32_t bytes_in_ringbuffer;
        // vRingbufferGetInfo(s_ringbuf_i2s, NULL, NULL, NULL, NULL, &bytes_in_ringbuffer);
        // int buffer_full_pct = (200*bytes_in_ringbuffer+1)/(BUFFER_SIZE*2); //*100;
        // ESP_LOGI("RINGBUFFER", "Percent full: %d (%d kb/%d kb)", buffer_full_pct, (bytes_in_ringbuffer/1024), (BUFFER_SIZE/1024));
      }

      // for (uint16_t i = 0; i < len; i++) {
      //   sbuffer0[i] =
      //       dynamic_vol * 0.5 *
      //       ((float)((int16_t)(audio[i * 4 + 1] << 8) + audio[i * 4 + 0])) /
      //       32768;
      //   sbuffer1[i] =
      //       dynamic_vol * 0.5 *
      //       ((float)((int16_t)(audio[i * 4 + 3] << 8) + audio[i * 4 + 2])) /
      //       32768;
      //   sbuffer2[i] = ((sbuffer0[i] / 2) + (sbuffer1[i] / 2));
      // }

      switch (dspFlow) {
        case dspfStereo: {   if (cnt%120==0)
          { 
            // ESP_LOGI("I2S", "In dspf Stero :%d",chunk_size);
            // ESP_LOGI("I2S", "First samplepoint: %d, volume: %d, after volume calc: %d", audio_pkt.samplebuf[0], volume, (audio_pkt.samplebuf[0]/100) * volume);
          } 
          // ws_server_send_bin_client(0,(char*)audio, 240);
          // printf("%d %d \n",byteWritten, i2s_evt.size );
          //}

          //TODO use volume to calculate sample amplitude



          for (uint16_t i = 0; i < audio_pkt.samplebuf_sz/2; i+=2) {
            audio_pkt.samplebuf[i + 0] = (muteCH[0] == 1) ? 0 : audio_pkt.samplebuf[i + 0]; //((audio_pkt.samplebuf[i * 4 + 0]/100)*volume);
            audio_pkt.samplebuf[i + 1] = (muteCH[1] == 1) ? 0 : audio_pkt.samplebuf[i + 1]; //((audio_pkt.samplebuf[i * 4 + 1]/100)*volume);
          }
          if (bits_per_sample == 16) {
            i2s_write(0, (char *)audio, chunk_size, &bytes_written,
                      portMAX_DELAY);
          } else {
            i2s_write_expand(0, (char *)audio, chunk_size, 16, 32,
                             &bytes_written, portMAX_DELAY);
          }
        } break;
        case dspfBassBoost: {  // CH0 low shelf 6dB @ 400Hz
          BIQUAD(sbuffer0, sbufout0, len, bq[6].coeffs, bq[6].w);
          BIQUAD(sbuffer1, sbufout1, len, bq[7].coeffs, bq[7].w);
          int16_t valint[2];
          for (uint16_t i = 0; i < len; i++) {
            valint[0] =
                (muteCH[0] == 1) ? (int16_t)0 : (int16_t)(sbufout0[i] * 32768);
            valint[1] =
                (muteCH[1] == 1) ? (int16_t)0 : (int16_t)(sbufout1[i] * 32768);
            dsp_audio[i * 4 + 0] = (valint[0] & 0xff);
            dsp_audio[i * 4 + 1] = ((valint[0] & 0xff00) >> 8);
            dsp_audio[i * 4 + 2] = (valint[1] & 0xff);
            dsp_audio[i * 4 + 3] = ((valint[1] & 0xff00) >> 8);
          }
          if (bits_per_sample == 16) {
            i2s_write(0, (char *)dsp_audio, chunk_size, &bytes_written,
                      portMAX_DELAY);
          } else {
            i2s_write_expand(0, (char *)dsp_audio, chunk_size, 16, 32,
                             &bytes_written, portMAX_DELAY);
          }

        } break;
        case dspfBiamp: {
          if (cnt % 120 == 0) {
            ESP_LOGI("I2S", "In dspf biamp :%d", chunk_size);
            // ws_server_send_bin_client(0,(char*)audio, 240);
            // printf("%d %d \n",byteWritten, i2s_evt.size );
          }
          // Process audio ch0 LOW PASS FILTER
          BIQUAD(sbuffer0, sbuftmp0, len, bq[0].coeffs, bq[0].w);
          BIQUAD(sbuftmp0, sbufout0, len, bq[1].coeffs, bq[1].w);

          // Process audio ch1 HIGH PASS FILTER
          BIQUAD(sbuffer0, sbuftmp0, len, bq[2].coeffs, bq[2].w);
          BIQUAD(sbuftmp0, sbufout1, len, bq[3].coeffs, bq[3].w);

          int16_t valint[2];
          for (uint16_t i = 0; i < len; i++) {
            valint[0] =
                (muteCH[0] == 1) ? (int16_t)0 : (int16_t)(sbufout0[i] * 32768);
            valint[1] =
                (muteCH[1] == 1) ? (int16_t)0 : (int16_t)(sbufout1[i] * 32768);
            dsp_audio[i * 4 + 0] = (valint[0] & 0xff);
            dsp_audio[i * 4 + 1] = ((valint[0] & 0xff00) >> 8);
            dsp_audio[i * 4 + 2] = (valint[1] & 0xff);
            dsp_audio[i * 4 + 3] = ((valint[1] & 0xff00) >> 8);
          }
          if (bits_per_sample == 16) {
            i2s_write(0, (char *)dsp_audio, chunk_size, &bytes_written,
                      portMAX_DELAY);
          } else {
            i2s_write_expand(0, (char *)dsp_audio, chunk_size, 16, 32,
                             &bytes_written, portMAX_DELAY);
          }
        } break;

        case dspf2DOT1: {  // Process audio L + R LOW PASS FILTER
          BIQUAD(sbuffer2, sbuftmp0, len, bq[0].coeffs, bq[0].w);
          BIQUAD(sbuftmp0, sbufout2, len, bq[1].coeffs, bq[1].w);

          // Process audio L HIGH PASS FILTER
          BIQUAD(sbuffer0, sbuftmp0, len, bq[2].coeffs, bq[2].w);
          BIQUAD(sbuftmp0, sbufout0, len, bq[3].coeffs, bq[3].w);

          // Process audio R HIGH PASS FILTER
          BIQUAD(sbuffer1, sbuftmp0, len, bq[4].coeffs, bq[4].w);
          BIQUAD(sbuftmp0, sbufout1, len, bq[5].coeffs, bq[5].w);

          int16_t valint[5];
          for (uint16_t i = 0; i < len; i++) {
            valint[0] =
                (muteCH[0] == 1) ? (int16_t)0 : (int16_t)(sbufout0[i] * 32768);
            valint[1] =
                (muteCH[1] == 1) ? (int16_t)0 : (int16_t)(sbufout1[i] * 32768);
            valint[2] =
                (muteCH[2] == 1) ? (int16_t)0 : (int16_t)(sbufout2[i] * 32768);
            dsp_audio[i * 4 + 0] = (valint[2] & 0xff);
            dsp_audio[i * 4 + 1] = ((valint[2] & 0xff00) >> 8);
            dsp_audio[i * 4 + 2] = 0;
            dsp_audio[i * 4 + 3] = 0;

            dsp_audio1[i * 4 + 0] = (valint[0] & 0xff);
            dsp_audio1[i * 4 + 1] = ((valint[0] & 0xff00) >> 8);
            dsp_audio1[i * 4 + 2] = (valint[1] & 0xff);
            dsp_audio1[i * 4 + 3] = ((valint[1] & 0xff00) >> 8);
          }
          i2s_write_expand(0, (char *)dsp_audio, chunk_size, 16, 32,
                           &bytes_written, portMAX_DELAY);
          i2s_write_expand(1, (char *)dsp_audio1, chunk_size, 16, 32,
                           &bytes_written, portMAX_DELAY);
        } break;
        case dspfFunkyHonda: {  // Process audio L + R LOW PASS FILTER
          BIQUAD(sbuffer2, sbuftmp0, len, bq[0].coeffs, bq[0].w);
          BIQUAD(sbuftmp0, sbufout2, len, bq[1].coeffs, bq[1].w);

          // Process audio L HIGH PASS FILTER
          BIQUAD(sbuffer0, sbuftmp0, len, bq[2].coeffs, bq[2].w);
          BIQUAD(sbuftmp0, sbufout0, len, bq[3].coeffs, bq[3].w);

          // Process audio R HIGH PASS FILTER
          BIQUAD(sbuffer1, sbuftmp0, len, bq[4].coeffs, bq[4].w);
          BIQUAD(sbuftmp0, sbufout1, len, bq[5].coeffs, bq[5].w);

          uint16_t scale = 16384;  // 32768
          int16_t valint[5];
          for (uint16_t i = 0; i < len; i++) {
            valint[0] =
                (muteCH[0] == 1) ? (int16_t)0 : (int16_t)(sbufout0[i] * scale);
            valint[1] =
                (muteCH[1] == 1) ? (int16_t)0 : (int16_t)(sbufout1[i] * scale);
            valint[2] =
                (muteCH[2] == 1) ? (int16_t)0 : (int16_t)(sbufout2[i] * scale);
            valint[3] = valint[0] + valint[2];
            valint[4] = -valint[2];
            valint[5] = -valint[1] - valint[2];
            dsp_audio[i * 4 + 0] = (valint[3] & 0xff);
            dsp_audio[i * 4 + 1] = ((valint[3] & 0xff00) >> 8);
            dsp_audio[i * 4 + 2] = (valint[2] & 0xff);
            dsp_audio[i * 4 + 3] = ((valint[2] & 0xff00) >> 8);

            dsp_audio1[i * 4 + 0] = (valint[4] & 0xff);
            dsp_audio1[i * 4 + 1] = ((valint[4] & 0xff00) >> 8);
            dsp_audio1[i * 4 + 2] = (valint[5] & 0xff);
            dsp_audio1[i * 4 + 3] = ((valint[5] & 0xff00) >> 8);
          }
          i2s_write_expand(0, (char *)dsp_audio, chunk_size, 16, 32,
                           &bytes_written, portMAX_DELAY);
          i2s_write_expand(1, (char *)dsp_audio1, chunk_size, 16, 32,
                           &bytes_written, portMAX_DELAY);
        } break;
        default:
          break;
      }

      // if (cnt%100==0)
      //{ //ws_server_send_bin_client(0,(char*)audio, 240);
      // printf("%d %d \n",byteWritten, i2s_evt.size );
      //}
      // if (audio != NULL) vRingbufferReturnItem(s_ringbuf_i2s, (void *)audio);
      free(audio_pkt.samplebuf);
      chunk_size = 0;
      // ts_size = 0;
    }
  }
}
// buffer size must hold 400ms-1000ms  // for 2ch16b48000 that is 76800 -
// 192000 or 75-188 x 1024

// #define BUFFER_SIZE 192 * (3840 + 12)
 
//#define BUFFER_SIZE 192 * (3528 + 12)    // 44100/16/2
//(3840 + 12)  PCM 48000/16/2 

// 3852 3528

xTaskHandle dsp_i2s_task_init(uint32_t sample_rate, bool slave) {
  setup_dsp_i2s(sample_rate, slave);
  ESP_LOGI("I2S","Setup i2s dma and interface");
#ifdef CONFIG_USE_PSRAM
  //TODO Change PSRAM switch to allocation of audio sample buffers

  audio_sample_queue = xQueueCreate(100, sizeof(audio_pkt_element_t));

  printf("Ringbuf ok\n");
#else
  printf(
      "Setup ringbuffer using internal ram - only space for 150ms - Snapcast "
      "buffer_ms parameter ignored \n");
  s_ringbuf_i2s = xRingbufferCreate(32 * 1024, RINGBUF_TYPE_BYTEBUF);
#endif
  if (audio_sample_queue == NULL) {
    printf("nospace for ringbuffer\n");
    return;
  }
  printf("Ringbuffer ok\n");
  xTaskCreatePinnedToCore(dsp_i2s_task_handler, "DSP_I2S", 48 * 1024, NULL, 2,
                          &s_dsp_i2s_task_handle, 0);

  return s_dsp_i2s_task_handle;
  
}

void dsp_i2s_task_deinit(void) {
  if (s_dsp_i2s_task_handle) {
    vTaskDelete(s_dsp_i2s_task_handle);
    s_dsp_i2s_task_handle = NULL;
  }
  if (audio_sample_queue) {
    vRingbufferDelete(audio_sample_queue);
    audio_sample_queue = NULL;
  }
}

size_t write_ringbuf(const uint8_t *data, size_t size) {
  // BaseType_t done = xRingbufferSend(s_ringbuf_i2s, (void *)data, size,
                                    // (portTickType)portMAX_DELAY);
  BaseType_t done = xQueueSend(audio_sample_queue, data, 40);
  return (done) ? size : 0;
}

// ESP32 DSP processor
//======================================================
// Each time a buffer of audio is passed to the DSP - samples are
// processed according to a dynamic list of audio processing nodes.

// Each audio processor node consist of a data struct holding the
// required weights and states for processing an automomous processing
// function. The high level parameters is maintained in the structure
// as well

// Release - Prove off concept
// ----------------------------------------
// Fixed 2x2 biquad flow Xover for biAmp systems
// Interface for cross over frequency and level

void dsp_setup_flow(double freq, uint32_t samplerate) {
  float f = freq / samplerate / 2.0;

  bq[0] = (ptype_t){LPF, f, 0, 0.707, NULL, NULL, {0, 0, 0, 0, 0}, {0, 0}};
  bq[1] = (ptype_t){LPF, f, 0, 0.707, NULL, NULL, {0, 0, 0, 0, 0}, {0, 0}};
  bq[2] = (ptype_t){HPF, f, 0, 0.707, NULL, NULL, {0, 0, 0, 0, 0}, {0, 0}};
  bq[3] = (ptype_t){HPF, f, 0, 0.707, NULL, NULL, {0, 0, 0, 0, 0}, {0, 0}};
  bq[4] = (ptype_t){HPF, f, 0, 0.707, NULL, NULL, {0, 0, 0, 0, 0}, {0, 0}};
  bq[5] = (ptype_t){HPF, f, 0, 0.707, NULL, NULL, {0, 0, 0, 0, 0}, {0, 0}};
  bq[6] = (ptype_t){LOWSHELF, f, 6, 0.707, NULL, NULL, {0, 0, 0, 0, 0}, {0, 0}};
  bq[7] = (ptype_t){LOWSHELF, f, 6, 0.707, NULL, NULL, {0, 0, 0, 0, 0}, {0, 0}};

  pnode_t *aflow = NULL;
  aflow = malloc(sizeof(pnode_t));
  if (aflow == NULL) {
    printf("Could not create node");
  }

  for (uint8_t n = 0; n <= 7; n++) {
    switch (bq[n].filtertype) {
      case LOWSHELF:
        dsps_biquad_gen_lowShelf_f32(bq[n].coeffs, bq[n].freq, bq[n].gain,
                                     bq[n].q);
        break;
      case LPF:
        dsps_biquad_gen_lpf_f32(bq[n].coeffs, bq[n].freq, bq[n].q);
        break;
      case HPF:
        dsps_biquad_gen_hpf_f32(bq[n].coeffs, bq[n].freq, bq[n].q);
        break;
      default:
        break;
    }
    for (uint8_t i = 0; i <= 4; i++) {
      printf("%.6f ", bq[n].coeffs[i]);
    }
    printf("\n");
  }
}

void dsp_set_xoverfreq(uint8_t freqh, uint8_t freql, uint32_t samplerate) {
  float freq = freqh * 256 + freql;
  printf("%f\n", freq);
  float f = freq / samplerate / 2.;
  for (int8_t n = 0; n <= 5; n++) {
    bq[n].freq = f;
    switch (bq[n].filtertype) {
      case LPF:
        for (uint8_t i = 0; i <= 4; i++) {
          printf("%.6f ", bq[n].coeffs[i]);
        }
        printf("\n");
        dsps_biquad_gen_lpf_f32(bq[n].coeffs, bq[n].freq, bq[n].q);
        for (uint8_t i = 0; i <= 4; i++) {
          printf("%.6f ", bq[n].coeffs[i]);
        }
        printf("%f \n", bq[n].freq);
        break;
      case HPF:
        dsps_biquad_gen_hpf_f32(bq[n].coeffs, bq[n].freq, bq[n].q);
        break;
      default:
        break;
    }
  }
}
