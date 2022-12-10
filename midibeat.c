/**
 * @file midibeat.c
 * @brief A pico board acting as USB Host and sending clock signals to external groovebox at press of a button
 * 
 * MIT License

 * Copyright (c) 2022 denybear, rppicomidi

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 */
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "bsp/board.h"
#include "tusb.h"

// constants
#define MIDI_CLOCK  0xF8
#define NB_TICKS    24
#define BEAT_120BPM_US  500000      // 1 beat @ 120BPM = 0.5 sec = 500000 usec
#define BUFFER_SIZE 100             // 100 bytes as send buffer

// On-board LED mapping. If no LED, set to NO_LED_GPIO
const uint NO_LED_GPIO = 255;
const uint LED_GPIO = 25;
// GPIO number for switch
const uint SWITCH_GPIO = 16;


// globals
static uint8_t midi_dev_addr = 0;
static uint64_t clock_ticks [NB_TICKS];
static uint64_t previous, now, next, window;    // time signals; time of previous tick, time of next tick, time now, xx% of duration of last beat.   
static uint8_t sstate;                      // state of state machine
static  uint8_t buffer [BUFFER_SIZE];               // buffer to send midi clocks
static uint8_t posbuffer;       // position in buffer  


static bool test_switch(void)
{

    // test previous state of state machine
    // set sstate value to only value of its least significant bit
    sstate &= 0b00000001;
    
    // anti-bounce
    // you are allowed to press only if time now is > xx% of duration of last beat (xx% of time between 2 beats)
    if ((now - previous) < window) sstate |= 0b00000010;  
    
    // test if switch has been pressed
    if (gpio_get (SWITCH_GPIO)) {
        sstate |= 0b00000100;
printf ("switch %x\n\r", sstate);
    }
    
    // we can take switch press into account only if
    // switch is pressed, and 50% average timing of beat has elapsed since previous press, and previous state was unpressed
    if (sstate == 0b00000100) {
        // set new timings
        next = now + (now - previous);      // take previous beat duration, and determine when next beat will occur
        window = (now - previous) / 2;      // 50% of time between 2 beats
        window = window < 100000 ? window : 100000;     // cap max value of window to 100ms = 100000us
        previous = now;
        sstate |= 0b00000001;               // new state, indicating that previous state was NEXT BEAT
        return true;
    }
    // do not take switch into account; reset state
    else sstate &=0b11111110;

    // if no onboard LED, then leave
    if (NO_LED_GPIO == LED_GPIO) return false;
    
    // Light onboard LED on/off, depending where we are within time window (right after beat or not)
    if (sstate & 0b00000010) gpio_put(LED_GPIO, true);  // if we are within time window, lite LED on
    else gpio_put(LED_GPIO, false);
    return false;
}


static void send_midi_clock (bool connected)
{
    uint32_t nwritten;
    uint8_t i;

    // set buffer with midi clock
    buffer [posbuffer] = MIDI_CLOCK;    // add to buffer
    posbuffer ++;                       // increment position
    
    if (connected && tuh_midih_get_num_tx_cables(midi_dev_addr) >= 1)
    {
        nwritten = tuh_midi_stream_write(midi_dev_addr, 0, buffer, posbuffer);
        if (nwritten != posbuffer) {
            TU_LOG1("Warning: Dropped %lu bytes\r\n", posbuffer);
        }

        // adjust posbuffer position (and cap it)
        posbuffer -=nwritten;       // we sent only nwritten bytes; adjust posbuffer accordingly
        if (posbuffer >= BUFFER_SIZE) posbuffer = BUFFER_SIZE - 1;  // cap posbuffer

    }
}


int main() {
    
    uint8_t i;
    uint64_t timediff;
    bool connected;


    bi_decl(bi_program_description("USB host sending clock signals to external groovebox at press of a button."));
    bi_decl(bi_2pins_with_names(LED_GPIO, "On-board LED", SWITCH_GPIO, "Switch"));

    board_init();
    printf("Pico midibeat\r\n");
    tusb_init();

    // Map the pins to functions
    gpio_init(LED_GPIO);
    gpio_set_dir(LED_GPIO, GPIO_OUT);
    gpio_init(SWITCH_GPIO);
    gpio_set_dir(SWITCH_GPIO, GPIO_IN);
    gpio_pull_down (SWITCH_GPIO);       // switch pull-down

    // Init variables
    memset (clock_ticks, 0, NB_TICKS * sizeof (uint64_t));  // set table to 0
    memset (buffer, 0, BUFFER_SIZE * sizeof (uint8_t));
    posbuffer = 0;
    sstate = 0;                                             // start with state 0
    now = to_us_since_boot (get_absolute_time());       // current time
    previous = now - BEAT_120BPM_US;                    // we assume at start that tempo is 120BPM
    next = 0;
    window = BEAT_120BPM_US / 2;        // 50% of time between 2 beats


    // main loop
    while (1) {
        
        tuh_task();
        // check connection to USB slave
        connected = midi_dev_addr != 0 && tuh_midi_configured(midi_dev_addr);


        now = to_us_since_boot (get_absolute_time());       // get current time


        if (test_switch ()) {
            // switch has been pressed

            // will be useful as we need to compute timing of new ticks
            // at this point, now and next are both correctly set
            timediff = (next - now) / NB_TICKS;       // time between now and next beat, divided by 24 (24 midi clock per beat)

            // make sure we send remaining clocks events, if any
            for (i=0; i<NB_TICKS ; i++) {
                if (clock_ticks [i] != 0) {
                        send_midi_clock (connected);
                        // now need to clear clock_ticks [i] as we are going to recalculate them all
                }

                // compute timings for new clock events
                clock_ticks [i] = now + (timediff * i);
            }
        }
                    
        // check if it is time to send a midi clock signal
        // regardless switch has been pressed or not
        for (i=0; i<NB_TICKS ; i++) {
            if ((clock_ticks[i] != 0) && (now >= clock_ticks [i])) {
                    send_midi_clock (connected);
                    clock_ticks[i] = 0;
            }
        }

        // For test purpose
        //send_midi_clock (connected);
        //sleep_ms (20);

        // flush send buffer
        if (connected)
            tuh_midi_stream_flush(midi_dev_addr);
    }
}


//--------------------------------------------------------------------+
// TinyUSB Callbacks
//--------------------------------------------------------------------+

// Invoked when device with hid interface is mounted
// Report descriptor is also available for use. tuh_hid_parse_report_descriptor()
// can be used to parse common/simple enough descriptor.
// Note: if report descriptor length > CFG_TUH_ENUMERATION_BUFSIZE, it will be skipped
// therefore report_desc = NULL, desc_len = 0
void tuh_midi_mount_cb(uint8_t dev_addr, uint8_t in_ep, uint8_t out_ep, uint8_t num_cables_rx, uint16_t num_cables_tx)
{
  printf("MIDI device address = %u, IN endpoint %u has %u cables, OUT endpoint %u has %u cables\r\n",
      dev_addr, in_ep & 0xf, num_cables_rx, out_ep & 0xf, num_cables_tx);

  if (midi_dev_addr == 0) {
    // then no MIDI device is currently connected
    midi_dev_addr = dev_addr;
  }
  else {
    printf("A different USB MIDI Device is already connected.\r\nOnly one device at a time is supported in this program\r\nDevice is disabled\r\n");
  }
}

// Invoked when device with hid interface is un-mounted
void tuh_midi_umount_cb(uint8_t dev_addr, uint8_t instance)
{
  if (dev_addr == midi_dev_addr) {
    midi_dev_addr = 0;
    printf("MIDI device address = %d, instance = %d is unmounted\r\n", dev_addr, instance);
  }
  else {
    printf("Unused MIDI device address = %d, instance = %d is unmounted\r\n", dev_addr, instance);
  }
}

void tuh_midi_rx_cb(uint8_t dev_addr, uint32_t num_packets)
{
/*
    if (midi_dev_addr == dev_addr)
    {
        if (num_packets != 0)
        {
            uint8_t cable_num;
            uint8_t buffer[48];
            while (1) {
                uint32_t bytes_read = tuh_midi_stream_read(dev_addr, &cable_num, buffer, sizeof(buffer));
                if (bytes_read == 0) return;
                if (cable_num == 0) {
                    // do something as processing
                }
            }
        }
    }
*/
    return;
}

void tuh_midi_tx_cb(uint8_t dev_addr)
{
    (void)dev_addr;
}
