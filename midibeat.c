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
#define ANTIBOUNCE_US   240000      // 0.24 sec = 240000 usec : used for switch anti-bouncing check
#define TIMEON_US       200000      // 0.20 sec : used as on/off time for leds 

#define MIDI_CLOCK  0xF8
#define NB_TICKS    24               // 24 ticks per beat (quarter note)
#define BEAT_120BPM_US  500000      // 1 beat @ 120BPM = 0.5 sec = 500000 usec
#define BEAT_40BPM_US   1500000     // 1 beat @ 40BPM = 1.5 sec = 1500000 usec
#define BEAT_240BPM_US  250000      // 1 beat @ 240BPM = 0.25 sec = 250000 usec

// On-board LED mapping. If no LED, set to NO_LED_GPIO
const uint NO_LED_GPIO = 255;
const uint LED_GPIO = 25;   // onboard led
const uint LED2_GPIO = 17;  // another led
// GPIO number for switch
const uint SWITCH_GPIO = 16;


// globals
static uint8_t midi_dev_addr = 0;
static uint64_t previous, now;    // time signals; time of previous tick, time now
static uint64_t nb_ticks, beat_120bpm_us, beat_40bpm_us, beat_240bpm_us;    // time signals; time of previous tick, time now


static bool test_switch(void)
{
    // test if switch has been pressed
    // in this case, line is down (level 0)
    if (gpio_get (SWITCH_GPIO)==0) {
        // anti bounce : if no bounce, then return ok
        if ((now-previous) > ANTIBOUNCE_US) return true;
    }
     
    // Light onboard LED on/off, depending where we are within time window
    if ((now - previous) < TIMEON_US) {
        if (NO_LED_GPIO != LED_GPIO) gpio_put(LED_GPIO, true);  // if onboard led and if we are within time window, lite LED on
        if (NO_LED_GPIO != LED2_GPIO) gpio_put(LED2_GPIO, true);  // if another led and if we are within time window, lite LED on
    }
    else {
        if (NO_LED_GPIO != LED_GPIO) gpio_put(LED_GPIO, false);  // if onboard led and if we are outside time window, lite LED off
        if (NO_LED_GPIO != LED2_GPIO) gpio_put(LED2_GPIO, false);  // if another led and if we are outside time window, lite LED off
    }
    return false;
}


static void send_midi_clock (bool connected)
{
    uint32_t nwritten;
    uint8_t buffer [1];               // buffer to send midi clocks

    // set buffer with midi clock
    buffer [0] = MIDI_CLOCK;    // add to buffer
   
    if (connected && tuh_midih_get_num_tx_cables(midi_dev_addr) >= 1)
    {
        nwritten = tuh_midi_stream_write(midi_dev_addr, 0, buffer, 1);
        if (nwritten != 1) {
            TU_LOG1("Warning: Dropped 1 byte\r\n");
        }
    }
}


static void choose_tempo(void)
// user can select tempo (number of switch press per beat/bar) by pressing switch at power on
// switch not pressed : standard "beat" mode, ie. 1 press per beat, ie. 4 press per bar
// switch pressed for 2 sec: compressed "beat" mode, 1 press per 1/2 beat, ie. 8 press per bar - this is very useful in case you have tempo/2 on your groovebox to save room
// (you store 2 times more things as you can store 2 bar in a single bar space, and play the bars at tempo/2).
// switch pressed for 4 sec: standard "bar" mode, 1 press per 4 beat, ie. 1 press per bar - useful if you prefer to click on bars and not on beats
// switch pressed for 6 sec: compressed "bar" mode, 1 press per 2 beat, ie. 2 press per bar - useful if you prefer to click on bars and not on beats, but your groovebox is set at tempo/2
{
	uint64_t press_on, press_off;    // time when switch is pressed on at startup, and when it is pressed off

        press_on = to_us_since_boot (get_absolute_time());       // get current time

		// loop if switch has been pressed
		// in this case, line is down (level 0)
		while (gpio_get (SWITCH_GPIO)==0);

        press_off = to_us_since_boot (get_absolute_time());       // get current time
		
		if (press_off-press_on < 2000000) {		// less than 2 sec
			// standard beat mode: 1 switch press per beat
			nb_ticks = NB_TICKS;
			beat_120bpm_us = BEAT_120BPM_US;
			beat_40bpm_us = BEAT_40BPM_US;
			beat_240bpm_us = BEAT_240BPM_US;
			return;
		}

		if (press_off-press_on < 4000000) {		// less than 4 sec
			// compressed beat mode: 2 switch press per beat
			nb_ticks = NB_TICKS / 2;
			beat_120bpm_us = BEAT_120BPM_US;
			beat_40bpm_us = BEAT_40BPM_US / 2;
			beat_240bpm_us = BEAT_240BPM_US / 2;
			return;
		}

		if (press_off-press_on < 6000000) {		// less than 6 sec
			// standard bar mode: 1 switch press per bar (1 per 4 beats)
			nb_ticks = NB_TICKS * 4;
			beat_120bpm_us = BEAT_120BPM_US * 4;
			beat_40bpm_us = BEAT_40BPM_US * 4;
			beat_240bpm_us = BEAT_240BPM_US * 4;
			return;
		}
	
		else {		// more than 6 sec
			// compressed bar mode: 2 switch press per bar (1 per 2 beats)
			nb_ticks = NB_TICKS * 2;				// = NB_TICKS * 4 / 2
			beat_120bpm_us = BEAT_120BPM_US * 4;
			beat_40bpm_us = BEAT_40BPM_US * 2;
			beat_240bpm_us = BEAT_240BPM_US * 2;
			return;
		}
}


int main() {
    
    uint64_t timediff, timedifftemp;
    bool connected;


    bi_decl(bi_program_description("USB host sending clock signals to external groovebox at press of a button."));
    bi_decl(bi_3pins_with_names(LED_GPIO, "On-board LED", LED2_GPIO, "Off-board LED", SWITCH_GPIO, "Switch"));

    board_init();
    printf("Pico midibeat\r\n");
    tusb_init();

    // Map the pins to functions
    gpio_init(LED_GPIO);
    gpio_set_dir(LED_GPIO, GPIO_OUT);
    gpio_init(LED2_GPIO);
    gpio_set_dir(LED2_GPIO, GPIO_OUT);
    gpio_init(SWITCH_GPIO);
    gpio_set_dir(SWITCH_GPIO, GPIO_IN);
    gpio_pull_up (SWITCH_GPIO);       // switch pull-up

	// Tempo can be modified at startup
	choose_tempo ();

    // Init variables
    now = to_us_since_boot (get_absolute_time());       // current time
    previous = now - beat_120bpm_us;                    // we assume at start that tempo is 120BPM in standard mode
    timediff = beat_120bpm_us / nb_ticks;


    // main loop
    while (1) {
        
        tuh_task();
        // check connection to USB slave
        connected = midi_dev_addr != 0 && tuh_midi_configured(midi_dev_addr);

        // Send clock and wait for sleeptime
        send_midi_clock (connected);
        sleep_us (timediff);

        now = to_us_since_boot (get_absolute_time());       // get current time

        if (test_switch ()) {
            // switch has been pressed

            timedifftemp = (now - previous) / nb_ticks;        // time between now and previous beat, divided by 24 (24 midi clock per beat)

            // it may be that we missed one switch press (switch may not be reliable 100%)
            // take this into account : in this case, (now - previous) shall be circa 2x more than expected; we put this limit at > 1.75x
            // check if timedifftemp < 1.75 * timediff : if true, then we did noot miss any press and new timing applies
            if (timedifftemp < (timediff + (timediff>>1) + (timediff>>2))) {
                timediff = timedifftemp;        // we assume no switch press was missed : update timediff value to new tempo
            }                                   // else we assume we missed 1 switch and we don't update timediff: we stick to current tempo

            // cap timediff to avoid to slow or too fast
            if (timediff > (beat_40bpm_us / nb_ticks)) timediff = beat_40bpm_us / nb_ticks;
            if (timediff < (beat_240bpm_us / nb_ticks)) timediff = beat_240bpm_us / nb_ticks;
            previous = now;
        }

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