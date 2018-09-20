/* Manchester Encoder Library 
 * Copyright (c) 2018 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef MAN_ENCODING_H
#define MAN_ENCODING_H

#include "mbed.h"

class ManchesterEncoder {
public:
    // Flag data ready
    volatile bool data_ready;

    ManchesterEncoder(PinName out_pin, PinName in_pin, int baud, bool idle_state = 0)
        : _output_pin(out_pin),
          _input_pin(in_pin, PullUp) 
    {
        _idle_state = idle_state;
        _output_pin = idle_state;
        // Half bit time in seconds
        float time_s = 1.0/(2.0*(float)baud);
        // Half bit time in microseconds
        _half_bit_time =  (int)(time_s * 1000000.0);
        data_ready = false;
        recv_data = 0;
        bit_recv_total = 8;
    }
   
    // Blocking receive call 
    int recv() {
        // -1 means no data ready in timeout period
        int ret = -1; 
        // Wait for timeout between response
        wait_us(2400);
        // Calculate timer stop time, 9 recv bits, stop condition, half bit extra
        int stop_time = (_half_bit_time*2*9) + 2400 + _half_bit_time;
        // Start a timer
        Timer t;
        t.start();
        // While reading data
        while(rx_in_progress && t.read_us() < stop_time) {};
        t.stop();
        if(data_ready) {
            //If there is data, clear our buffer
            ret = recv_data;
            recv_data = 0;
        }
        return ret;
    }
    
    void send_24(uint32_t data_out) {
        // We don't want to be preempted because this is time sensitive 
        core_util_critical_section_enter();
        clear_interrupts();
        // Send start condition
        _output_pin = !_idle_state;
        wait_us(_half_bit_time);
        _output_pin = _idle_state;
        wait_us(_half_bit_time);
        // Send the data
        for(int i = 0; i < 24; i++) {
            // Send the actual MSb
            _output_pin = (bool)(data_out & 0x800000);
            wait_us(_half_bit_time);
            // Send the inverted MSb
            _output_pin = !((bool)(data_out & 0x800000));
            wait_us(_half_bit_time);
            // Shift to next bit
            data_out = data_out << 1;
        }
        // Send the stop condition
        _output_pin = _idle_state;
        core_util_critical_section_exit();
        _input_pin.rise(callback(this, &ManchesterEncoder::rise_handler));
        wait_us(13500);
    }

    void send(uint16_t data_out) {
        // We don't want to be preempted because this is time sensitive 
        core_util_critical_section_enter();
        clear_interrupts();
        // Send start condition
        _output_pin = !_idle_state;
        wait_us(_half_bit_time);
        _output_pin = _idle_state;
        wait_us(_half_bit_time);
        // Send the data
        for(int i = 0; i < 16; i++) {
            // Send the actual MSb
            _output_pin = (bool)(data_out & 0x8000);
            wait_us(_half_bit_time);
            // Send the inverted MSb
            _output_pin = !((bool)(data_out & 0x8000));
            wait_us(_half_bit_time);
            // Shift to next bit
            data_out = data_out << 1;
        }
        // Send the stop condition
        _output_pin = _idle_state;
        core_util_critical_section_exit();
        detach();
        bit_recv_total = 8;
        _input_pin.rise(callback(this, &ManchesterEncoder::rise_handler));
        wait_us(13500);
    } 
    
    void attach(mbed::Callback<void(uint8_t)> status_cb) {
        bit_recv_total = 24;
        _sensor_event_cb = status_cb;
        _input_pin.rise(callback(this, &ManchesterEncoder::rise_handler));
    }
    
    void detach() {
        _sensor_event_cb = NULL;
        clear_interrupts();
    }

private:
    
    void clear_interrupts() {
        _input_pin.rise(0);
        _input_pin.fall(0);
    } 

    void stop() {
        clear_interrupts();
        if(rx_in_progress){
            data_ready = true;
        }
        rx_in_progress = false;
        // Call sensor event handler
        if(_sensor_event_cb)
            _sensor_event_cb(recv_data);
        _input_pin.rise(callback(this, &ManchesterEncoder::rise_handler));
    }

    void irq_handler() { 
        clear_interrupts();
        rx_in_progress = true;
        data_ready = false;
        // Clear any stop timers
        t2.detach();
        t1.attach_us(callback(this, &ManchesterEncoder::read_state), 1.5*(float)_half_bit_time);
        t2.attach_us(callback(this, &ManchesterEncoder::stop), 2450);
    }

    void read_state() {
        int state = _input_pin.read();
        if (bit_count < bit_recv_total) {
            uint8_t mask = ((bool)state) << ((bit_recv_total-1)-bit_count++);
            recv_data |= mask;
        }
        if (state == 0) {
            _input_pin.rise(callback(this, &ManchesterEncoder::irq_handler));
        }
        else { 
            _input_pin.fall(callback(this, &ManchesterEncoder::irq_handler));
        }
    }
 
    void rise_handler() {
        bit_count = 0;
        clear_interrupts();
        // fall handler called in less than 1.5*_half_bit_time means start condition
        _input_pin.fall(callback(this, &ManchesterEncoder::irq_handler));
        // Stop condition if rise handler is not called in time
        t2.attach_us(callback(this, &ManchesterEncoder::stop), 1.5*(float)_half_bit_time);
    }
    
    // Pin to output encoded data
    DigitalOut _output_pin;
    // Pin to read encoded data
    InterruptIn _input_pin;
    // Half the time for each bit (1/(2*baud)) 
    int _half_bit_time;
    volatile uint32_t recv_data;
    volatile uint8_t bit_count;
    volatile bool rx_in_progress;
    // Total amount of bits expected
    volatile uint8_t bit_recv_total;
    bool _idle_state;
    Timeout t1;
    Timeout t2;
    
    Callback<void(uint8_t)> _sensor_event_cb;
};

#endif
 
