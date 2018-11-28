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

#define DONE_FLAG (1UL << 0)

struct event_msg {
    uint8_t addr;
    uint8_t inst_type;
    uint16_t info;
};

class ManchesterEncoder {
public:
    // Flag data ready
    volatile bool data_ready;

    ManchesterEncoder(PinName out_pin, PinName in_pin, int baud,
                      bool idle_state = 0);

    // Blocking receive call
    int recv();

    void send_24(uint32_t data_out);

    void set_recv_frame_length(int num);

    void send(uint16_t data_out);

    void attach(mbed::Callback<void(uint32_t)> status_cb);

    void detach();

    void reattach();

private:
    void clear_interrupts();

    void stop();

    void irq_handler();

    void read_state();

    void rise_handler();

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
    EventFlags event_flags;

    Callback<void(uint32_t)> _sensor_event_cb;
    Callback<void(uint32_t)> _sensor_event_cb_save;
};

#endif
