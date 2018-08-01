/* DALI Driver  
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

#ifndef DALI_DRIVER_H
#define DALI_DRIVER_H

#include "mbed.h"
#include "manchester/encoder.h"

// Special commands that do not address a specific device
// These values will be used as address byte in DALI command
enum SpecialCommandOpAddr {
    SEARCHADDRH = 0xB1,
    SEARCHADDRM = 0xB3,
    SEARCHADDRL = 0xB5,
    DTR0 = 0xA3,
    DTR1 = 0xC3,
    DTR2 = 0xC5,
    INITIALISE = 0xA5,
    RANDOMISE = 0xA7,
    PROGRAM_SHORT_ADDR = 0xB7,
    COMPARE = 0xA9,
    WITHDRAW = 0xAB
};

// Command op codes 
enum CommandOpCodes {
    GO_TO_SCENE = 0x10,
    OFF = 0x00,
    ON_AND_STEP_UP = 0x08,
    QUERY_GEAR_GROUPS_L = 0xC0, // get lower byte of gear groups status
    QUERY_GEAR_GROUPS_H = 0xC1, // get upper byte of gear groups status
    READ_MEM_LOC = 0xC5,
    
    // Commands below are "send twice"
    SET_SCENE = 0x40,
    SET_FADE_TIME = 0x2E,
    SET_FADE_RATE = 0x2F,
    SET_MIN_LEVEL = 0x2B,
    REMOVE_FROM_SCENE = 0x50,
    REMOVE_FROM_GROUP = 0x70,
    ADD_TO_GROUP = 0x60,
    SET_MAX_LEVEL = 0x2A
};

#define YES 0xFF

class DALIDriver {
public:
    // Variables stored in light unit (page 47,Table 14 of iec62386-102
    struct LightUnit {
        uint8_t addr;
        uint8_t fadeTime;
        uint8_t lampOn;
        uint8_t actualLevel;
        uint8_t fadeRate;
        // physical minimum level corresponding to the minimum light output the control gear can operate at
        uint8_t PHM;
    };

    /** Constructor DALIDriver
     *
     *  @param out_pin      Output pin for the DALI commands
     *  @param in_pin       Input pin for responses from DALI slaves
     *  @param baud         Signal baud rate
     *  @param idle_state   The default state of the line (high for DALI) 
     */
    DALIDriver(PinName out_pin, PinName in_pin, int baud = 1200, bool idle_state = 1);
    
    ~DALIDriver();

    /** Initialise the driver 
    *
    *   @returns    true if successful 
    *       
    */
    bool init();

    
    /** Send a command on the bus 
    *
    *   @param address     The address byte for command
    *   @param opcode      The opcode byte 
    *
    */ 
    void send_command(uint8_t address, uint8_t opcode);

    bool add_to_group(LightUnit* light, uint8_t group);
    bool remove_from_group(LightUnit* light, uint8_t group);
    void set_level(LightUnit* light, uint8_t level);
    void turn_off(LightUnit* light);
    void set_fade_rate(LightUnit* light, uint8_t rate);
    void set_fade_time(LightUnit* light, uint8_t time);
    void set_scene(LightUnit* light, uint8_t scene);
    void remove_from_scene(LightUnit* light, uint8_t scene);
    void go_to_scene(LightUnit* light, uint8_t scene);
    void send_twice(uint8_t addr, uint8_t opcode);


    // Array of lights on bus
    LightUnit* lights;

private:

    /** Assign addresses to the logical units on the bus 
    *
    *   @returns    The number of logical units found on bus
    *
    *   NOTE: This process is mostly copied from page 82 of iec62386-102
    *   The addresses will be in the range [0, number units - 1]
    */ 
    int assign_addresses();

    /** Set the controller search address 
    * This address will be used in search commands to determine what 
    * control units have this address or a numerically lower address
    *
    *   @param val    Search address valued (only lower 24 bits are used)
    *
    */ 
    void set_search_address(uint32_t val);

    /** Check the response from the bus 
    *
    *   @param expected    Expected response from the bus
    *   @returns           
    *       True if response matches expected response
    *
    */ 
    bool check_response(uint8_t expected);

    /** Get the index of a control unit
    *
    *   @param addr     The address of the device
    *   @returns        The index of the device
    */
    int getIndexOfLogicalUnit(uint8_t addr);

    // The encoder for the bus signals
    ManchesterEncoder encoder;
    // The number of logical units on the bus
    int num_logical_units;
};

#endif
