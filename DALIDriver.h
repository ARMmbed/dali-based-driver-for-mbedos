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
    TERMINATE = 0xA1,
    WITHDRAW = 0xAB
};

// Command op codes 
enum CommandOpCodes {
    GO_TO_SCENE = 0x10,
    OFF = 0x00,
    ON_AND_STEP_UP = 0x08,
    QUERY_GEAR_GROUPS_L = 0xC0, // get lower byte of gear groups status
    QUERY_GEAR_GROUPS_H = 0xC1, // get upper byte of gear groups status
    QUERY_ACTUAL_LEVEL = 0xA0,
    QUERY_PHM = 0x9A,
    QUERY_FADE = 0xA5,
    READ_MEM_LOC = 0xC5,
    
    // Commands below are "send twice"
    SET_SCENE = 0x40,
    SET_FADE_TIME = 0x2E,
    SET_FADE_RATE = 0x2F,
    SET_MIN_LEVEL = 0x2B,
    REMOVE_FROM_SCENE = 0x50,
    REMOVE_FROM_GROUP = 0x70,
    ADD_TO_GROUP = 0x60,
    SET_SHORT_ADDR = 0x80,
    SET_MAX_LEVEL = 0x2A
};

#define YES 0xFF

class DALIDriver {
public:

    /** Constructor DALIDriver
     *
     *  @param out_pin      Output pin for the DALI commands
     *  @param in_pin       Input pin for responses from DALI slaves
     *  @param baud         Signal baud rate
     *  @param idle_state   The default state of the line (high for DALI) 
     */
    DALIDriver(PinName out_pin, PinName in_pin, int baud = 1200, bool idle_state = 0);
    
    ~DALIDriver();

    /** Initialise the driver 
    *
    *   @returns    the number of logical units on the bus 
    *       
    */
    int init();

    
    /** Send a standard command on the bus 
    *
    *   @param address     The address byte for command
    *   @param opcode      The opcode byte 
    *
    */ 
    void send_command_standard(uint8_t address, uint8_t opcode);
		
    /** Send a standard command on the bus 
    *
    *   @param address     The address byte for command
    *   @param opcode      The opcode byte 
    *
    */ 
    void send_command_special(uint8_t address, uint8_t opcode);

    /** Send a direct arc power command on the bus 
    *
    *   @param address     The address byte for command
    *   @param opcode      The opcode byte 
    *
    */ 
    void send_command_direct(uint8_t address, uint8_t opcode);

    /** Get the address of a group 
    *
    *   @param group_number    The group number [0-15]
    *   @returns
    *       8 bit address of the group
    *
    */ 
    uint8_t get_group_addr(uint8_t group_number);

    /** Add a device to a group 
    *
    *   @param addr    8 bit device address 
    *   @param group   The group number [0-15]
    *   @returns
    *       true if command success
    *
    */ 
    bool add_to_group(uint8_t addr, uint8_t group);

    /** Remove a device from a group 
    *
    *   @param addr    8 bit device address 
    *   @param group   The group number [0-15]
    *   @returns
    *       true if command success
    *
    */ 
    bool remove_from_group(uint8_t addr, uint8_t group);

    
    /** Set the light output for a device/group 
    *
    *   @param addr    8 bit address (device or group)
    *   @param level   Light output level [0,254]
    *   NOTE: Refer to section 9.3 of iec62386-102 for dimming curve
    *
    */ 
    void set_level(uint8_t addr, uint8_t level);

    /** Turn a device/group off 
    *
    *   @param addr    8 bit address (device or group)
    *
    */
    void turn_off(uint8_t addr);

    /** Turn a device/group on 
    *
    *   @param addr    8 bit address (device or group)
    *
    */
    void turn_on(uint8_t addr);

    /** Get the current light level 
    *
    *   @param addr    8 bit address (device or group)
    *   @returns
    *       Light level [0, 254] from QUERY ACTUAL LEVEL command
    *
    */
    uint8_t get_level(uint8_t addr);

    /** Get the fade time and fade rate 
    *
    *   @param addr    8 bit address (device or group)
    *   @returns
    *       The 8 bit representation of the fade time/fade rate from the QUERY FADE TIME/FADE RATE command 
    *       The answer shall be XXXX YYYYb, where XXXXb equals fadeTime and YYYYb equals fadeRate 
    *
    */
    uint8_t get_fade(uint8_t addr);

    /** Get the physical minimum 
    *
    *   @param addr    8 bit address (device or group)
    *   @returns
    *       Physical minimum light output the control gear can operate at [0, 254] from QUERY PHYSICAL MINIMUM command
    *
    */
    uint8_t get_phm(uint8_t addr);

    /** Set the fade rate for a device/group 
    *
    *   @param addr    8 bit address (device or group)
    *   @param rate    Fade rate [1, 15]
    *   NOTE: Refer to section 9.5.3 of iec62386-102 for fade rate calculation
    *
    */
    void set_fade_rate(uint8_t addr, uint8_t rate);

    /** Set the fade time for a device/group 
    *
    *   @param addr    8 bit address (device or group)
    *   @param rate    Fade time [1, 15]
    *   NOTE: Refer to section 9.5.2 of iec62386-102 for fade time calculation
    *
    */
    void set_fade_time(uint8_t addr, uint8_t time);


    /** Set the light output for a scene 
    *
    *   @param addr    8 bit address (device or group)
    *   @param scene   scene number [0, 15] 
    *   @param level   Light output level [0,254]
    *
    */ 
    void set_scene(uint8_t addr, uint8_t scene, uint8_t level);

    /** Remove device/group from scene 
    *
    *   @param addr    8 bit address (device or group)
    *   @param scene   scene number [0, 15] 
    *
    */ 
    void remove_from_scene(uint8_t addr, uint8_t scene);

    /** Go to a scene 
    *
    *   @param addr    8 bit address (device or group)
    *   @param scene   scene number [0, 15] 
    *
    */ 
    void go_to_scene(uint8_t addr, uint8_t scene);

    static const uint8_t broadcast_addr = 0xFF;

private:
    
    // Some commands must be sent twice, utility function to do that
    void send_twice(uint8_t addr, uint8_t opcode);
    
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
