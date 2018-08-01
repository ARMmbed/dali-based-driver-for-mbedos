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

#include "DALIDriver.h"

DALIDriver::DALIDriver(PinName out_pin, PinName in_pin, int baud, bool idle_state)
    : encoder(out_pin, in_pin, baud, idle_state)
{
}

DALIDriver::~DALIDriver() 
{
    delete [] lights;
}

bool DALIDriver::add_to_group(LightUnit* light, uint8_t group)
{
    // Send the command to add to group
    send_command(light->addr, ADD_TO_GROUP + group);
    // Query upper or lower bits of gearGroups 16 bit variable
    uint8_t cmd = group < 8 ? QUERY_GEAR_GROUPS_L : QUERY_GEAR_GROUPS_H;
    // Send query command
    send_twice(light->addr, cmd);
    // Receive gearGroups variable
    uint8_t resp = encoder.recv(); 
    // Group bit will be set if this light is a memeber of that group
    uint8_t mask = 1 << group;
    bool contained = resp & mask;
    // Return whether light is part of group
    return contained;
}

bool DALIDriver::remove_from_group(LightUnit* light, uint8_t group)
{
    // Send the command to remove from group
    send_command(light->addr, REMOVE_FROM_GROUP + group);
    // Query upper or lower bits of gearGroups 16 bit variable
    uint8_t cmd = group < 8 ? QUERY_GEAR_GROUPS_L : QUERY_GEAR_GROUPS_H;
    // Send query command
    send_twice(light->addr, cmd);
    // Receive gearGroups variable
    uint8_t resp = encoder.recv(); 
    // Group bit will be set if this light is a memeber of that group
    uint8_t mask = 1 << group;
    bool contained = resp & mask;
    // Return whether light is not part of group
    return !contained;
}

void DALIDriver::set_level(LightUnit* light, uint8_t level) {
    // Change address to have 0 in LSb to signify 'direct arc power'
    uint8_t addr = light->addr & 0xFE;
    // Set the level of the light, will fade using programmed fade settings
    send_command(addr, level);
}

void DALIDriver::turn_off(LightUnit* light) {
    send_command(light->addr, OFF);
}

void DALIDriver::send_twice(uint8_t addr, uint8_t opcode) { 
    send_command(addr, opcode);
}

void DALIDriver::set_fade_time(LightUnit* light, uint8_t time) {
    send_command(DTR0, time);
    // Send twice command
    send_twice(light->addr, SET_FADE_TIME); 
}

void DALIDriver::set_fade_rate(LightUnit* light, uint8_t rate) {
    send_command(DTR0, rate);
    // Send twice command
    send_twice(light->addr, SET_FADE_RATE); 
}

void DALIDriver::set_scene(LightUnit* light, uint8_t scene) {
    send_command(DTR0, scene);
    // Send twice command
    // TODO: Do we really need to set DTR0? Pg. 50 spec 102
    send_twice(light->addr, SET_SCENE + scene); 
}

void DALIDriver::remove_from_scene(LightUnit* light, uint8_t scene) {
    send_twice(light->addr, REMOVE_FROM_SCENE + scene);
}

void DALIDriver::go_to_scene(LightUnit* light, uint8_t scene) {
    send_twice(light->addr, GO_TO_SCENE + scene);
}

void DALIDriver::send_command(uint8_t address, uint8_t opcode) 
{
    encoder.send((address << 8) | opcode);
}

bool DALIDriver::check_response(uint8_t expected) 
{
    uint8_t response = encoder.recv();
    return (response == expected);
}

int DALIDriver::getIndexOfLogicalUnit(uint8_t addr)
{
    send_command(DTR1, 0x00);
    send_command(DTR0, 0x1A);
    send_command(READ_MEM_LOC, (addr << 1) + 1);
    return encoder.recv();
} 

void DALIDriver::set_search_address(uint32_t val) 
{
    send_command(SEARCHADDRH, val >> 16);
    send_command(SEARCHADDRM, (val >> 8) & (0x00FF));
    send_command(SEARCHADDRL, val & 0x0000FF);
}

bool DALIDriver::init() 
{
    // TODO: does this need to happen every time controller boots?
    num_logical_units = assign_addresses();
    lights = new LightUnit[num_logical_units];
    for(uint8_t i = 0; i < num_logical_units; i++) {
        LightUnit light;
        // This makes address always for 'standard command' not for 'direct arc power'
        // section 7.7.2 of 102 spec
        light.addr = (i << 1) + 1;
        lights[i] = light;
    }
    return (num_logical_units != 0);
}
    
// Return number of logical units on the bus
int DALIDriver::assign_addresses() 
{
    int searchCompleted = false;
    uint8_t numAssignedShortAddresses = 0;
    int assignedAddresses[63] = {false}; 
    int highestAssigned = -1;
    send_command(DTR0, 0xFF);
    // Start initialization phase
    send_command(INITIALISE, 0x00);
    // Assign all units a random address
    send_command(RANDOMISE, 0x00);
    wait_ms(100);
    
    while(true) {
        // Set the search address to the highest range
        set_search_address(0xFFFFFF);
        // Compare logical units search address to global search address
        send_command(COMPARE, 0x00);
        // Check if any device responds yes
        bool yes = check_response(YES);       
        // If no devices are unassigned (all withdrawn), we are done
        if (!yes) {
            return numAssignedShortAddresses;
        }
        if(numAssignedShortAddresses < 63) {
            uint32_t searchAddr = 0xFFFFFF;
            for(int i = 23; i>=0; i--) {
                uint32_t mask = 1 << i;
                searchAddr = searchAddr & (~mask);
                // Set a new search address
                set_search_address(searchAddr);
                send_command(COMPARE, 0x00);
                // Check if any devices match
                bool yes = check_response(YES);
                if(!yes) {
                    //No unit here, revert the mask
                    searchAddr = searchAddr | mask;
                }
                // If yes, then we found at least one device
            }
            set_search_address(searchAddr);
            send_command(COMPARE, 0x00);
            bool yes = check_response(YES);
            if (yes) {
                // We found a unit, let's program the short address with a new address
                // Give it a temporary short address
                send_command(PROGRAM_SHORT_ADDR, (63 << 1) + 1);
                uint8_t new_addr = numAssignedShortAddresses;
                if (new_addr < 63) {
                    if (assignedAddresses[new_addr] == true) {
                        // Duplicate addr?
                    }   
                    else {
                        // Program new address as short address
                        send_command(PROGRAM_SHORT_ADDR, (new_addr << 1) + 1);
                        // Tell unit to withdraw (no longer respond to search queries)
                        send_command(WITHDRAW, 0x00);
                        numAssignedShortAddresses++;
                        assignedAddresses[new_addr] = true;
                        if(new_addr > highestAssigned) 
                            highestAssigned = new_addr;
                    }
                }
                else {
                    // expected < 63 ?
                }
            }
            else {
                // No device found
            }
        }
        // Refresh initialization state
        send_command(INITIALISE, 0x00);
    }

}
