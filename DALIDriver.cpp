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
    return (num_logical_units != 0);
}
    
// Return number of logical units on the bus
int DALIDriver::assign_addresses() 
{
    int searchCompleted = false;
    int numAssignedShortAddresses = 0;
    int assignedAddresses[63] = {false}; 
    int highestAssigned = -1;
    send_command(DTR0, 0xFF);
    send_command(INITIALISE, 0x00);
    send_command(RANDOMISE, 0x00);
    wait_ms(100);
    
    while(true) {
        // Set the search address to the highest range
        set_search_address(0xFFFFFF);
        // Compare logical units search address to global search address
        send_command(COMPARE, 0x00);
        // Check if any device responds yes
        bool yes = check_response(YES);       
        // If no devices are unassigned, we are done
        if (!yes) {
            return numAssignedShortAddresses;
        }
        if(numAssignedShortAddresses < 63) {
            uint32_t searchAddr = 0xFFFFFF;
            for(int i = 23; i>=0; i--) {
                uint32_t mask = 1 << i;
                searchAddr = searchAddr & (~mask);
                set_search_address(searchAddr);
                send_command(COMPARE, 0x00);
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
                // We found a unit, let's program the short address with its index
                // Give it a temporary short address
                send_command(PROGRAM_SHORT_ADDR, (63 << 1) + 1);
                // Find the index
                int index = getIndexOfLogicalUnit(63);
                if (index < 63) {
                    if (assignedAddresses[index] == true) {
                        // Duplicate addr?
                    }   
                    else {
                        // Program index as short address
                        send_command(PROGRAM_SHORT_ADDR, (index << 1) + 1);
                        // Tell unit to withdraw
                        send_command(WITHDRAW, 0x00);
                        numAssignedShortAddresses++;
                        assignedAddresses[index] = true;
                        if(index > highestAssigned) 
                            highestAssigned = index;
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
