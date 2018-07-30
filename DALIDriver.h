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

class DALIDriver {
public:
    DALIDriver(PinName out_pin, PinName in_pin, int baud = 1200, bool idle_state = 1);
    bool init();

private:
    ManchesterEncoder encoder;
};

#endif
