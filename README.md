# DALI-based Driver for Mbed OS

A driver for DALI lights that adhere to iec62386-102. THe driver also supports DALI sensor instances part of iec62386-103, like presence and motion sensors. 

## Example Usage - Lights only configuration

```
#include "mbed.h"
#include "DALIDriver.h"

int main() {
    DALIDriver dali(D0, D2);
    dali.init();
    int num_devices = dali.get_num_lights();
    printf("Devices on bus: %d\r\n", num_devices);
    // Devices will be addressed [0, num_devices-1]

    // Add all devices to group 1
    for(int i = 0; i < num_devices; i++) {
        dali.add_to_group(i, 1);
    }

    // Get the group address of group 1
    uint8_t group_addr = dali.get_group_addr(1);

    dali.turn_off(group_addr); // turn entire group off

    dali.set_fade_time(group_addr, 12); // 28 seconds
    dali.set_fade_rate(group_addr, 5);  // 80 steps/second

    // Set the scene 2 for the group to be at level 254
    dali.set_scene(group_addr, 2, 254);

    // Group go to scene 2
    dali.go_to_scene(group_addr, 2);
    
    // wait for 5 seconds
    wait(5);
    
    // Turn off all lights using the broadcast address
    dali.turn_off(DALIDriver::broadcast_addr);
}
```

## Example Usage - Colored lights

```
#include "mbed.h"
#include "DALIDriver.h"

int main() {
    DALIDriver dali(D0, D2);
    dali.init();
    int num_devices = dali.get_num_lights();
    printf("Devices on bus: %d\r\n", num_devices);
    // Devices will be addressed [0, num_devices-1]

    for(int i = 0; i < num_devices; i++) {
        dali.turn_on(i);
        uint8_t channels = dali.query_rgbwaf_channels(i);
        printf("Channels for device %i: 0x%X\r\n", i, channels); 
        if (channels == 4) {
            printf("RGBD light\r\n");
            // Set red light
            dali.set_color(i, 255, 0, 0);
            wait(3);
            // Set green light
            dali.set_color(i, 0, 255, 0);
        }
        else if (dali.query_temperature_capable(i)) {
            printf("Temperature light\r\n");
            // Set very warm color
            dali.set_color(i, 2500);
            wait(3);
            // Set very cold color
            dali.set_color(i, 7042);
    }
}
```


## Example usage - Lights and sensors

```
#include "mbed.h"
#include "DALIDriver.h"

EventQueue eventQueue;

void handle_sensor(uint32_t data)
{
    printf("Sensor event message: %lu\r\n", data);
    event_msg m = controller.parse_event(data);
    if (m.inst_type == OCCUPANCY) {
        // a 1 in the LSb means motion detected
        int motion = (m.info & 1) ? 1 : 0;
        printf("Motion captured, occupancy status: %d\r\n", motion);
    }
}


int main() {
    DALIDriver dali(D0, D2);
    // Assign addresses to devices on the bus
    // Lights will be addressed [0, num_lights-1]
    // Instances will be addressed [num_lights, num_lights + num_inputs - 1]
    dali.init();
    
    // number of lights
    int num_lights = dali.get_num_lights();
    // number of input instances
    int num_inputs = controller.get_num_inputs();
    
    printf("Devices on bus: %d\r\n", num_lights + num_inputs);
    
    // Handle motion events
    dali.attach(eventQueue.event(handle_sensor));

    // Add all devices to group 1
    for(int i = 0; i < num_lights; i++) {
        dali.add_to_group(i, 1);
    }

    // Get the group address of group 1
    uint8_t group_addr = dali.get_group_addr(1);

    dali.turn_off(group_addr); // turn entire group off

    dali.set_fade_time(group_addr, 12); // 28 seconds
    dali.set_fade_rate(group_addr, 5);  // 80 steps/second

    // Set the scene 2 for the group to be at level 254
    dali.set_scene(group_addr, 2, 254);

    // Group go to scene 2
    dali.go_to_scene(group_addr, 2);
}
```

