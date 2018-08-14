# dali-driver

## Example Usage

```
#include "mbed.h"
#include "DALIDriver.h"

int main() {
    DALIDriver dali(D0, D2);
    int num_devices = dali.init();
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
}
```
