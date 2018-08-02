# dali-driver

## Example Usage

```
#include "mbed.h"
#include "DALIDriver.h"

int main() {
    DALIDriver dali(D0, D2);
    int num_devices = dali.init();
    // Devices will be addressed [0, num_devices-1]

    // Add all devices to group 1
    for(int i = 0; i < num_devices; i++) {
        dali.add_to_group(i, 1);
    }

    // Get the group address of group 1
    uint8_t group_addr = dali.get_group_addr(1);

    dali.turn_on(group_addr); // turn entire group on 

    dali.set_fade_time(group_addr, 10);
    dali.set_fade_rate(group_addr, 5);

    // Set the scene 2 for the group to be at level 100
    dali.set_scene(group_addr, 2, 100);

    // Group go to scene 2
    dali.go_to_scene(group_addr, 2);
}
```
