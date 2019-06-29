# Configuration files

These are the four files modified while i was working with TI-RTOS:
* For initial configuration of *beaglebone* board:
	* *bbbAM335x.c*
* For peripheral pin configuration:
	* *am335x_beagleboneblack_pinmux_data.c*
* For board *pinmux* config: 
	* *bbbAM335x_pinmux.c*
* For clock configuration:
	* *am335x_prcm.c*

## Modifications

* Configuration of GPIO other modules 
	* All modules configured
	* Several GPIO pins of all modules were mapped, _but not all pins_
* Configuration of I2C modules
	* All modules configured

## Path of the modified files
```
/home/{user}/ti/pdk_am335x_1_0_14/packages/ti/board/src/bbbAM335x/bbbAM335x.c
/home/{user}/ti/pdk_am335x_1_0_14/packages/ti/starterware/board/am335x/am335x_beagleboneblack_pinmux_data.c
/home/{user}/ti/pdk_am335x_1_0_14/packages/ti/board/src/bbbAM335x/bbbAM335x_pinmux.c
/home/{user}/ti/pdk_am335x_1_0_14/packages/ti/starterware/soc/am335x/am335x_prcm.c
```