# Nuvation NUV300 BMS driver

## Bugfix for estimating SoC without measuring cell voltages

### Part A
This use-case of no individual cells connected to the BMS is unique and we are exploring new ways to use the SoC feature. It was not designed to work in this use-case, but it is an interesting opportunity to try to push it to the limits.

In your configuration file, please set stack_soc.vfull to 100000 and set stack_soc.vempty to -100000. This should completely remove the dependency on cell voltages.

One other catch is that without any individual cells connected, stack_cell_stat will not update. This likely needs to be functional, and you can make it tick by writing a 1 to the stack_cell_stat.update register. I believe this will need to be done once at start and it might not require this step upon each additional start.


### Part B
If the SoC gauge is flipping between 0% and 99% depending on if you are charging the supercap or discharging the supercap, then I think it is close to being functional. Here are a few things to look into:
- During a full charge, are you successfully reaching stack_soc.vfullavg when the charge current is equal to or more positive than stack_soc.ifull? stack_soc.ifull might need to be set to a more negative number to ensure the full condition is being met. SoC should be snapping to 100% if the full condition is being met, and it sounds like this is not happening.
- During a full discharge, are you successfully reaching stack_soc.vemptyavg? By the observed 0% SoC value, it sounds like it is reaching the empty condition properly but I thought I would ask for completeness.
- Using the Registers tab, what is the value of stack_soc.measured_capacity? If you think the number should be larger, please edit the register to set it back to the stack_soc.nominal_capacity number. This should fix the immediate snapping of the SoC value between 0% and 99% while a full charge cycle is performed. If the measured capacity is 0, that is an invalid number and must be changed back to the nominal capacity number before executing a full charge/discharge cycle.
 
We never designed the SoC to work in the use-case where no individual cells are connected. It sounds like it is almost working, which is further along than I’d thought we would be able to achieve. If the above suggestions do not resolve the SoC gauge issue, then I think it is safe to assume SoC does not work in this unique use-case.
