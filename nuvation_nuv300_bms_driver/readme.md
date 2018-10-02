# Nuvation NUV300 BMS driver

## Bugfix for estimating SoC without measuring cell voltages
This use-case of no individual cells connected to the BMS is unique and we are exploring new ways to use the SoC feature. It was not designed to work in this use-case, but it is an interesting opportunity to try to push it to the limits.

In your configuration file, please set stack_soc.vfull to 100000 and set stack_soc.vempty to -100000. This should completely remove the dependency on cell voltages.

One other catch is that without any individual cells connected, stack_cell_stat will not update. This likely needs to be functional, and you can make it tick by writing a 1 to the stack_cell_stat.update register. I believe this will need to be done once at start and it might not require this step upon each additional start.
