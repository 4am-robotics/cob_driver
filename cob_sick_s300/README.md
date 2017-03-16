# cob_sick_s300
This package implements a driver for the Sick S300 Safety laser scanners.
It provides an implementation for both, the old (1.40) and the new (2.10) protocol.
Thus, the old Sick S300 Professional CMS as well as the new Sick S300 Expert are supported.

However, it does not cover the full functionality of the protocol:
- It only handles distance measurements properly
- It only handles no or only one configured measurement range field properly
- It does not handle I/O-data or reflector data
(though it reads the reflector marker field in the distance measurements)

See http://wiki.ros.org/cob_sick_s300 for more details.

## S300 Configuration
Here are a few notes about how to best configure the S300:
- Configure the RS422 output to 500kBaud (otherwise, the scanner only provides a lower frequency)
- Configure the scanner to Continuous Data Output
- Send data via one telegram
- Only configure distances, no I/O or reflector data (otherwise, the scanner only provides a lower frequency).
- Configuration of the measurement ranges
    - For protocol 1.40: only configure one measurement range field with the full range (-45° to 225°) with all values.
    - For protocol 2.10: do not configure a measurement range field
      (otherwise, the scanner only provides a lower frequency).
- If you want to only use certain measurement ranges, do this on the ROS side using e.g. the `cob_scan_filter`
located in this package as well.
