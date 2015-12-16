cob\_scan\_unifier
====================

General description
---------------------
This package implements a node that unifies scan messages from a given numer of laser scanners

Node: scan\_unifier\_node
---------------------

The actual node that unifies a given number of laser scans
#### Parameters
**input\_scans** *(List of std::string)*
 The names of the scan topics to subscribe to as list of strings.

**loop\_rate** *(double, default: 100.0 [hz])*
 The loop rate of the ros node.

#### Published Topics
**scan\_unified** *(sensor_msgs::LaserScan)*
 Publishes the unified scans.

#### Subscribed Topics
**input\_scan\_name** *(sensor_msgs::LaserScan)*
 The current scan message from the laser scanner with topic name specified via the parameter **input\_scan\_topics**


#### Services


#### Services called
