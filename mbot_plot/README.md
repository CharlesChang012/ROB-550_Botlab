## Overview
This repository contains a python script that reads in log files from the mbot and generates a plot of the position.

## How to use?
Run the following command to read the log:

```bash
$ python3 read_lcm_log.log -f [filename] -m [maze] -s [speed]
```

### Input Parameters
- **filename**
  
  **Description:** Path to the log file.  
  **Example:**  
  `my_log_file.log`

- **maze**
  
  **Description:** Choose the type of maze. You can select one of the following:
  - square
  - cp1

- **speed**
  
  **Description:** Choose the speed setting for the maze. You can select one of the following:
  - low
  - high
