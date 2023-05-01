# Advantech Boards ROS 2 wrapper

Welcome to the Advantech ROS 2 wrapper repository! This repository contains the source code and documentation of a ROS 2 wrapper for the Advantech devices. In our lab we have only the [Advantech 4718](https://www.advantech.com/en/products/1-2mlkno/usb-4718/mod_cb08e924-0165-46dd-8a50-0aebc79ea6c9) but I think the [SDK](https://www.advantech.com/en/support/details/driver?id=1-LXHFQJ) provided by Advantech is the same for all the boards so it **should** also work in your device.
 
## Table of Contents

- [Overview](#overview)
- [Hardware](#hardware)
- [Installation](#installation)
- [Usage](#usage)
- [Contributing](#contributing)

## Overview

The Advantech 4718 is a high-performance industrial GPIO board designed for use in harsh environments. This repository contains the software and documentation needed to use and configure the device. The [SDK](https://www.advantech.com/en/support/details/driver?id=1-LXHFQJ) provided by Advantech are supported up to Ubuntu 20.04 but I tested them and **they also work in Ubuntu 22.04**.

## Hardware
The repo has been tested in with **Ubuntu 22.04** and **ROS 2 Humble** with **Cyclone as DDS**. Further compatibility with other Linux/ROS Distros is not excluded but it has not been tested.

## Installation

To install the software and tools required to use the Advantech, follow these steps:

1. Clone the repository to your local machine:
   
    ```bash
    git clone https://github.com/pucciland95/advantech_usb4718.git
    ```

1. Navigate to the root directory of the repository.
    ```bash
    cd advantech_usb4718
    ```

2. Install any required dependencies using the provided instructions [here](https://www.advantech.com/en/support/details/driver?id=1-LXHFQJ) depending on the board you have.
   
3. Compile the package using:
    ```bash
    colcon build
    ```

## Usage

The Advantech 4718 can be used for a wide range of industrial applications. The package is provided with a launch file with just one argument (pay attention that the board should be connected to your computer via USB port):

- **`board_name`**: this is the identifier of your board. Change it accordingly to your needs.

Once launched the package provides the user with the following services:

  - **`reset_do`**: resets all the digital outputs of the board to LOW.
  - **`set_do_high`**: sets HIGH the digital outputs of a specific board port.
  - **`set_do_low`**: sets LOW the digital outputs of a specific board port.
  - **`print_do_status`**: prints on screen the DO status.


and the following topic:

  - **`DI_board_status`**: provides the user with the digital input status (HIGH/LOW) written in esadecimal form.
    

## Contributing

We welcome contributions to this repository! If you'd like to contribute, please follow these steps:

1. Fork the repository to your own account.
2. Make your changes in a new branch.
3. Submit a pull request to have your changes reviewed and merged.


