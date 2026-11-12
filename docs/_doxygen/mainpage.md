# DYNAMIXEL Driver API reference {#mainpage}

The classes, types and control tables of the DYNAMIXEL driver, generated from the headers in `inc/`.

Start with dynamixel::DYNAMIXEL, the bus master, and dynamixel::UartInterface, the base of the port you write. Errors
are dynamixel::Error inside dynamixel::Result; the configuration is dynamixel::Config. The packet layer is in
dynamixel::protocol and the control tables are in `dynamixel_tables.hpp`.

The guides, with examples that compile, are on the documentation site: <a href="../">DYNAMIXEL Driver
documentation</a>.
