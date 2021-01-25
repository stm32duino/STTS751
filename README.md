# STTS751
Arduino library to support the STTS751 digital temperature sensor

## API

This sensor uses I2C to communicate.
For I2C it is then required to create a TwoWire interface before accessing to the sensors:  

    TwoWire dev_i2c(I2C_SDA, I2C_SCL);  
    dev_i2c.begin();

An instance can be created and enabled when the I2C bus is used following the procedure below:  

    STTS751Sensor Temp(&dev_i2c, I2C_ADDRESS);
    Temp.begin();
    Temp.Enable();

I2C_ADDRESS depends on the ID of the sensor and the pull-up resistor value connected to Addr/Therm pin.
Please check the Datasheet for more information.

The access to the sensor values is done as explained below:  

  Read temperature.  

    float temperature;
    Temp.GetTemperature(&temperature);

## Documentation

You can find the source files at  
https://github.com/stm32duino/STTS751

The STTS751 datasheet is available at  
https://www.st.com/content/st_com/en/products/mems-and-sensors/temperature-sensors/stts751.html
