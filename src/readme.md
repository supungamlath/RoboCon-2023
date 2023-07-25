Add these lines at the top of Adafruit_SSD1306.cpp in the Adafruit_SSD1306 library

```
#define I2C_SDA 15
#define I2C_SCL 5
```

Change this line

```
wire->begin();
```

to the following

```
wire->begin(I2C_SDA, I2C_SCL);
```
