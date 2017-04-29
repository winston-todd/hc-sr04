#Linux Driver for RadioShack Ultrasonic Range Sensor (sku 2760342) on ODROID-C1/C1+

Default signal GPIO = 102 (J2 - Pin18)

Use module parameters to override defaults:
```
sudo insmod range_sensor.ko [signal_gpio=xxx]
```


