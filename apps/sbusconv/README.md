# sbusconv

ESP32 serial-to-ether converter which uses ethernet PHYs. Supports LAN8720 and TK110 PHY. I'm using the hardwares [here](https://github.com/kazkojima/esp32-phy), though the any hardwares can be used with tiny changes on configuration or small fixes.

It converts the popular s.bus format serial RC signals to the RC UDP packets which Parrot bebop drone defines. The packet looks like:

```
struct __attribute__((packed)) rcpkt {
    uint32_t version;
    uint64_t timestamp_us;
    uint16_t sequence;
    uint16_t pwms[8];
};
```

One can easily change it to the mavlink packet, for example, if needed.

Currently no WiFi fuction is used, though you could easily add some controle or data transfer functions via WiFi, if you like.

There is only one unusual configuration item COPY_CH6_CH7 which copies the CH6 value to the CH7 value. My transmitter can't send CH7/CH8 values via s.bus although these channels are generally used for the convenient function like the emergency motor stop. This option makes it possible to use those function on my transmitter.
