# LoRaWAN Driver for BL602 and BL604

Read the articles...

-   ["LoRaWAN on PineDio Stack BL604 RISC-V Board"](https://lupyuen.github.io/articles/lorawan2)

-   ["PineCone BL602 Talks LoRaWAN"](https://lupyuen.github.io/articles/lorawan)

-   ["The Things Network on PineDio Stack BL604 RISC-V Board"](https://lupyuen.github.io/articles/ttn)

Semtech LoRaWAN endpoint stack, ported to BL602 from Apache Mynewt OS...

https://github.com/apache/mynewt-core/tree/master/net/lora/node

BL602 LoRaWAN firmware with command-line interface is here...

[`pinedio_lorawan`: BL602 LoRaWAN Firmware](https://github.com/lupyuen/bl_iot_sdk/tree/master/customer_app/pinedio_lorawan)

To add this driver to an existing BL602 / BL604 project:

```bash
cd bl_iot_sdk/components/3rdparty
git submodule add https://github.com/lupyuen/lorawan
```

![BL602 LoRaWAN Driver](https://lupyuen.github.io/images/lorawan-driver.png)
