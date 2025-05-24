> [!TIP]
> If, during project create, you omit to specify *use UART for stdio* - for printf() over serial port,
>  you can always add it later to CMakeLists.txt like this:
```
target_link_libraries(pololu_minImu_9
        pico_stdio_uart
        )
```

>[!IMPORTANT]
>Raspberry PI PIco Project Extension -> before **Debug Project** - always select Build Type to *Debug* (use Switch Build Type)
