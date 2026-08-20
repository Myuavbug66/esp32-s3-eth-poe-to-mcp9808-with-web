# ESP32-S3 PoE MCP9808 Temperature Monitor

This project targets the ESP32-S3-ETH-PoE board. The W5500 Ethernet interface
provides the network connection and the MCP9808 sensor is read over I2C. The
web temperature page is served over the PoE Ethernet connection at the static
IP configured in `sdkconfig`.

USB is only needed for initial programming or serial debugging; it is not used
as the application network interface or power source.

## OTA firmware updates

Flash this OTA-enabled version once over USB. Future firmware binaries can be
uploaded over PoE Ethernet with PowerShell:

```powershell
Invoke-WebRequest -Uri http://192.168.1.136/ota -Method Post -InFile build/esp32-s3-eth-to-mcp9808-temp-001.bin -ContentType application/octet-stream
```

The board reboots automatically after a successful upload. Keep the computer
on the same network and use the static IP configured in `sdkconfig`.

## Telnet log monitor

With the board connected to PoE Ethernet, connect a Telnet client to port 23:

```powershell
telnet 192.168.1.136 23
```

The Telnet session receives the same `ESP_LOG` output as the USB monitor. Only
one Telnet client is supported; a new connection replaces the previous one.
