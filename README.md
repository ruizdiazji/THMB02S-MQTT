
Lectura de temperatura y humedad de un sensor THMB02S y publicacion en mqtt

                                                                      +5V
                                                                       ^
                        +-----------------------------------------+    |
               RX2 -----| RO                                  Vcc |----+----- Brown
                        |                                         |
               TX2 -----| DI               MAX485               A |---------- Yellow (or Green)
    ESP32               |               TTL to RS485              |
    BOARD       D4 -+---| DE                                    B |---------- Blue              TH-MB-02-S
                    |   |                                         |
                    +---| /RE                                 GND |----+----- Black
                        +-----------------------------------------+    |
                                                                       v
                                                                      GND

                                                                    
Outputs:

    I (555) modbus_example: Characteristic #0 Humidity: 31.9 %
    I (1595) modbus_example: Characteristic #1 Temperature: 27.1 C
    I (2635) modbus_example: Characteristic #2 Humidity calibration: 0.0 %
    I (3675) modbus_example: Characteristic #3 Temperature Calibration: 0.0 C
    I (4715) modbus_example: Characteristic #4 ModBus address: 1
    I (5755) modbus_example: Characteristic #5 ModBus baud rate: 4800 bauds

## Arquitectura (WiFi + MQTT + BLE)

El servicio de Modbus  `main/modbus_service.c` expone lecturas periódicas del sensor.  
`app_main` combina tres módulos:

- `app_config.[ch]`: guarda en NVS las credenciales WiFi, broker MQTT (URI, usuario, password) y el tópico a publicar.
- `ble_config.[ch]`: levanta un GATT server NimBLE (`THMB02S Config`) con 6 características (SSID, password, URI, user, pass, topic). Al escribir se persiste en NVS y se reinicia la red.
- `network_manager.[ch]`: inicializa WiFi STA y el cliente MQTT de ESP-IDF; publica `{temperature, humidity, addr, timestamp}` cada ~2s usando el tópico configurado y mantiene el reloj sincronizado vía SNTP.

Flujo sugerido:

1. Flashear, abrir BLE central (por ejemplo nRF Connect) y buscar `THMB02S Config`.
2. Escribir strings UTF-8 en cada característica (usar formato `mqtt://host:port` para la URI).
3. Al completar SSID + password + URI + topic, el equipo se conecta automáticamente al WiFi/MQTT y empieza a publicar.

Se puede personalizar los valores por defecto editando `app_config_set_defaults` y `sdkconfig.defaults`.

### Timestamps en las publicaciones

Cada mensaje MQTT incluye un campo `timestamp` (epoch en milisegundos, UTC). Para obtenerlo, el `network_manager` inicializa SNTP contra `pool.ntp.org`, solicita resincronización cada vez que el STA obtiene IP y después según `CONFIG_LWIP_SNTP_UPDATE_DELAY` (1 h por defecto). Si se necesita otro intervalo, se debe ajustar ese valor a traves de `idf.py menuconfig` o agregandolo a `sdkconfig.defaults`. Mientras no haya sincronización disponible, se publica `timestamp: 0`.
