# ESP-Hosted-MCU Wi-Fi Design & Implementation details


## Sequence Diagram for Wi-Fi communication

On a ESP chipset with native Wi-Fi, a Wi-Fi api call or network data
from the application is processed internally on the chip and a Wi-Fi
response is returned to the application.

```mermaid
sequenceDiagram
    box Grey Host With Native WI-Fi
    participant app as Application
    participant api as ESP-IDF Wi-Fi Library
    participant wifi as Wi-Fi Hardware
    end

    app ->> api : esp_wifi_xxx() or Network Data
    api ->> wifi : 
    Note over wifi : Do Wi-Fi action
    wifi -->> api : Wi-Fi response or Data
    api -->> app : Response or Network Data
```

Using Wi-Remote and ESP-Hosted, the Wi-Fi api call from the
application is converted into a Hosted Call and transported to the
slave. The slave converts the Hosted Call back into an Wi-Fi api
call. The response (optionally with data) is converted into a Hosted
Response and transported back to the host. On the host, the Hosted
Response is converted back into a Wi-Fi response (optionally with
data) is returned to the application.

For Network Data, Hosted does not do data conversion and only
encapsulates the data for transport.

```mermaid
sequenceDiagram
    box Grey Host with ESP-Hosted
    participant app as Application
    participant remote as Wi-Fi Remote
    participant hostedh as ESP-Hosted
    participant transporth as Host Transport
    end

    box SlateGrey Slave ESP-Hosted
    participant transports as Slave Transport
    participant hosteds as Slave Hosted
    participant api as ESP-IDF Wi-Fi Library
    participant wifi as Wi-Fi Hardware
    end

    app ->> remote : esp_wifi_xxx()
    remote ->> hostedh : esp_wifi_remote_xxx()
    app ->> hostedh : Network Data
    Note over hostedh : add Hosted header
    hostedh ->> transporth : 

    transporth ->> transports : SPI/SDIO

    transports ->> hosteds : 
    Note over hosteds : remove Hosted header
    hosteds ->> api : esp_wifi_xxx()
    api ->> wifi : Wi-Fi command
    hosteds ->> wifi : Network Data
    Note over wifi: Do Wi-Fi action
    wifi -->> hosteds : Network Data
    wifi -->> api : Wi-Fi response
    api -->> hosteds : Response
    Note over hosteds : add Hosted header
    hosteds -->> transports : 

    transports -->> transporth : SPI/SDIO

    transporth -->> hostedh : 
    Note over hostedh : remove Hosted header
    hostedh -->> app : Network Data
    hostedh -->> remote : Wi-Fi Command response
    remote -->> app : Response
```




