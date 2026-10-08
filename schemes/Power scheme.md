| Component               | Voltage | Current |
| ----------------------- | ------- | ------- |
| Raspberry Pi            | 5 V     | 3 A     |
| Motor controller        | 9 V     | 2 A     |
| Motors                  | 4 V     | 1 A     |
| TF-Luna distance sensor | 5 V     | 150 mA  |
| BnO085                  | 3.3V    | 5mA     |
| PiCam                   | 5V      | 150mA   |

## Power Distribution

This diagram shows the power requirements listed for our Bobotics robot. It is a planning overview, not a wiring diagram: the power source and any voltage regulators still need to be specified.

```mermaid
flowchart TD
    SOURCE["Power source<br/>Voltage and capacity to be specified"]

    SOURCE --> V5["5 V supply"]
    SOURCE --> V9["9 V supply"]
    SOURCE --> V4["4 V supply"]
    SOURCE --> V33["3.3 V supply"]

    V5 --> PI["Raspberry Pi<br/>5 V · 3 A"]
    V5 --> LUNA["TF-Luna distance sensor<br/>5 V · 150 mA"]
    V5 --> CAM["PiCam<br/>5 V · 150 mA"]

    V9 --> DRIVER["Motor controller<br/>9 V · 2 A"]
    V4 --> MOTORS["Motors<br/>4 V · 1 A"]
    V33 --> IMU["BNO085<br/>3.3 V · 5 mA"]

    classDef source fill:#263238,color:#ffffff,stroke:#263238
    classDef supply fill:#e3f2fd,color:#102a43,stroke:#1976d2
    classDef component fill:#f5f5f5,color:#212121,stroke:#616161

    class SOURCE source
    class V5,V9,V4,V33 supply
    class PI,LUNA,CAM,DRIVER,MOTORS,IMU component
```

### Notes

- Voltage and current labels reproduce our recorded values.
- Branches group components by voltage; they do not specify physical connections.
- Confirm whether
