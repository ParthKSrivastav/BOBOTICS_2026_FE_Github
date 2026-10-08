| Component               | Voltage | Current |
| ----------------------- | ------- | ------- |
| Raspberry Pi            | 5 V     | 3 A     |
| Motor controller        | 9 V     | 2 A     |
| Motors                  | 4 V     | 1 A     |
| TF-Luna distance sensor | 5 V     | 150 mA  |
| BnO085                  | 3.3V    | 5mA     |
| PiCam                   | 5V      | 150mA   |

## Power Distribution

Our robot uses two power paths. A 10,000 mAh power bank powers the Raspberry Pi, which supplies the sensors and camera. A separate 9 V supply powers the motor controller, which supplies the motors.

```mermaid
flowchart TD
    BANK["10,000 mAh power bank"]
    PI["Raspberry Pi<br/>5 V · 3 A"]

    SUPPLY["9 V power supply"]
    CONTROLLER["Motor controller<br/>9 V · 2 A"]
    MOTORS["Motors<br/>4 V · 1 A"]

    BNO["BNO085<br/>3.3 V · 5 mA"]
    LUNA["TF-Luna distance sensors × 2<br/>5 V · 150 mA listed"]
    CAMERA["PiCam<br/>5 V · 150 mA"]

    BANK -->|"5 V power"| PI

    PI -->|"3.3 V power"| BNO
    PI -->|"5 V power"| LUNA
    PI -->|"Camera connection: FFC ribbon cable (flat flexible cable)"| CAMERA

    SUPPLY -->|"9 V power"| CONTROLLER
    CONTROLLER -->|"Motor power output"| MOTORS

    classDef source fill:#263238,color:#ffffff,stroke:#263238
    classDef controller fill:#e3f2fd,color:#102a43,stroke:#1976d2
    classDef device fill:#f5f5f5,color:#212121,stroke:#616161

    class BANK,SUPPLY source
    class PI,CONTROLLER controller
    class BNO,LUNA,CAMERA,MOTORS device
