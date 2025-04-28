# Dokumentation: Raspberry Pi Pico HIL Simulator für 3-Achsen-Roboter

## 1. Zweck

Dieses Programm verwandelt einen Raspberry Pi Pico in einen Hardware-in-the-Loop (HIL) Simulator. Er simuliert das physikalische Verhalten eines Roboters mit drei Linearachsen (X, Y, Z) und einem Greifer. Der Simulator empfängt Steuerbefehle (PWM, Richtung, Greifer öffnen/schließen) von einer separaten Steuerungs-Pico und sendet simulierte Sensorwerte (Endschalter, Greiferposition, Achspositionen über I2C) zurück an die Steuerung. Zusätzlich sendet der HIL-Simulator den aktuellen Zustand (X, Y, Z, Greiferstatus) über die USB-Serielle-Schnittstelle an einen angeschlossenen Computer zur Visualisierung oder Überwachung.

Das System verwendet FreeRTOS für das Task-Management auf dem HIL-Simulator-Pico.

## 2. Hardware-Setup

**Benötigte Komponenten:**

* **2x Raspberry Pi Pico:**
    * Ein Pico für den HIL-Simulator (auf den dieser Code geflasht wird).
    * Ein Pico für die Robotersteuerung (benötigt separate Steuerungssoftware).
* **Verbindungskabel:** Um die GPIOs der beiden Picos zu verbinden.
* **Computer:** Zum Kompilieren, Flashen und zur Überwachung der seriellen Ausgabe des HIL-Simulators.
* **USB-Kabel:** Zum Verbinden des HIL-Simulator-Picos mit dem Computer.
* **Stromversorgung:** Für beide Picos.

**Verbindungsplan (HIL-Simulator-Pico <-> Steuerungs-Pico):**

| Signal                | HIL Simulator Pin Name | HIL Simulator GPIO | Richtung (aus Sicht HIL Sim) | Steuerungs-Pico Pin | Funktion                                      |
| :-------------------- | :--------------------- | :----------------- | :--------------------------- | :------------------ | :-------------------------------------------- |
| **Achse X** |                        |                    |                              |                     |                                               |
| PWM                   | `PIN_X_PWM`            | 0                  | Input                        | Output              | PWM-Signal für Motor X                        |
| Richtung              | `PIN_X_DIR`            | 1                  | Input                        | Output              | Richtungssignal für Motor X                   |
| Endschalter Min       | `PIN_X_END_MIN`        | 2                  | Output                       | Input               | Simulierter Endschalter X Minimum erreicht    |
| Endschalter Max       | `PIN_X_END_MAX`        | 3                  | Output                       | Input               | Simulierter Endschalter X Maximum erreicht    |
| **Achse Y** |                        |                    |                              |                     |                                               |
| PWM                   | `PIN_Y_PWM`            | 6                  | Input                        | Output              | PWM-Signal für Motor Y                        |
| Richtung              | `PIN_Y_DIR`            | 7                  | Input                        | Output              | Richtungssignal für Motor Y                   |
| Endschalter Min       | `PIN_Y_END_MIN`        | 8                  | Output                       | Input               | Simulierter Endschalter Y Minimum erreicht    |
| Endschalter Max       | `PIN_Y_END_MAX`        | 9                  | Output                       | Input               | Simulierter Endschalter Y Maximum erreicht    |
| **Achse Z** |                        |                    |                              |                     |                                               |
| PWM                   | `PIN_Z_PWM`            | 12                 | Input                        | Output              | PWM-Signal für Motor Z                        |
| Richtung              | `PIN_Z_DIR`            | 13                 | Input                        | Output              | Richtungssignal für Motor Z                   |
| Endschalter Min       | `PIN_Z_END_MIN`        | 14                 | Output                       | Input               | Simulierter Endschalter Z Minimum erreicht    |
| Endschalter Max       | `PIN_Z_END_MAX`        | 15                 | Output                       | Input               | Simulierter Endschalter Z Maximum erreicht    |
| **Greifer** |                        |                    |                              |                     |                                               |
| Befehl Öffnen         | `PIN_GRIP_CMD_OPEN`    | 18                 | Input                        | Output              | Befehl zum Öffnen des Greifers                |
| Befehl Schließen      | `PIN_GRIP_CMD_CLOSE`   | 19                 | Input                        | Output              | Befehl zum Schließen des Greifers             |
| Sensor Offen          | `PIN_GRIP_SNS_OPEN`    | 20                 | Output                       | Input               | Simulierter Sensor: Greifer ist offen         |
| Sensor Geschlossen    | `PIN_GRIP_SNS_CLOSED`  | 21                 | Output                       | Input               | Simulierter Sensor: Greifer ist geschlossen   |
| **I2C Kommunikation** |                        |                    |                              |                     |                                               |
| I2C SDA               | `I2C_PIN_SDA`          | 4                  | Input/Output                 | Input/Output        | I2C Datenleitung                              |
| I2C SCL               | `I2C_PIN_SCL`          | 5                  | Input/Output                 | Input/Output        | I2C Taktleitung                               |
| **Masse** |                        |                    |                              |                     |                                               |
| GND                   | GND                    | GND                | ---                          | GND                 | Gemeinsame Masse ist **essenziell**!          |

**WICHTIG:** Stellen Sie sicher, dass die GND-Pins beider Picos miteinander verbunden sind!

## 3. Software-Setup

* **HIL-Simulator-Pico:**
    * Kompilieren Sie den bereitgestellten C-Code (`pico_hil_simulator_c`) mithilfe des Pico SDK, CMake und FreeRTOS.
    * Flashen Sie die resultierende `.uf2`-Datei auf den HIL-Simulator-Pico.
* **Steuerungs-Pico:**
    * Sie müssen eine eigene Steuerungssoftware für diesen Pico entwickeln. Diese Software ist **nicht** Teil dieses Projekts.
    * Die Steuerungssoftware muss:
        * Die PWM-, Richtungs- und Greifer-GPIOs als Ausgänge konfigurieren und ansteuern.
        * Die Endschalter- und Greifer-Sensor-GPIOs als Eingänge konfigurieren und lesen.
        * Den I2C-Bus als Controller (Master) initialisieren.
        * Die simulierten Achspositionen vom HIL-Simulator über I2C lesen (siehe Abschnitt 6).

## 4. Betrieb

1.  Verbinden Sie die beiden Picos gemäß dem Verbindungsplan.
2.  Verbinden Sie den HIL-Simulator-Pico über USB mit Ihrem Computer.
3.  Öffnen Sie ein serielles Terminalprogramm (z.B. Minicom, PuTTY, Thonny) auf Ihrem Computer und verbinden Sie sich mit dem seriellen Port des HIL-Simulator-Picos (Baudrate: 115200).
4.  Versorgen Sie beide Picos mit Strom.
5.  Der HIL-Simulator-Pico startet und gibt Initialisierungsmeldungen über die serielle Schnittstelle aus.
6.  Der HIL-Simulator beginnt, die Eingänge vom Steuerungs-Pico zu lesen und die Robotermechanik zu simulieren.
7.  Die simulierten Endschalter- und Greifer-Sensor-Ausgänge werden entsprechend dem simulierten Zustand gesetzt.
8.  Der I2C-Puffer wird kontinuierlich mit den aktuellen simulierten Achspositionen aktualisiert.
9.  Der aktuelle Simulatorstatus wird periodisch über die serielle USB-Schnittstelle ausgegeben.
10. Die Steuerungs-Pico kann nun über I2C die Positionen abfragen und basierend auf den gelesenen Sensorwerten (Endschalter, Greifer) und den gelesenen I2C-Positionen die PWM-, Richtungs- und Greiferbefehle an den HIL-Simulator senden.

## 5. Konfiguration (im Quellcode des HIL-Simulators)

Folgende Konstanten im C-Code können angepasst werden:

* `I2C_PERIPHERAL_ADDR`: I2C-Adresse des HIL-Simulators (Standard: 0x30).

## 6. I2C Kommunikationsprotokoll (Position lesen)

Die Steuerungs-Pico (I2C Master) kann die simulierten Achspositionen vom HIL-Simulator-Pico (I2C Slave, Adresse `I2C_PERIPHERAL_ADDR`) wie folgt lesen:

1.  **Adresse schreiben:** Senden Sie einen I2C-Schreibbefehl an die Adresse des HIL-Simulators. Das erste (und einzige) Datenbyte in diesem Schreibbefehl ist die Startadresse des "Registers", das Sie lesen möchten:
    * `I2C_REG_X_POS_START` (0x00) für die X-Position.
    * `I2C_REG_Y_POS_START` (0x04) für die Y-Position.
    * `I2C_REG_Z_POS_START` (0x08) für die Z-Position.
2.  **Daten lesen:** Senden Sie direkt danach einen I2C-Lesebebefehl an die Adresse des HIL-Simulators. Lesen Sie **4 Bytes**. Diese 4 Bytes repräsentieren den `float`-Wert der jeweiligen Achsposition (in der Byte-Reihenfolge des Picos, typischerweise Little-Endian). Der HIL-Simulator inkrementiert nach jedem gelesenen Byte automatisch die interne Registeradresse.

**Beispiel (Lesen der X-Position):**

* Controller -> HIL Sim (Write): `[Slave Addr + W]` `[0x00]`
* Controller -> HIL Sim (Read): `[Slave Addr + R]` -> HIL Sim sendet 4 Bytes zurück.

Die Steuerung muss die empfangenen 4 Bytes wieder zu einem `float`-Wert zusammensetzen.

## 7. Serielle Ausgabe (USB)

Der HIL-Simulator sendet kontinuierlich den aktuellen Zustand über die USB-Serielle-Schnittstelle diese wird von robot_sim.py ausgelesen und angezeigt.
