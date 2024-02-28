# sensirion_slf3s
Python driver for Sensirion SLF3S_XXXXX type flow sensors attached to a Sensirion SCC1-USB sensor cable. These come as part of the evaluations kits such as the one for the [SLF3S-0600F](https://www.sensirion.com/products/catalog/EK-SLF3S-0600F/) sensor. The cables communicate to the host via the SHDLC (Sensirion High-Level Data Link Control) protocol.  

## Usage
Just clone this repo and create a virtualenv which fulfils the requirements from requirements.txt. 
The API documentation can be found here: https://tnaegele.github.io/sensirion_slf3s/.

### Example
The following example code starts the sensor and prints periodically the measured values (requires at least Python 3.10):
~~~python
from sensirion_slf3s import slf3s,find_sensor_port
from sensirion_shdlc_driver import ShdlcSerialPort, ShdlcConnection
import time


sensor_port = find_sensor_port()

with (ShdlcSerialPort(port=sensor_port, baudrate=115200) as port, 
        slf3s(ShdlcConnection(port), slave_address=0) as device):

    device.start(water=True)
    
    while True:
        flow,temp,flag = device.get_last()
        print(f'flow rate: {flow} ul/min, temperature: {temp} C, flag: {flag}')
        time.sleep(0.1)
~~~

## Todo
- `find_sensor_port` function is still experimental and can only handle one connected sensor
## Useful links
- Sensor cable data sheet: https://sensirion.com/media/documents/EE77392F/65290BF6/LQ_DS_SCC1-RS485-USB_Datasheet.pdf
- SHDLC command set: https://media.digikey.com/pdf/Data%20Sheets/Sensirion%20PDFs/LQ_CO_RS485SensorCable_SHDLC_Commands_D2.pdf
- Sensor data sheet: https://www.sensirion.com/media/documents/C4F8D965/65290BC3/LQ_DS_SLF3S-0600F_Datasheet.pdf
- SHDLC base Python driver: https://sensirion.github.io/python-shdlc-driver/shdlc.html
