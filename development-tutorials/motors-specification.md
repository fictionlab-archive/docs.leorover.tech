# Motors specification

Leo Rover v. 1.6 \(as well as older Turtle Rover\) uses Buehler Motors motors \(sic!\) as the wheels actuators.

Here you have all the useful resources listed in case you need to access the motors specifications.

## Buehler Motors 1.61.077.414

### Specification

{% embed url="http://www.mobot.pl/download/1\_61\_077\_4xx\_en.pdf" %}

### CAD model

{% embed url="https://www.traceparts.com/en/product/buhler-motor-dc-gear-motor-planetary-gear-161077xxx-12-v-i-732?CatalogPath=BUHLER%3ABuhler.020&Product=10-15022011-099936&PartNumber=1.61.077.414" %}



## Pololu Romi 12 CPR magnetic encoders

### Connection

| Encoder pin | Mating connection |
| :--- | :--- |
| M1 | negative \(-\) pole of the motor & OUTA of Core2ROS |
| M2 | positive \(+\) pole of the motor & OUTB of Core2ROS |
| VCC | +5V pin of Core2-ROS |
| OUT A |  |
| OUT B |  |
| GND | GND pin of Core2-ROS |

### Dimension drawings

Mind the encoder magnets are modified to fit Buehler Motors shaft. Internal hole is redrilled to 3 mm \(instead of 1.5 mm\).

{% embed url="https://www.pololu.com/file/0J1210/romi-encoder-dimension-diagram.pdf" %}

### CAD models 

{% embed url="https://www.pololu.com/product/3542/resources" %}



