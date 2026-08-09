# Estado de una estacion. Se publica de forma continua a 2 Hz.

uint8 IDLE=0
uint8 BUSY=1
uint8 DONE=2
uint8 ERROR=3

uint8 state
string station_id
string detail
builtin_interfaces/Time stamp
