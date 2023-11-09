// Use this #define to indicate which UART/TTL port to use
#define UART_SERIAL Serial
void setup() {
// Setup the USB Serial connection
Serial.begin(115200);
// Setup the UART/TTL Serial
UART_SERIAL.begin(115200);
}
void loop() {
// Get byte from USB
if(Serial.available())
UART_SERIAL.write(Serial.read());
// Get byte from UART/TTL
if(UART_SERIAL.available())
Serial.write(UART_SERIAL.read());


}
