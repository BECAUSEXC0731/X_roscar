#include <Arduino.h>
#include "WiFi.h"
#include "WiFiUdp.h"
#include "Netprint.h"


extern WiFiUDP udp;
extern IPAddress serverIP;
void netPrintf(const char *format, ...)
{
  char buffer[128];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  udp.beginPacket(serverIP, serverPort);
  udp.print(buffer);
  udp.endPacket();
}
