#include <CDrMODBUS.h>

#define MODBUS_REGISTER_DEVICE_ID 1
#define MODBUS_REGISTER_DAY_NUMBER 2
#define MODBUS_REGISTER_SECONDS 3

CDrMODBUS myBus;


void modbus_callback(uint8_t unitID,uint8_t functionCode,uint16_t firstReg,uint16_t countReg, uint16_t *send_buffer) {
  if (unitID==1 && functionCode==3 && firstReg==MODBUS_REGISTER_DEVICE_ID && countReg==2) {
     uint32_t temp = 1;
     send_buffer[0] = temp >> 16;
     send_buffer[1] = temp - (send_buffer[0]<<16);
     return;
  }
  if (unitID==1 && functionCode==3 && firstReg==MODBUS_REGISTER_DAY_NUMBER && countReg==2) {
     uint32_t temp = 365;
     send_buffer[0] = temp >> 16;
     send_buffer[1] = temp - (send_buffer[0]<<16);
     return;
  }
  if (unitID==1 && functionCode==3 && firstReg==MODBUS_REGISTER_SECONDS && countReg==2) {
     uint32_t temp = 3600;
     send_buffer[0] = temp >> 16;
     send_buffer[1] = temp - (send_buffer[0]<<16);
     return;
  }
}

void setup() {
  myBus.set_modbus_server_callback(modbus_callback);
  myBus.beginServer();
}

void loop() {
  myBus.update();
}
