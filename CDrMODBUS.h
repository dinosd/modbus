/*
  CDrMODBUS.h - Library for modBus Server and Client
  Created by Constantinos Dafalias Nov 2016.
  Released into the public domain.
*/
#ifndef CDrMODBUS_h
#define CDrMODBUS_h
#include "Arduino.h"
#include <ESP8266WiFi.h>

#define MAX_MODBUS_CLIENTS 1
#define MAX_MODBUS_REGISTERS 1

struct MODBUS_DEVICE {
  uint16_t device_id;
  uint32_t serial_number;
  uint16_t unit_id;
  uint8_t device_ip;
  boolean is_static;
  uint8_t static_id;
  boolean monitor_enabled;
};





class CDrMODBUS {
  public:
    CDrMODBUS();
	void debug_enable(boolean flag);
    void beginServer();
    void endServer();
    void beginClient();
    void endClient();
    void beginScanNetwork(uint32_t millis_repeat);
	void endScanNetwork();
	void addSlave(uint8_t unitID, String hostname);
	uint64_t getRegister64(uint8_t unitID,uint8_t functionCode,uint16_t firstReg,uint16_t countReg);
	uint32_t getRegister32(uint8_t unitID,uint8_t functionCode,uint16_t firstReg,uint16_t countReg);
	boolean scan_status();
	void update();
	unsigned long real_millis(unsigned long starttime);
	void print_devices();
	typedef void (*TMODBUS_SERVER_CALLBACK) (uint8_t unitID,uint8_t functionCode,uint16_t firstReg,uint16_t countReg,uint16_t *buffer);
	void set_modbus_server_callback(TMODBUS_SERVER_CALLBACK callback);
  private:
    MODBUS_DEVICE devices[255];
	uint8_t currentIP_byte;
	uint8_t scanFound;
	uint8_t myIP_byte;
	boolean scanActive;
	void scanUpdate();
	void serverUpdate();
	void initTable();
	uint32_t scan_wait_time;
	uint32_t scan_next_time;
	uint8_t static_slaves;
	boolean debug_active;
	WiFiServer *server;
	String staticSlaves[10];
	uint8_t staticSlavesUnits[10];
	uint8_t staticSlaveCount;
	uint64_t getNumber(uint8_t *buf,byte offset,byte words);
	boolean server_started;
	WiFiClient modBUSClient;
	boolean MODBUS_SERVER_CALLBACK_enabled;
	TMODBUS_SERVER_CALLBACK  MODBUS_SERVER_CALLBACK;
	String getHostOfUnit(uint8_t unitID);
	
};

#endif

