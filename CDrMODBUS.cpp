#include "CDrMODBUS.h"
#include "Arduino.h"
#include <ESP8266WiFi.h> 

WiFiServer MODBUS_TCP_SERVER(502);


CDrMODBUS::CDrMODBUS() {
  currentIP_byte = 0;
  scanActive = false;
  scan_wait_time = 15000;
  initTable();
  debug_active = false;
  staticSlaveCount = 0;
  server_started = false;
  MODBUS_SERVER_CALLBACK_enabled = false;
}

void CDrMODBUS::beginServer() {
  MODBUS_TCP_SERVER.begin();
  server_started = true;
}
void CDrMODBUS::set_modbus_server_callback(TMODBUS_SERVER_CALLBACK callback) {
	MODBUS_SERVER_CALLBACK_enabled = true;
	MODBUS_SERVER_CALLBACK = callback;
}
void CDrMODBUS::serverUpdate() {
	if (!server_started) return;
	if (!modBUSClient.connected() ) {
		modBUSClient = MODBUS_TCP_SERVER.available();
	}
	else
	{
		if (debug_active) {
			//Serial.println("MOD BUS REQUEST ARRIVED");
		}
		size_t recv_len = modBUSClient.available();
		uint32_t now = millis();
		while (recv_len == 0 && real_millis(now) < 3000) {
			yield();
			yield();
			recv_len = modBUSClient.available();
		}
		if (recv_len == 0) {
			modBUSClient.stop();
			return;
		}
		uint8_t recv_buffer[recv_len];
		if (debug_active) {
			//Serial.print("modBUSClient.available()=");
			//Serial.println(recv_len);
		}
		modBUSClient.read(recv_buffer,recv_len);
		
		uint16_t transactionid = 256 * recv_buffer[0] + recv_buffer[1];
		uint16_t protocolid = 256 * recv_buffer[2] + recv_buffer[3];
		uint16_t size = 256 * recv_buffer[4] + recv_buffer[5];
		uint8_t  unitID = recv_buffer[6];
		uint8_t  functionCode = recv_buffer[7];
		uint16_t firstRegister = 256 * recv_buffer[8] + recv_buffer[9];
		uint16_t numberOfRegisters = 256 * recv_buffer[10] + recv_buffer[11];
		
		
		
		uint16_t sendbuf[numberOfRegisters];
		for (byte i=0;i<numberOfRegisters;i++) {
			sendbuf[i] = 65536;
		}
		
		size_t response_len = 9+(numberOfRegisters*2);
		uint8_t response[response_len];
		
		boolean selfHandle = false;
		if (unitID==1 && functionCode==3 && firstRegister==4000 && numberOfRegisters==1) {
			selfHandle = true;
			sendbuf[0] = scanFound;
		}
		if (unitID==1 && functionCode==3 && firstRegister >= 4001 && firstRegister <= 4256 && numberOfRegisters==1) {
			selfHandle==true;
			sendbuf[0] = staticSlavesUnits[devices[unitID].static_id];
		}
		
		if (unitID!=1) {				// forward request to detected devices
			selfHandle = true;
			
			uint32_t temp = getRegister32(staticSlavesUnits[devices[unitID].static_id],functionCode,firstRegister,numberOfRegisters);
			if (numberOfRegisters == 1) {
				sendbuf[0] = (uint16_t)temp;
			}
			if (numberOfRegisters==2) {
				sendbuf[0]  = temp >> 16;
				sendbuf[1] = temp - (temp<<16);
			}
		}
		
		if (selfHandle == false && MODBUS_SERVER_CALLBACK_enabled) {
			MODBUS_SERVER_CALLBACK(unitID,functionCode,firstRegister,numberOfRegisters,sendbuf);
		}
		
		response[0] = recv_buffer[0];   //transaction id high
		response[1] = recv_buffer[1];	//transaction id low
		
		response[2] = recv_buffer[2];	//protocolid id high
		response[3] = recv_buffer[3];	//protocolid id low
		
		response[4] = highByte(numberOfRegisters*2 + 2);	//length high
		response[5] = lowByte(numberOfRegisters*2 + 2);	//length low
		
		response[6] = recv_buffer[6];	// unit id
		response[7] = recv_buffer[7];	// modbus functionCode
		response[8] = numberOfRegisters*2;  //data length
		
		for (byte i=0;i<numberOfRegisters;i++) {
			response[i*2+9] = highByte(sendbuf[i]);
			response[i*2+10] = lowByte(sendbuf[i]);
		}
		modBUSClient.write(response,response_len);
		modBUSClient.stop();
		if (debug_active) {
			//Serial.println("MOD BUS RESPONSE SENT");
		}
		return;
	}
}
void CDrMODBUS::debug_enable(boolean flag) {
	debug_active = flag;
}
uint32_t CDrMODBUS::getRegister32(uint8_t unitID,uint8_t functionCode,uint16_t firstReg,uint16_t countReg) {
	return (uint32_t)getRegister64(unitID,functionCode,firstReg,countReg);
}
uint64_t CDrMODBUS::getRegister64(uint8_t unitID,uint8_t functionCode,uint16_t firstReg,uint16_t countReg) {
	if (scanFound==0) return 0;
	const uint8_t send_buffer[12] = {0,1,
                           0,0,
                           0,
                           6,
                           unitID,
                           functionCode,
                           highByte(firstReg),lowByte(firstReg),
                           highByte(countReg),lowByte(countReg)};

	
	String host;
	if (devices[unitID].is_static) {
		host = staticSlaves[devices[unitID].static_id];
	}
	else
	{
		IPAddress ip = WiFi.localIP();
		host = String(ip[0]) + "." + String(ip[1]) + "." + String(ip[2]) + "." + String(devices[unitID].device_ip);
	}
	WiFiClient client;
	if (client.connect(host.c_str(),502) ) {
		size_t sent = client.write(send_buffer,12);
		unsigned long now = millis();
		yield();
		delay(400);
		while (client.available() == 0) {
          delay(10);
			if (real_millis(now) > 10000) {
              client.stop();
              return 0;
			}
		}
		size_t recv_len = client.available();
		uint8_t recv[recv_len];
		client.read(recv,recv_len);
		client.stop();
		uint64_t ret = getNumber(recv,9,countReg);
		return ret;
	}
	else
	{
		client.stop();
		return 0;
	}
	return 0;
}
String CDrMODBUS::getHostOfUnit(uint8_t unitID) {
	String host;
	if (unitID>scanFound-1) return "";
	if (devices[unitID].is_static) host=staticSlaves[devices[unitID].static_id];
	if (!devices[unitID].is_static) {
		IPAddress ip = WiFi.localIP();
		WiFiClient client;
		host = String( ip[0] ) + "." + String(ip[1]) + "." + String(ip[2]) + "." + String(devices[unitID].device_ip);
	}
	return host;
}
uint64_t CDrMODBUS::getNumber(uint8_t *buf,byte offset,byte words) {
	uint64_t ret = 0;
	int word_size = words * 2;
	for (byte i=1;i<=word_size;i++) {
		uint64_t temp = buf[offset+i-1] << (word_size-i) * 8;
        ret+=temp;
    }
	return ret;   
}

void CDrMODBUS::endServer() {
	MODBUS_TCP_SERVER.begin();
}

void CDrMODBUS::beginClient() {
	
}

void CDrMODBUS::endClient() {
	
}
unsigned long CDrMODBUS::real_millis(unsigned long starttime) {
	unsigned long now = millis();
	unsigned long duration=0;
	if (now < starttime)
	{
		duration = 4294967295 - starttime + now;
	}
	else
	{
		duration = now - starttime;
	}
	return duration;  
}
void CDrMODBUS::initTable() {

	
	myIP_byte = WiFi.localIP()[3];  // exclude me from scanning
	scanFound = 0;
	
	for (byte i=0;i<255;i++) {
		devices[i].device_id = 0;
		devices[i].serial_number = 0;
		devices[i].unit_id = 0;
		devices[i].is_static = false;
		devices[i].static_id = 0;
		devices[i].monitor_enabled = false;
	}
	scanFound=1;
	
	devices[0].device_id = scanFound;
	devices[0].unit_id = scanFound;
	devices[0].device_ip = myIP_byte;
	
	
	for (uint8_t i=0;i<staticSlaveCount;i++) {
		scanFound++;
		devices[i+1].is_static = true;
		devices[i+1].static_id = i;
		devices[i+1].device_id = scanFound;
		devices[i+1].unit_id = staticSlavesUnits[i];
		devices[i+1].device_ip = myIP_byte;
	}
	
}
void CDrMODBUS::print_devices() {
	if (!debug_active) return;
	for (uint8_t i=0;i<scanFound;i++) {
		Serial.print("[");
		Serial.print(i);
		Serial.print("]= device_id:");
		Serial.print(devices[i].device_id,DEC);
		Serial.print(" unit_id:");
		Serial.print(devices[i].unit_id,DEC);
		Serial.print(" device_ip:");
		Serial.print(devices[i].device_ip,DEC);
		Serial.print(" is_static:");
		Serial.print(devices[i].is_static,DEC);
		
		if (devices[i].is_static) {
			Serial.print(" host:");
			Serial.print( staticSlaves[devices[i].static_id]);
		}
		Serial.println("");
	}
}
void CDrMODBUS::beginScanNetwork(uint32_t millis_repeat) {
	 myIP_byte = WiFi.localIP()[3];
	 scanActive = true;
	 scanFound = staticSlaveCount + 1;
	 currentIP_byte = 0;
	 scan_wait_time = millis_repeat;
	 scan_next_time = millis() + 1000;
	 if (debug_active) Serial.println("MODBUS SCANNET INIT");
}
void CDrMODBUS::update() {
	scanUpdate();
	yield();
	serverUpdate();
	yield();
}
void CDrMODBUS::addSlave(uint8_t unitID, String hostname) {
	staticSlaves[staticSlaveCount] = hostname;
	staticSlavesUnits[staticSlaveCount] = unitID;
	staticSlaveCount++;
	initTable();
}
void CDrMODBUS::scanUpdate() {
	if (!scanActive) return;
	
	if (millis() < scan_next_time) return;
	
	currentIP_byte++;
	if (currentIP_byte==myIP_byte) currentIP_byte++;
	if (currentIP_byte==0) { // this meas that we did a rollover after 255 searches
		endScanNetwork();
	}
	if (!scanActive) return;
	
	IPAddress ip = WiFi.localIP();
	WiFiClient client;
	String host = String( ip[0] ) + "." + String(ip[1]) + "." + String(ip[2]) + "." + String(currentIP_byte);
	if (debug_active) Serial.println("MODBUS SCAN HOST " + host);
	
	if (client.connect(host.c_str(),502) ) {
        client.stop();
        scanFound++;
		devices[scanFound].is_static = false;
		devices[scanFound].static_id = 0;
		devices[scanFound].device_id = scanFound;
		devices[scanFound].unit_id = scanFound;
		devices[scanFound].device_ip = currentIP_byte;
        return;
	}
	else
	{
        client.stop();
	}
	yield();
	scan_next_time = millis() + scan_wait_time;
}
void CDrMODBUS::endScanNetwork() {
	scanActive = false;
	
	if (debug_active) {
		Serial.print("Found "); Serial.print(scanFound,DEC); Serial.print(" devices. Static devices: ");Serial.println(staticSlaveCount,DEC);
		Serial.println("MODBUS SCANNET DONE");
	} 
}
boolean CDrMODBUS::scan_status() {
	return scanActive;
}