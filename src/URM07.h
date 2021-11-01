// # Author: Ricardo Arroyo (ricardo.arroyolopez@gmail.com)
// # based on DFRobot SW written by Strictus.zhang@dfrobot.com
// # Date: 28.02.2021
// # Product Name: URM07-UART Ultrasonic Sensor
// # SKU: SEN0153
// # version number: 1.0
// # Code Description: 20-750cm distance measurement, the received data is not verified


#define header_H    0x55 //Header
#define header_L    0xAA //Header
#define device_Addr 0x11 //Address
#define data_Length 0x00 //Data length
#define get_Dis_CMD 0x02 //Command: Read Distance
#define get_Temp_CMD 0x03 //Command: Read Temperature
#define checksumDist    (header_H+header_L+device_Addr+data_Length+get_Dis_CMD) //Checksum Distance
#define checksumTemp     (header_H+header_L+device_Addr+data_Length+get_Temp_CMD) //checksum Temperature

#define URM07_DEFAULT_ADDRESS 0x11

int URM07ReadDistance(Stream& mySerial) {
unsigned char i=0;
unsigned char Rx_DATA[8];
unsigned char CMD[6]={
  header_H,header_L,device_Addr,data_Length,get_Dis_CMD,checksumDist}; //Distance command package

  for(i=0;i<6;i++){
    mySerial.write(CMD[i]);
  }
  delay(150);  //Wait for the result
  i=0;
  while (mySerial.available()){  //Read the return data (Note: this demo is only for the reference, no data verification)
    Rx_DATA[i++]=(mySerial.read());
  }

  return ((Rx_DATA[5]<<8)|Rx_DATA[6]); //Read the distance value
}

float URM07ReadTemperature(Stream& mySerial) {
unsigned char i=0;
unsigned char Rx_DATA[8];
unsigned char CMD[6]={header_H,header_L,device_Addr,data_Length,get_Temp_CMD,checksumTemp}; //Temperature Command package
float temp10;

 for(i=0;i<6;i++){
    mySerial.write(CMD[i]);
    }
 delay(50);  //Wait Data Return
 i=0;
 while (mySerial.available()){  //Read returned Data (Note: Demo is just for Reference , no data verification)
    Rx_DATA[i++]=(mySerial.read());
    }
 temp10 = ((Rx_DATA[5]<<8)|Rx_DATA[6]);  //Read temperature Value (10 times)

 return temp10/10.0;
}
