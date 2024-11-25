/*!
 *@file findCard.ino
 *@brief Detect gestures
 *@details  
 *@copyright   Copyright (c) 2024 DFRobot Co.Ltd (http://www.dfrobot.com)
 *@license     The MIT license (MIT)
 *@author [fengli](li.feng@dfrobot.com)
 *@version  V1.0
 *@date  2024-08-09
 *@https://github.com/DFRobot/DFRobot_CardReader
*/


#include <DFRobot_CardReader.h>



DFRobot_CardReader card;

unsigned char UID[5],Temp[4]     ;
bool findCard(void)
{
    if(card.pcdRequest(0x52,Temp)==0)
    {
      if(Temp[0]==0x04&&Temp[1]==0x00)  
          Serial.println("MFOne-S50");
        else if(Temp[0]==0x02&&Temp[1]==0x00)
          Serial.println("MFOne -S70");
        else if(Temp[0]==0x44&&Temp[1]==0x00)
          Serial.println("MF-UltraLight");
        else if(Temp[0]==0x08&&Temp[1]==0x00)
          Serial.println("MF-Pro");
        else if(Temp[0]==0x44&&Temp[1]==0x03)
          Serial.println("MF Desire");
        else
          Serial.println("Unknown");
      //  Serial.println("SUCCESS!");


     return true; 
    }
    return false;
    //else Serial.println("Faile!");                                             
}
unsigned char  DefaultKey[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; 
uint8_t buffer[32];
uint8_t RevBuffer[16]={1,2,3,4,7};
void setup(){
	
  
  Serial.begin(115200);
  card.begin(15);
  card.pcdAntennaOn(); 

	
}


void loop(){
	delay(1000);
  if(findCard()){
     if(card.pcdAnticoll(buffer) == 0){
        Serial.print("UUID :");
		    Serial.print(buffer[0]);
        Serial.print(buffer[1]);
        Serial.print(buffer[2]);
        Serial.println(buffer[3]);
        if(card.pcdSelect(buffer) == 0){

          Serial.println("connect success");
      
          if(card.pcdAuthState(0x60, 5, DefaultKey, buffer) == 0)// 校验卡密码
          {
            Serial.println("key success");
           
            card.pcdWrite(5,RevBuffer);
            delay(1000);
            char status=card.pcdRead(5,buffer);
            Serial.print("data: ");
            for(uint8_t i = 0; i<6;i++){
              
              Serial.print(buffer[i]);
              Serial.print(" ");
            }
          }
        } 

        
      }


      
  }

}