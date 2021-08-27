/**
 * \par Copyright (C), 2012-2016, MakeBlock
 * @file    Me_Megapi_encoder_pid_pos.ino
 * @author  MakeBlock
 * @version V1.0.0
 * @date    2016/07/14
 * @brief   Description: this file is sample code for Megapi encoder motor device.
 *
 * Function List:
 *    1. uint8_t MeEncoderOnBoard::getPortB(void);
 *    2. uint8_t MeEncoderOnBoard::getIntNum(void);
 *    3. void MeEncoderOnBoard::pulsePosPlus(void);
 *    4. void MeEncoderOnBoard::pulsePosMinus(void);
 *    5. void MeEncoderOnBoard::setMotorPwm(int pwm);
 *    6. double MeEncoderOnBoard::getCurrentSpeed(void);
 *    7. void MeEncoderOnBoard::setSpeedPid(float p,float i,float d);
 *    8. void MeEncoderOnBoard::setPosPid(float p,float i,float d);
 *    7. void MeEncoderOnBoard::setPosPid(float p,float i,float d);
 *    8. void MeEncoderOnBoard::setPulse(int16_t pulseValue);
 *    9. void MeEncoderOnBoard::setRatio(int16_t RatioValue);
 *    10. void MeEncoderOnBoard::moveTo(long position,float speed,int16_t extId,cb callback);
 *    11. void MeEncoderOnBoard::loop(void);
 *    12. long MeEncoderOnBoard::getCurPos(void);
 *
 * \par History:
 * <pre>
 * <Author>     <Time>        <Version>      <Descr>
 * Mark Yan     2016/07/14    1.0.0          build the new
 * </pre>
 */

#include <MeMegaPi.h>


const float Pi = 3.1415926535897932384626433832795;
const float d = 0.064; //m
float dist = 5;                               //set the desired distance in m (will be set to 5m)
float rot = ((dist / (Pi * d))*360);         //required total angular rotation (in degrees)




MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);

void isr_process_encoder1(void)
{
  if(digitalRead(Encoder_1.getPortB()) == 0)
  {
    Encoder_1.pulsePosMinus();
  }
  else
  {
    Encoder_1.pulsePosPlus();;
  }
}

void isr_process_encoder2(void)
{
  if(digitalRead(Encoder_2.getPortB()) == 0)
  {
    Encoder_2.pulsePosMinus();
  }
  else
  {
    Encoder_2.pulsePosPlus();
  }
}

void setup()
{
  attachInterrupt(Encoder_1.getIntNum(), isr_process_encoder1, RISING);
  attachInterrupt(Encoder_2.getIntNum(), isr_process_encoder2, RISING);
  Serial.begin(115200);
  
  //Set PWM 8KHz
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);

  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);

  Encoder_1.setPulse(8);
  Encoder_2.setPulse(8);
  Encoder_1.setRatio(26.9);
  Encoder_2.setRatio(26.9);
  //Encoder_1.setPosPid(1.8,0,1.2);
  //Encoder_2.setPosPid(1.8,0,1.2);
 // Encoder_1.setSpeedPid(0.18,0,0);
 // Encoder_2.setSpeedPid(0.18,0,0); 
  
  
}

void turn_180() {                                   //turn function
      Encoder_1.move(180,70);        
      Encoder_2.move(180,70); 
      delay(2000);
      }

void backwards() {
      Encoder_1.move(-rot,70);                       //backwardsfunction
      Encoder_2.move(rot,70);
      delay(2000);
}

void forwards() {                                   //forwards function
       Encoder_1.RunTurns(1, 100);
       Encoder_2.RunTurns(-1, 100);
      //Encoder_1.move(rot,70);            
      //Encoder_2.move(-rot,70);      
      //delay(2000);
}

void reset_encoders()  {                            //resets encoder postion
      //Encoder_1.reset(SlOT1);
      //Encoder_2.reset(SlOT2);
      Encoder_1.setPulsePos(0);                    
      Encoder_2.setPulsePos(0);
      delay(1000);
}

void loop()
{ 
      reset_encoders();
      forwards();
      //reset_encoders();
      turn_180();
      //reset_encoders();
      forwards();             
  
  
  
  Encoder_1.loop();
  Encoder_2.loop();
  
  //Serial.println(rot);
  //Serial.println(dist);
  //Serial.print("Speed 1:");
  //Serial.print(Encoder_1.getCurrentSpeed());
  //Serial.print("   ,Speed 2:");
  //Serial.print(Encoder_2.getCurrentSpeed());
  //Serial.print("   ,CurPos 1:");
  Serial.print(Encoder_1.getCurPos());
  //Serial.print("   ,CurPos 2:");
  //Serial.println(Encoder_2.getCurPos());
  
}
