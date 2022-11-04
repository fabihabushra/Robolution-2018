#define sensorNum 8
#define rotationSpeed 180
int maxSpeed = 250; //185

int trueSpeed = 250;
int sonarSpeed = 180;

int blackLimit[sensorNum];

const int motorPin1 = 5,motorPin2 = 6;        //right motor
const int motorPin3 = 9,motorPin4 = 10;       //left motor

float error, prevError=0;
float mappedValue, targetValue = 7;     

float safety=0.35;

float kp=70;                         //60
float kd=70;                      //200
                              
float sonar_kp = 25;
float sonar_kd = 40;

int motorResponse;
float correction;

int digitalReading[sensorNum];
int leftSpeed,rightSpeed;
int pidAllBlack=0,rightIRsBlack=0,stopAllBlack=0 ;
int leftIR=0,rightIR=0;
int contiBlack=30,stopCounter=0;
int contiWhite=10,resetIRCounter=0;
float time=3;

int danceCount=0;

int prev, curr, diff;
int danceDelay = 300;
int gapDelay = 2000; 

const int rSonarTrig = 12;
const int rSonarEcho = 11;

const int fSonarTrig = 8;
const int fSonarEcho = 7;

const int lSonarTrig = 4;
const int lSonarEcho = 3;

int sonarTimer=0;

long fDistance;
long rFDistance;
long rBDistance;
long difference;

int caveSpeed=150;

int sonarFinishKey=0,ringCompleteKey=0,sonarActive=0;

void setup()
{
  //initialize IR pins
  for(int i = 0; i < sensorNum; i++)
  {
    pinMode(A0 + i, INPUT);
  }

  //initialize motor pins
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT); 
  pinMode(motorPin3, OUTPUT);
  pinMode(motorPin4, OUTPUT);

  pinMode(fSonarTrig, OUTPUT);
  pinMode(fSonarEcho, INPUT);
  pinMode(rSonarTrig, OUTPUT);
  pinMode(rSonarEcho, INPUT);
  pinMode(lSonarTrig, OUTPUT);
  pinMode(lSonarEcho, INPUT);  
  
    Serial.begin(38400); //8F56
calibration();
  

}



void loop()
{
 
//sensorRead();

sensorMapping();
/*
Serial.println(mappedValue);
Serial.println();
Serial.print(leftSpeed);
Serial.print(" ");
Serial.print(rightSpeed);
Serial.println();
*/
// For IR Reading
for(int i = 0; i < sensorNum; i++)


{
  //Serial.print(analogRead(A0+i));
  if(analogRead(A0+i)<blackLimit[i])
 Serial.print("B");
 else Serial.print("W");
 Serial.print(" ");
 Serial.print(analogRead(A0+i));
 Serial.print(" && ");
 Serial.print(blackLimit[i]);
 Serial.print(" ||| ");
}
Serial.println();
Serial.print(leftIR);
Serial.print(" & ");
Serial.println(rightIR);



  if(stopCounter>contiBlack)
  {
  while(stopAllBlack==1){brake(); sensorMapping();}
  }

if(sonarFinishKey==0 && ringCompleteKey>1)
{
  
  triggerSonars();
  kp = sonar_kp;
  kd = sonar_kd;
  maxSpeed = sonarSpeed;
    
  if (fDistance<20 && fDistance >5)
  {
    sonarActive = 1;
    brake();
    delay(50);
    for(int i=0;i<3;i++)
    {
      delay(10);
      triggerSonars();
      if ( fDistance>20 )
      {
        sonarActive=0;
      }
      
    }
  
    if(sonarActive==1)
    {
    maxSpeed = trueSpeed;
    curveRun();
    }
    else
    {
      sensorMapping();
      pid();
      motor(leftSpeed,rightSpeed);
    }
     
  }
}
  //ring condition
  if(digitalReading[3]==1  && digitalReading [7]==1 || digitalReading[0] == 1 && digitalReading[6] == 1)
  {

        plannedCRotate();          
        delay(300);
        
        sensorMapping();
        

    pid();
    motor(leftSpeed,rightSpeed);      
    ringCompleteKey+=1;

  }

  
 if(mappedValue!=100)
   {  

      pid();
      motor(leftSpeed,rightSpeed);
      
   }
   
 else 
  { 

      
       
       if (leftIR==0 && rightIR==1) 
      {{while((mappedValue==100)) 
      {plannedCRotate();  
      sensorMapping();
      } 
      pid(); 
      motor(leftSpeed,rightSpeed);
      } rightIR=0;}
      
      else  if (leftIR==1 && rightIR==0) 
      {{while ((mappedValue==100))
      {plannedACRotate();  
      sensorMapping();
      } 
      pid(); 
      motor(leftSpeed,rightSpeed);
      } leftIR=0;}
      
      else 
      {
        if(sonarFinishKey==0)
        {  
          forwardDance();
          danceCount+=1;
        }
        else
        {
          pid(); 
          motor(maxSpeed,maxSpeed);  
        }
      } 

   } 
 } 
 

  


 


void pid()
{
  
  error=targetValue-mappedValue;
  correction=(kp*error)+(kd*(error-prevError));
  prevError=error;
  motorResponse=(int)correction;
 
 if(motorResponse>maxSpeed) motorResponse=maxSpeed;
 
if(motorResponse<-maxSpeed) motorResponse=-maxSpeed;

   if(motorResponse>0)
  {
    rightSpeed=maxSpeed;
    leftSpeed=maxSpeed-motorResponse;
  }
  else 
  {
    rightSpeed=maxSpeed+ motorResponse;
    leftSpeed=maxSpeed;
  }

}

void motor(int left, int right)
{
  
  if(right>0)
  {
  analogWrite(motorPin1,right);
  analogWrite(motorPin2,0);
  }
  else
  {
    analogWrite(motorPin1,0);
    analogWrite(motorPin2,-right);
  }

  if(left>0)
  {
  analogWrite(motorPin3,left);
  analogWrite(motorPin4,0);
  }
  else
  {
   analogWrite(motorPin3,0);
   analogWrite(motorPin4,-left); 
  }

 }



void brake(void)
{
  analogWrite(motorPin1, 0);
  analogWrite(motorPin2, 0);
  analogWrite(motorPin3, 0);
  analogWrite(motorPin4, 0);
}

void plannedACRotate()
{
  analogWrite(motorPin1,rotationSpeed);
  analogWrite(motorPin2, 0);
  analogWrite(motorPin3, 0);
  analogWrite(motorPin4,rotationSpeed);

}

void plannedCRotate()
{
  analogWrite(motorPin1,0);
  analogWrite(motorPin2, rotationSpeed);
  analogWrite(motorPin3, rotationSpeed);
  analogWrite(motorPin4,0);

}


void forwardDance(void)
{

int loopCounter = (int) (danceDelay / diff);
int gapSearch = (int) (gapDelay/diff);

  for(int i = gapSearch; i > 0; i--)
  {
    motor(150,150);
    
    if(mappedValue != 100)
    {
      pid();
      motor(leftSpeed, rightSpeed);
      return;
    }
     
  }
  
  
  
  for(int i = loopCounter; i > 0; i--)
  {

  plannedCRotate();
  sensorMapping();

   if(mappedValue != 100)
   {
    pid();
    motor(leftSpeed, rightSpeed);
    return;
   }

  }

  while(1)
  {
  plannedACRotate();
  sensorMapping();

   if(mappedValue != 100)
   {
    pid();
    motor(leftSpeed, rightSpeed);
    return;
   }

  }
  
}



long mstocm(long microseconds)
{
 
  return (microseconds*346.3)/2/10000;
}

// variables used triggered time oldTime interruptVariable distance

long trigger(int trigPin,int echoPin)
{
  unsigned long oldTime;
  unsigned long time;
  unsigned long timeLimit=2000;

  digitalWrite(echoPin,LOW);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  
  while(digitalRead(echoPin)==LOW); 
  
  oldTime=micros();
  while(digitalRead(echoPin)==HIGH&&(micros()-oldTime<timeLimit));
  time=micros()-oldTime;

  
  long distance =  mstocm(time);

  Serial.print("               ");
  //Serial.println(distance);
  
  return distance;
}


void triggerSonars()
{
  Serial.println("             ");
  int sonarDelay=10;  //use 50 if 50 works fine
  


  
  if(millis()-sonarTimer<sonarDelay)
   delay(sonarDelay-millis()-sonarTimer);

  fDistance=trigger(fSonarTrig,fSonarEcho);
  rFDistance=trigger(rSonarTrig,rSonarEcho);
  rBDistance=trigger(lSonarTrig,lSonarEcho);
  difference=rFDistance-rBDistance;
  sonarTimer=millis();
  Serial.println("           ");  
}

void curveRun()
{
  plannedCRotate();
  delay(300);
  motor(maxSpeed,maxSpeed-20);
  delay(350);
  plannedACRotate();
  delay(300);
  motor(maxSpeed,maxSpeed-20);
  delay(300);

  triggerSonars();
  delay(10);
  while(fDistance>15 || fDistance<5)
  {
      triggerSonars(); 
      delay(10);
  }
  plannedACRotate();
  delay(300);
  motor(maxSpeed,maxSpeed-20);
  delay(300);    
  caveCode();
  
}

void caveCode()
{
  sensorMapping();

  /*
  while(mappedValue==100)
  {
    triggerSonars();
    delay(10);
    Serial.print("Difference: ");
    Serial.print(difference);
    Serial.println();    
    if(difference>2 && difference<80)
    {
      motor(caveSpeed+difference*100,caveSpeed-20);
      delay(100);
      Serial.print("Dane");      
      
    } 
    
    else if(difference < -2)
    {
      motor(caveSpeed,caveSpeed-20-difference*100);
      delay(100);
      Serial.print("Bame");   
    }

    else if(difference > 80 )
    {
      do
      {
      motor(maxSpeed, maxSpeed);
      sensorMapping();
      }while(mappedValue == 100);





      
    }
    else 
    {
      motor(caveSpeed,caveSpeed-20);  //The minus because bot bends left
      delay(100);  
      Serial.print("Shoja");
    }
    sensorMapping();
  } */

  do
  {
    sensorMapping();
    motor(maxSpeed, maxSpeed - 20);
  } while(mappedValue == 100);

  
  pid();
  motor(leftSpeed,rightSpeed);
  sonarFinishKey=1;
}

void delta_caveCode()
{
  sensorMapping();
  Serial.print("Difference: ");
  Serial.print(difference);
  Serial.println();
  
  while(mappedValue==100)
  {
    triggerSonars();
    delay(10);
    if(difference>5 )
    {
      motor(caveSpeed+difference*7,caveSpeed-40);
      delay(20);
      Serial.print("Dane");      
      
    } else 
    {
      motor(caveSpeed,caveSpeed-40);  //The minus because bot bends left
      delay(20);  
      Serial.print("Shoja");
    }
    sensorMapping();
  } 
  pid();
  motor(leftSpeed,rightSpeed);
}



 //auto calibration
void sensorMapping()
{
int sum=0,pidDetect=0, stopDetect=0; 
 
    
 for (int i = 0; i <sensorNum; i++)
  { 
    
    if (analogRead(A0+i) < blackLimit[i])           
     { 

      if(i>1 && i<6) {sum += i*2; pidDetect++;}
      stopDetect++;
      digitalReading[i]= 1;
    } else digitalReading[i]= 0;
    }


    
   if(pidDetect!=0){  
  mappedValue = sum / pidDetect;
   }
   else mappedValue=100;



if (digitalReading[0]==1 || digitalReading[7]==1) 
{leftIR=digitalReading[0]; rightIR=digitalReading[7];resetIRCounter=0;}

if(digitalReading[0]==0 && digitalReading[sensorNum-1]==0)
{
 resetIRCounter++;
 if(resetIRCounter>contiWhite)
 {
 leftIR=0;
 rightIR=0;
 }
}

if(pidDetect>=6) 
{pidAllBlack=1;}else pidAllBlack=0; 

if(stopDetect==sensorNum)
{
stopAllBlack=1;
stopCounter++;

} else {stopAllBlack=0; stopCounter = 0;}

}


 
void calibration()
{
  plannedCRotate();
  float upSum = 0,lowSum = 0;
  int sensorArray[sensorNum][2];

  for(int i = 0; i < sensorNum; i++)
    {
      sensorArray[i][0] = analogRead(A0+i);
      sensorArray[i][1] = analogRead(A0+i);
    }
 

  int loopCounter = (int)(time * 1000 / 2.5);  
  while(loopCounter)
  {
    for(int i = 0; i < sensorNum; i++)
    {
      if(analogRead(A0+i)<sensorArray[i][0]) sensorArray[i][0]=analogRead(A0+i);
      if(analogRead(A0+i)>sensorArray[i][1]) sensorArray[i][1]=analogRead(A0+i);
    }
  loopCounter--;

  }

 for(int i=0; i < sensorNum; i++)
  blackLimit[i] = (int)(sensorArray[i][0] + safety * (sensorArray[i][1] - sensorArray[i][0]));
prev = millis();
sensorMapping ();
curr= millis();
diff = curr - prev;

  brake();
  

}

