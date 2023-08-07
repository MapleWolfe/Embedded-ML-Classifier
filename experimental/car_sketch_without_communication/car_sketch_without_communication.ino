#define MAX_MOTOR_SPEED 200

//Right motor
int enableRightMotor=5; 
int rightMotorPin1=2;
int rightMotorPin2=3;

//Left motor
int enableLeftMotor=6;
int leftMotorPin1=4;
int leftMotorPin2=7;


int received_int[] = {0,1,2,3,4,5,6,7,8};
int gesture_label = 9;

void setup(){
  pinMode(enableRightMotor,OUTPUT);
  pinMode(rightMotorPin1,OUTPUT);
  pinMode(rightMotorPin2,OUTPUT);
  
  pinMode(enableLeftMotor,OUTPUT);
  pinMode(leftMotorPin1,OUTPUT);
  pinMode(leftMotorPin2,OUTPUT);    
  Serial.begin(9600);
}

void loop(){
    for (int i = 0; i < 8; i++) { // this is removed for received data 
      gesture_label = received_int[i];
      //We will receive value as 0 to 254. Center value is 127
      if (gesture_label == 0){    //right-L
        right();
        forward();
        left();
        forward();
        stop();
        Serial.print(gesture_label);
      }

      else if (gesture_label == 1){    //left_l
        left();
        forward();
        right();
        forward();
        stop();
        Serial.print(gesture_label);

      }

      else if (gesture_label == 2){    //square
        right();
        forward();
        right();
        forward();
        right();
        forward();
        right();
        forward();
        stop();
        Serial.print(gesture_label);

      }

      else if (gesture_label == 3){    //reverse_square
        left();
        forward();
        left();
        forward();
        left();
        forward();
        left();
        forward();
        stop();
        Serial.print(gesture_label);

      }

      else if (gesture_label == 4){    //forward
        forward();
        stop();
        Serial.print(gesture_label);

      }
      else if (gesture_label == 5){    //right
        right();
        stop();
        Serial.print(gesture_label);

      }
      else if (gesture_label == 6){    //left
        left();
        stop();
        Serial.print(gesture_label);

      }
      else if (gesture_label == 7){    //reverse
        back();
        stop();
        Serial.print(gesture_label);

      }
      else if (gesture_label == 8){    //wave
        left();
        forward();
        right();
        forward();
        left();
        forward();
        right();
        forward();
        stop();
        Serial.print(gesture_label);

        }
    }
  } 

void rotateMotor(int rightMotorSpeed, int leftMotorSpeed){
  if (rightMotorSpeed < 0)
  {
    digitalWrite(rightMotorPin1,LOW);
    digitalWrite(rightMotorPin2,HIGH);    
  }
  else if (rightMotorSpeed > 0)
  {
    digitalWrite(rightMotorPin1,HIGH);
    digitalWrite(rightMotorPin2,LOW);      
  }
  else
  {
    digitalWrite(rightMotorPin1,LOW);
    digitalWrite(rightMotorPin2,LOW);      
  }
  
  if (leftMotorSpeed < 0)
  {
    digitalWrite(leftMotorPin1,LOW);
    digitalWrite(leftMotorPin2,HIGH);    
  }
  else if (leftMotorSpeed > 0)
  {
    digitalWrite(leftMotorPin1,HIGH);
    digitalWrite(leftMotorPin2,LOW);      
  }
  else
  {
    digitalWrite(leftMotorPin1,LOW);
    digitalWrite(leftMotorPin2,LOW);      
  }  

  analogWrite(enableRightMotor, abs(rightMotorSpeed));
  analogWrite(enableLeftMotor, abs(leftMotorSpeed));    
}
void forward(){
  rotateMotor(MAX_MOTOR_SPEED, MAX_MOTOR_SPEED); // forward
  delay (1000);
}
void back(){
  rotateMotor(-MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED); // forward
  delay (1000);
}
void right(){
  rotateMotor(-MAX_MOTOR_SPEED, MAX_MOTOR_SPEED); // forward
  delay (500);
}
void left(){
  rotateMotor(MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED); // forward
  delay (500);
}
void stop(){
  rotateMotor(0, 0); // forward
  delay (100);
}