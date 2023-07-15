#include <Servo.h>
Servo servo;

const int SERVO_PIN = 9;      // 서보모터1 연결핀
const int M1_PWM = 5;   // DC모터1 PWM 핀 
const int M1_DIR1 = 7;   // DC모터1 DIR1 핀
const int M1_DIR2 = 8;   // DC모터 1 DIR2 핀

const int M2_PWM = 6;   // DC모터2 PWM 핀 
const int M2_DIR1 = 11;   // DC모터2 DIR1 핀 
const int M2_DIR2 = 12;   // DC모터2 DIR2 핀

void setup() {
    pinMode(M1_PWM, OUTPUT);
    pinMode(M1_DIR1, OUTPUT);
    pinMode(M1_DIR2, OUTPUT);

    pinMode(M2_PWM, OUTPUT);
    pinMode(M2_DIR1, OUTPUT);
    pinMode(M2_DIR2, OUTPUT);
    
    //서보모터 초기화
    servo.attach(SERVO_PIN); 
    servo.write(0);
    delay(500);
  
}
void loop() {
    // 정회전
    digitalWrite(M1_DIR1, HIGH);
    digitalWrite(M1_DIR2, LOW);
    analogWrite(M1_PWM, 255);   // 0 ~ 255의 PWM값으로 속도 조절

    digitalWrite(M2_DIR1, HIGH);
    digitalWrite(M2_DIR2, LOW);
    analogWrite(M2_PWM, 255);   // 0 ~ 255의 PWM값으로 속도 조절
    delay(500);
    servo.write(180);
    delay(500);
    digitalWrite(M1_DIR1, HIGH);
    digitalWrite(M1_DIR2, LOW);
    analogWrite(M1_PWM, 255);   // 0 ~ 255의 PWM값으로 속도 조절

    digitalWrite(M2_DIR1, HIGH);
    digitalWrite(M2_DIR2, LOW);
    analogWrite(M2_PWM, 255);   // 0 ~ 255의 PWM값으로 속도 조절
    delay(500);
    servo.write(90);       // 오른쪽 이동(정상동작 확인용 코드)
    delay(1000);
    servo.detach();
    // 정지
    //digitalWrite(M1_DIR1, LOW);
    //digitalWrite(M1_DIR2, LOW);
    //analogWrite(M1_PWM, 0);
    //digitalWrite(M2_DIR1, LOW);
    //digitalWrite(M2_DIR2, LOW);
    //analogWrite(M2_PWM, 0);


    // 역회전
    //digitalWrite(M1_DIR1, LOW);
    //digitalWrite(M1_DIR2, HIGH);
    //analogWrite(M1_PWM, 255);   // 0 ~ 255의 PWM값으로 속도 조절
    //digitalWrite(M2_DIR1, LOW);
    //digitalWrite(M2_DIR2, HIGH);
    //analogWrite(M2_PWM, 255);   // 0 ~ 255의 PWM값으로 속도 조절

}
