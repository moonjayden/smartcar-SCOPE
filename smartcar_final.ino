#include <Servo.h>
Servo servo;

const int SERVO1_PIN = 9;      // 서보모터1 연결핀
const int IR_R = A3;  //  적외선센서 우측 핀
const int IR_L = A4;  // 적외선센서 좌측 핀

const int M1_PWM = 5;   // DC모터1 PWM 핀 왼
const int M1_DIR1 = 7;   // DC모터1 DIR1 핀
const int M1_DIR2 = 8;   // DC모터 1 DIR2 핀

const int M2_PWM = 6;   // DC모터2 PWM 핀
const int M2_DIR1 = 11;   // DC모터2 DIR1 핀
const int M2_DIR2 = 12;   // DC모터2 DIR2 핀

const int FC_TRIG  = 13;   // 전방 초음파 센서 TRIG 핀
const int FC_ECHO = 10;  // 전방 초음파 센서 ECHO 핀
const int L_TRIG = A2;  // 좌측 초음파 센서 TRIG 핀
const int L_ECHO = 3;  // 좌측 초음파 센서 ECHO 핀
const int R_TRIG = 2;   // 우측 초음파 센서 TRIG 핀
const int R_ECHO = A5;  // 우측 초음파 센서 ECHO 핀

const int MAX_DISTANCE = 2000; // 초음파 센서의 최대 감지거리


float center;
float left;
float right;


int state = 1;
int p_state = 0;
int t_state = 0;
int course_num = -1;

// 자동차 튜닝 파라미터 =====================================================================
int detect_ir = 23; // 검출선이 흰색과 검정색 비교


int punch_pwm = 200;   // 정지 마찰력 극복 출력 (0 ~ 255) 조정해야하는 값
int punch_time = 50; // 정지 마찰력 극복 시간 (단위 msec)
int stop_time = 300; // 전진후진 전환 시간 (단위 msec)

int max_ai_pwm = 110; // 자율주행 모터 최대 출력 (0 ~ 255)
int min_ai_pwm = 70; //자율주행 모터 최소 출력 (0 ~ 255)

int angle_offset = 0; // 서보 모터 중앙각 오프셋 (단위: 도)
int angle_limit = 55; // 서보 모터 회전 제한 각 (단위: 도)

int center_detect = 200; // 전방 감지 거리 (단위: mm)
int center_start = 160; // 전방 출발 거리 (단위: mm)
int center_stop = 70; // 전방 멈춤 거리 (단위: mm)

int side_detect = 125; // 좌우 감지 거리 (단위: mm)


float cur_steering;
float cur_speed;
float compute_steering;
float compute_speed;

float max_pwm;
float min_pwm;

// 초음파 거리측정
float GetDistance(int trig, int echo)
{
  digitalWrite(trig, LOW);
  delayMicroseconds(4);
  digitalWrite(trig, HIGH);
  delayMicroseconds(20);
  digitalWrite(trig, LOW);

  unsigned long duration = pulseIn(echo, HIGH, 5000);
  if (duration == 0)
    return MAX_DISTANCE;
  else
    return duration * 0.17;     // 음속 340m/s
}

int ir_sensing(int pin) {
  return analogRead(pin);
}

// 앞바퀴 조향
void SetSteering(float steering)
{
  cur_steering = constrain(steering, -1, 1);// constrain -1~ 1 값으로 제한

  float angle = cur_steering * angle_limit;
  int servoAngle = angle + 90;
  servoAngle += angle_offset;

  servoAngle = constrain(servoAngle, 0, 180);
  servo.write(servoAngle);
}


// 뒷바퀴 모터회전
void SetSpeed(float speed)
{
  speed = constrain(speed, -1, 1);

  if ((cur_speed * speed < 0) // 움직이는 중 반대 방향 명령이거나
      || (cur_speed != 0 && speed == 0)) // 움직이다가 정지라면
  {
    cur_speed = 0;
    digitalWrite(M1_PWM, HIGH);
    digitalWrite(M1_DIR1, LOW);
    digitalWrite(M1_DIR2, LOW);

    digitalWrite(M2_PWM, HIGH);
    digitalWrite(M2_DIR1, LOW);
    digitalWrite(M2_DIR2, LOW);

    if (stop_time > 0)
      delay(stop_time);
  }

  if (cur_speed == 0 && speed != 0) // 정지상태에서 출발이라면
  {
    if (punch_time > 0)
    {
      if (speed > 0)
      {
        analogWrite(M1_PWM, punch_pwm);
        digitalWrite(M1_DIR1, HIGH);
        digitalWrite(M1_DIR2, LOW);

        analogWrite(M2_PWM, punch_pwm);
        digitalWrite(M2_DIR1, HIGH);
        digitalWrite(M2_DIR2, LOW);
      }
      else if (speed < 0)
      {
        analogWrite(M1_PWM, punch_pwm);
        digitalWrite(M1_DIR1, LOW);
        digitalWrite(M1_DIR2, HIGH);

        analogWrite(M2_PWM, punch_pwm);
        digitalWrite(M2_DIR1, LOW);
        digitalWrite(M2_DIR2, HIGH);
      }
      delay(punch_time);
    }
  }

  if (speed != 0) // 명령이 정지가 아니라면
  {
    int pwm = abs(speed) * (max_pwm - min_pwm) + min_pwm;           // 0 ~ 255로 변환

    if (speed  > 0)
    {
      analogWrite(M1_PWM, pwm);
      digitalWrite(M1_DIR1, HIGH);
      digitalWrite(M1_DIR2, LOW);

      analogWrite(M2_PWM, pwm);
      digitalWrite(M2_DIR1, HIGH);
      digitalWrite(M2_DIR2, LOW);
    }
    else if (speed < 0)
    {
      analogWrite(M1_PWM, pwm);
      digitalWrite(M1_DIR1, LOW);
      digitalWrite(M1_DIR2, HIGH);

      analogWrite(M2_PWM, pwm);
      digitalWrite(M2_DIR1, LOW);
      digitalWrite(M2_DIR2, HIGH);
    }
  }
  cur_speed = speed;
}

void driving() {
  compute_steering = cur_steering;
  compute_speed = cur_speed;
  
  center = GetDistance(FC_TRIG, FC_ECHO);
  left = GetDistance(L_TRIG, L_ECHO);
  right = GetDistance(R_TRIG, R_ECHO);
  stop_line(); // 정지선일 때 검출 <- 상황에 따라 course_num을 정해줌

  course(); // 코스
  if(course_num==-2 && center<=center_stop){ // 시작할때 정지
    SetSpeed(0);
    SetSteering(0);
    delay(500);
    compute_speed=0;
    compute_steering=0;
    
  }
  if(center<=center_stop&&right<=150&&left<=150){ //종료
    SetSpeed(0);
    SetSteering(0);
    delay(500);
    compute_speed=0;
    compute_steering=0;
    
    
  }
  SetSpeed(compute_speed);
  SetSteering(compute_steering);
}

void straight(float speed) { // 기본주행
  if (ir_sensing(IR_R) >= detect_ir && ir_sensing(IR_L) >= detect_ir ) {  //차선이 검출되지 않을 경우 직진
    compute_steering = 0;
    compute_speed = speed;
  }

  else if (ir_sensing(IR_R) <= detect_ir) { // 오른쪽 차선이 검출된 경우
    compute_steering = -1;
    compute_speed = 0.1;
  }

  else if (ir_sensing(IR_L) <= detect_ir) { //왼쪽 차선이 검출된 경우
    compute_steering = 1;
    compute_speed = 0.1;
  }
}
void course(){
  if(course_num==0){ //평행주차 in
    straight(0.1); //벽 검출하지 않는다면 계속 정상주행
    if(left<=side_detect && right<=side_detect){ //벽 검출되면 평행주차 시작
      course_num=1;
      p_state=1;
    }
  }
  else if (course_num == 1) { //평행주차
    p_parking();
  }
  else if (course_num == 2){ //T자 주차 정렬 완료 or 회피주행 시작
    avoid();
  }
  else if (course_num == 3){ //T자 진입
    t_parking();
  }
  else {
    straight(0.1);
  }
}
void stop_line(){
  if (ir_sensing(IR_R) <= detect_ir && ir_sensing(IR_L) <= detect_ir) { //양쪽 차선이 검출된 경우
    if (p_state == 0){
      SetSteering(0);
      SetSpeed(0);  //일단 멈춤
      delay(1500);
      compute_steering=0;
      compute_speed=0;
      
      if(center == MAX_DISTANCE && left >= side_detect && right >= side_detect){
       course_num=0; //정지선을 검출하고 다 뚫려있는 상태에서 전진(평행주차 course_num=1 or 교차로 course_num=0) 1번 2번 상황
      }
      else if(left <= side_detect && right <= side_detect){ //옆 다 막혀있는 상태 회피주행 시작) 4 6
          p_state=1;
          course_num=2;
        }
      
      else if(center != MAX_DISTANCE && left >= MAX_DISTANCE && (right >= MAX_DISTANCE)){ //앞만 막히고 양 옆은 막히지 않음 (T자 주차 진입) 3
        
        p_state=1;
        course_num=3;
      }
      else{ //기본주행
        course_num = -1;
      }
      Serial.println("!!!!");
      
      SetSteering(0);
      SetSpeed(0.1);
      compute_steering=0;
      compute_speed=0.1;
      delay(300);
    }
  }

}


void p_parking(){//평행주차
  if(p_state==1){    //평행주차 코스 진입 (우회전)
    straight(0.1);
    if(right>=120){ 
      compute_steering=1;
      compute_speed=0.1;
    }
    if(center<=center_stop){ //평행주차 코스 진입 완료
      p_state=2;
    }


  }
  else if(p_state==2){ //후진
    compute_steering=0;
    compute_speed=-0.1;
    if(right==MAX_DISTANCE){ //후진 완료
      p_state=3;
    }
  }
  else if(p_state==3){ //평행주차 자세 만들기 위한 좌회전
    compute_steering=-1;
    compute_speed=0.1;
    if(center<=center_stop){ //궁극적인 목표
      p_state=4;
    }
    else if(ir_sensing(IR_R)<=detect_ir){ //그 목표 전에 차선 감지한다면 후진 후 좌회전 반복
        SetSteering(0);
        SetSpeed(-0.1);
        delay(500);
        compute_steering=0;
        compute_speed=-0.1;
    }
  }

  else if(p_state==4){ //주차 
    compute_steering=1;
    compute_speed=-0.1;
    if(center>=MAX_DISTANCE){
      p_state=5;
    }
  }
  else if(p_state==5){ //주차 마무리
    compute_steering=-1;
    compute_speed=-0.1;
    if(right<=120){  //궁극적인 주차 목표        
      compute_steering=0;      
      if(center>=290){
        compute_speed=0;
        p_state=6;
      }
    }
  }

  
  else if(p_state==6){ //주차 완료 정지 후 탈출(좌회전)
    delay(1500);
    p_state=7;

  }
  else if(p_state==7){
    compute_steering=-1;
    compute_speed=0.1;
    if(center<=center_stop){ //좌회전 후 벽 만남, 좌회전 정지
      p_state=8;
    }
  }
  else if(p_state==8){ //후진
    compute_steering=0;
    compute_speed=-0.1;
    if(center>=200){//후진 완료
      p_state=9;
    }
  }
  else if(p_state==9){//우회전
    compute_steering=0.7;
    compute_speed=0.1;
    if(left<=125 && right<=125){
      p_state=0;
      course_num=-1;
    }
    else if(ir_sensing(IR_R)<=detect_ir){
      SetSteering(1);
      SetSpeed(-0.1);
      delay(500);

      compute_steering=1;
      compute_speed=-0.1;
    }  
    else if(ir_sensing(IR_L)<=detect_ir){
      SetSteering(0);
      SetSpeed(-0.1);
      delay(500);
    }
    
  }
  else if( p_state==10){
    compute_steering=0;
    compute_speed=0;
  }

}
 

void t_parking(){ //t자 주차
  if(p_state==1){
    compute_steering=0;
    compute_speed=0.1;
    if(center<=center_start){
      p_state=2;
    }
  }
  else if(p_state==2){
    compute_steering=-0.5; //기본적으로 좌회전 후 후진을 반복(자세 만들때 까지)
    compute_speed=0.1;
    if(ir_sensing(IR_L)<=detect_ir && ir_sensing(IR_R)<=detect_ir){ //정렬 완료 후 후진(정지선 중복 검출 방지)
      SetSteering(0.3);
      SetSpeed(-0.1);
      compute_steering=0;
      compute_speed=-0.1;
      delay(500);
      
      p_state=3;
    }
    else if(ir_sensing(IR_L)<=detect_ir){
      SetSteering(0.3);
      SetSpeed(-0.1);
      compute_steering=0;
      compute_speed=-0.1;
      delay(500);
      p_state=3;
    }


    else if(ir_sensing(IR_R)<=detect_ir){
      if(left+right<=180){
      SetSteering(0);
      SetSpeed(-0.1);
      compute_steering=0;
      compute_speed=-0.1;
      delay(500);
      p_state=3;
        
      }
      SetSteering(0);
      SetSpeed(-0.1);
      delay(300);
    }



  }
  else if(p_state==3){ //후진
    compute_steering=0;
    compute_speed = -0.1;
    if(ir_sensing(IR_R)<=detect_ir && ir_sensing(IR_L)<=detect_ir){
      SetSteering(0);
      SetSpeed(0);
      delay(1500);
      SetSteering(0);
      SetSpeed(0.1);
      delay(500);
      compute_speed=0.1;
      compute_steering=0;
      course_num=-1;
      p_state=0;
    }
  }

}

void avoid(){ //회피주행
  if(p_state==1){ //처음 진입은 기본 주행으로
    straight(0.1);
    if(center<=center_stop){ //장애물 만남
      p_state=2;
    }
  }
  else if(p_state==2){ //후진
    compute_steering=0;
    compute_speed=-0.1;
    if(left<=side_detect && right<=side_detect){ //후진 완료
      SetSteering(0);
      SetSpeed(-0.1);
      delay(300);
      p_state=3;
    }
  }
  else if(p_state==3){ //장애물 피하기 위한 좌회전
    compute_steering=-1;
    compute_speed=0.1;
    if(ir_sensing(IR_L)<=detect_ir){ //좌회전 완료
      SetSteering(0);
      SetSpeed(-0.1);
      delay(500);
      compute_steering=0;
      compute_speed=-0.1;
      p_state=5;
    }
  }
  else if(p_state==4){ //90도 우회전을 위한 후진
    compute_steering=0;
    compute_speed=-0.1;
    if(right<=250){ //후진 완료
      p_state=5;
    }
  }
  
  
  else if(p_state==5){ //90도 코스를 위한 코딩 (90도를 회전하기 위해서는 후진이 필수)
    if(ir_sensing(IR_L)<=detect_ir){ //오른쪽 커브
      SetSteering(0);
      SetSpeed(-0.1);
      delay(500);
      SetSteering(0.5);
      SetSpeed(0.1);
      delay(500);
      compute_speed=0.1;
      compute_steering=0.5;
    }
    else if(ir_sensing(IR_R)<=detect_ir){ //왼쪽 커브
      SetSteering(0);
      SetSpeed(-0.1);
      delay(500);
      SetSteering(-0.5);
      SetSpeed(0.1);
      delay(500);
      compute_speed=0.1;
      compute_steering=-0.5;
    }
    else{ //기본적으로는 기본 주행
      straight(0.1);
    }
    
  }


  
}



void setup() {

  Serial.begin(115200);
  servo.attach(SERVO1_PIN); //서보모터 초기화

  pinMode(IR_R, INPUT);
  pinMode(IR_L, INPUT);

  pinMode(FC_TRIG, OUTPUT);
  pinMode(FC_ECHO, INPUT);

  pinMode(L_TRIG, OUTPUT);
  pinMode(L_ECHO, INPUT);

  pinMode(R_TRIG, OUTPUT);
  pinMode(R_ECHO, INPUT);

  pinMode(M1_PWM, OUTPUT);
  pinMode(M1_DIR1, OUTPUT);
  pinMode(M1_DIR2, OUTPUT);
  pinMode(M2_PWM, OUTPUT);
  pinMode(M2_DIR1, OUTPUT);
  pinMode(M2_DIR2, OUTPUT);

  max_pwm = max_ai_pwm;
  min_pwm = min_ai_pwm;
  p_state=0;
  t_state=0;  
  course_num= -2;
  SetSteering(0);
  SetSpeed(0);

}

void loop() {



  driving();
  
  Serial.println(p_state);
  Serial.println(course_num);
  Serial.println(right);
  Serial.println(left);
  Serial.println(center);
  Serial.println(ir_sensing(IR_R));
  Serial.println(ir_sensing(IR_L));


}
