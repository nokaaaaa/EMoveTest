#include <mbed.h>
#include <cmath>
#include <math.h>
#include "pins.hpp"
#include "parameters.hpp"
#include "Encoder/Encoder.hpp"
#include "driveBase/driveBase.hpp"

// モータの設定
DriveMotor motor0(PwmOutPins::MOTOR4_PWM , DigitalOutPins::MOTOR4_DIR, MOTOR_0_KP_1, MOTOR_0_KI_1, MOTOR_0_KD_1, MOTOR_0_KP_2, MOTOR_0_KI_2, MOTOR_0_KD_2, 0);
DriveMotor motor1(PwmOutPins::MOTOR2_PWM, DigitalOutPins::MOTOR2_DIR,  MOTOR_1_KP_1, MOTOR_1_KI_1, MOTOR_1_KD_1, MOTOR_1_KP_2, MOTOR_1_KI_2, MOTOR_1_KD_2,0);
DriveMotor motor2(PwmOutPins::MOTOR1_PWM, DigitalOutPins::MOTOR1_DIR, MOTOR_2_KP_1, MOTOR_2_KI_1, MOTOR_2_KD_1, MOTOR_2_KP_2, MOTOR_2_KI_2, MOTOR_2_KD_2,0);
DriveMotor motor3(PwmOutPins::MOTOR3_PWM, DigitalOutPins::MOTOR3_DIR, MOTOR_3_KP_1, MOTOR_3_KI_1, MOTOR_3_KD_1, MOTOR_3_KP_2, MOTOR_3_KI_2, MOTOR_3_KD_2,0);

//アクチュエータの設定
DriveMotor act1(PwmOutPins::MOTOR5_PWM, DigitalOutPins::MOTOR5_DIR,0,0,0,0,0,0,1);//トッピング回収
DriveMotor act2(PwmOutPins::MOTOR6_PWM, DigitalOutPins::MOTOR6_DIR,0,0,0,0,0,0,1);//昇降機


// encoderの設定(encoder1が横向き、encoder2が縦向き)
Encoder encoder1(InterruptInPins::MEASURING_ENCODER1_A, DigitalInPins::MEASURING_ENCODER1_B, ENC_RES_MAX, 0, false);
Encoder encoder2(InterruptInPins::MEASURING_ENCODER2_A, DigitalInPins::MEASURING_ENCODER2_B, ENC_RES_MAX, 0, false);


//重要なメモ R2の向きが0どの間(2つめのトッピングをとるまで)はカルマンフィルタでposをとり、最後はロータリのみ
//重要なメモ2 カルマンフィルタによって得られるPos自体はOdometry classで20Hz(?)で計算する(Tickerの無駄を減らせる)
//重要なメモ3 最初のR2の向きが0度、反時計回りが正の角度（構造体のみ)
DriveBase driveBase(&encoder1, &encoder2,&motor0, &motor1, &motor2, &motor3);

Localization loc(&encoder1, &encoder2);

Timer timer;

bool flag=false;

string str;

float acts=0.7;//act1の手前から奥もしくは奥から手前にかかる時間
float actpwm=-0.5;//act1のpwm

float delay=0.5;//act1,足回りの連続動作を禁止

float act2pwm =-0.8;//act2のpwm

float ds=1.0;//走行中の速さ

float ts=0.5;//奥のトッピングを取りに行くスピード

float senkai_time=3.5;//π/2旋回の時間

void drive(){
  wait_us(3*1000000);

   act1.setPWM(-actpwm);//最初は回収機構が手前にあると仮定
   wait_us(1000000*acts);

   act1.setPWM(0);//止める

   driveBase.goTo(0, 1400, 0,true,true);//手前のトッピングまで進む
   act1.setPWM(actpwm);//回収
   wait_us(1000000*(acts+0.1));

   act1.setPWM(0);//急にpwm変化するとだめなので静止遅延入れる
   wait_us(1000000*delay);

   act1.setPWM(-actpwm);//回収機構を前に出す
   wait_us(1000000*acts);

   act1.setPWM(0);//固定
   driveBase.goTo(0, 1800, 0,true,true);//奥のトッピングに行く
   act1.setPWM(actpwm);//回収
   wait_us(1000000*acts);

   act1.setPWM(0);//急にpwm変化するとだめなので静止遅延入れる
   wait_us(1000000*delay);

   act1.setPWM(-actpwm);//回収機構を前に出す
   wait_us(1000000*acts);
   act1.setPWM(0);
   driveBase.goTo(0, 1100, 0,true,true);//後ろにさがる
   driveBase.runNoEncoder(0,0,0,0,delay);//曲がり角のディレイ
   driveBase.goTo(-600, 1200, 0,true,true);//左に行く
   driveBase.runNoEncoder(0,0,0,0,delay);//曲がり角のディレイ
   driveBase.goTo(-700, 1400, 0,true,true);//前に行く
   act1.setPWM(actpwm);//回収
   wait_us(1000000*(acts+0.1));
   act1.setPWM(0);//急にpwm変化するとだめなので静止遅延入れる
   wait_us(1000000*delay);
   act1.setPWM(-actpwm);//前に出す
   wait_us(1000000*acts);
   act1.setPWM(0);//前に出す状態を固定
   driveBase.goTo(-700, 1700, 0,true,true);//前に行く
   act1.setPWM(actpwm);//2こめ回収
   wait_us(1000000*(acts+0.1));
   act1.setPWM(0);//奥に収納
   driveBase.goTo(-700, 900, 0,true,true);//後ろに行く
   driveBase.runNoEncoder(0,0,0,0,delay);//曲がり角のディレイ
   driveBase.goTo(-1400, 900, 0,true,true);//左に行く
   driveBase.runNoEncoder(0,0,0,0,delay);//皿到着と回転のディレイ
   driveBase.senkai(0.5,senkai_time);//右にπ/2旋回
   driveBase.runNoEncoder(0,0,0,0,delay);
   driveBase.goTo(-1400, 800, 0,true,true);//少し後ろにさがる(内部では(-2000,1000)にいると思っているため)
   act2.setPWM(act2pwm);//トッピング
   wait_us((1.5)*1000000);
   act2.setPWM(0);
   wait_us(7*1000000);
   driveBase.runNoEncoder(0,ds,0,0,2);//前に進む
   driveBase.runNoEncoder(0,0,0,0,10000);//待機
   

    while(1){
        
     
    }
}

void drivesim()
{
  wait_us(3*1000000);

   act1.setPWM(-actpwm);//最初は回収機構が手前にあると仮定
   wait_us(1000000*acts);

   act1.setPWM(0);//止める

   driveBase.goTo(0, 1650, 0,true,true);//手前のトッピングまで進む
   wait_us(1000000*delay);
   driveBase.goTo(0, 0, 0,true,true);

}

//やってみること idleをflaseにしてみる
//driveBaseクラスのgoTowardTargetAccDccの最後のif文内でlocalizationをリセットしてみる
//加速度制限変えてみる
//PIDの値変えてみる
//直接PWMを書き込んでいるためリアルタイムで書き込む値を表示して安全に動かす
int main()
{
  drive();
}