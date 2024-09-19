#include <mbed.h>
#include <math.h>
#include <chrono>
#include "driveBase.hpp"
#include "parameters.hpp"
#include "simpleFunctions/simpleFunctions.hpp"


inline bool limit_filter(float* value, float minValue, float maxValue){
    if(*value < minValue){
        *value = minValue;
        return 1;
    }else if(*value > maxValue){
        *value = maxValue;
        return 1;
    }else{
        return 0;
    }
}

//初期化ここでlocalizationの初期化も行う!!!すごい重要
DriveBase::DriveBase(Encoder* encoder1,Encoder* encoder2 ,DriveMotor* motor_0, DriveMotor* motor_1, DriveMotor* motor_2, DriveMotor* motor_3, float kp_1, float ki_1, float kd_1, float kp_2, float ki_2, float kd_2):localization(encoder1,encoder2), pidController(SPEED_ADJUSTMENT_FREQUENCY, kp_1, ki_1, kd_1), pidRotateController(SPEED_ADJUSTMENT_FREQUENCY, kp_2, ki_2, kd_2) 
{
    motors[0] = motor_0;
    motors[1] = motor_1;
    motors[2] = motor_2;
    motors[3] = motor_3;

    moving = false;
    loop = [this] {return;};
}

void DriveBase::resetAlld()
{
    resetPID();
    timer.reset();
}

void DriveBase::resetPID(){
    for (int i=0;i<4;i++){
        motors[i]->pidController.reset();
    }
    pidController.reset();
    pidRotateController.reset();
}


#if 1
//速度を指定して移動
void DriveBase::go(float targetSpeedX, float targetSpeedY, float targetSpeedD, bool absolute){

    float targetSpeedR = sqrtf(targetSpeedX*targetSpeedX + targetSpeedY*targetSpeedY);

    //速度を制限する
    if(targetSpeedR > MAX_SPEED){
        targetSpeedX = MAX_SPEED*(targetSpeedX/targetSpeedR);
        targetSpeedY = MAX_SPEED*(targetSpeedY/targetSpeedR);
        pidController.reset();
    }


    float targetAccX = (targetSpeedX - lastTargetSpeedX) * SPEED_ADJUSTMENT_FREQUENCY;
    float targetAccY = (targetSpeedY - lastTargetSpeedY) * SPEED_ADJUSTMENT_FREQUENCY;
    float targetAccD = (targetSpeedD - lastTargetSpeedD) * SPEED_ADJUSTMENT_FREQUENCY;

    float targetAccR = sqrtf(targetAccX*targetAccX + targetAccY*targetAccY);

    //加速度を制限する
    if(targetAccR > MAX_ACCELERATION){
        targetAccX = MAX_ACCELERATION*(targetAccX/targetAccR);
        targetAccY = MAX_ACCELERATION*(targetAccY/targetAccR);
        targetSpeedX = localization.speedX + targetAccX/SPEED_ADJUSTMENT_FREQUENCY;
        targetSpeedY = localization.speedY + targetAccY/SPEED_ADJUSTMENT_FREQUENCY;
    }
    

    
    //デバッグ用
    _s1 = int(targetSpeedX);
    _s2 = int(targetSpeedY);

    lastTargetSpeedX = targetSpeedX;
    lastTargetSpeedY = targetSpeedY;
    lastTargetSpeedD = targetSpeedD;

    float vx, vy;
    vx = targetSpeedX;
    vy = targetSpeedY;
    

    //各モーターの速度
    float speeds[4]; //モーターの速度[mm/s]
    speeds[0] = SQRT2/2 * (- vx + vy) ;
    speeds[1] = SQRT2/2 * (- vx - vy) ;
    speeds[2] = SQRT2/2 * (+ vx - vy) ;
    speeds[3] = SQRT2/2 * (+ vx + vy) ;
    
    /*
    for (int i=0;i<4;i++){
        motors[i]->rotate(speeds[i],localization.motorSpeed[i]);
    }
    */
   //各モーターにpidせず直接渡す
   for (int i=0;i<4;i++){
        motors[i]->setPWM(0.005*speeds[i]);//このままだと大きすぎるかも
    }

}

#else
//成分ごとに制限をかける場合．おそらく使用しない．（計算量は減るが方向によって最高速度が違うのは不自然なので)
//速 度を指定して移動
void DriveBase::go(float targetSpeedX, float targetSpeedY, float targetSpeedD){
    if(limit_filter(&targetSpeedX, -MAX_SPEED, +MAX_SPEED)){
        pidController.reset();
    }
    if(limit_filter(&targetSpeedY, -MAX_SPEED, +MAX_SPEED)){
        pidController.reset();
    }
    if(limit_filter(&targetSpeedD, -MAX_ROTATE_SPEED, +MAX_ROTATE_SPEED)){
        pidRotateController.reset();
    }

    limit_filter(&targetSpeedX, lastTargetSpeedX-MAX_ACCELERATION/SPEED_ADJUSTMENT_FREQUENCY, lastTargetSpeedX+MAX_ACCELERATION/SPEED_ADJUSTMENT_FREQUENCY);
    limit_filter(&targetSpeedY, lastTargetSpeedY-MAX_ACCELERATION/SPEED_ADJUSTMENT_FREQUENCY, lastTargetSpeedY+MAX_ACCELERATION/SPEED_ADJUSTMENT_FREQUENCY);
    limit_filter(&targetSpeedD, lastTargetSpeedD-MAX_ROTATE_ACCELERATION/SPEED_ADJUSTMENT_FREQUENCY, lastTargetSpeedD+MAX_ROTATE_ACCELERATION/SPEED_ADJUSTMENT_FREQUENCY);

    _s1 = int(targetSpeedX);
    _s2 = int(targetSpeedY);

    lastTargetSpeedX = targetSpeedX;
    lastTargetSpeedY = targetSpeedY;
    lastTargetSpeedD = targetSpeedD;

    //X, Yに回転行列をかける
    float vx = cos(localization.direction)*targetSpeedX + sin(localization.direction)*targetSpeedY;
    float vy = -sin(localization.direction)*targetSpeedX + cos(localization.direction)*targetSpeedY;

    //各モーターの速度
    float speeds[4]; //モーターの速度
    speeds[0] = SQRT2/2 * (- vx + vy) + TRED_RADIUS * targetSpeedD;
    speeds[1] = SQRT2/2 * (- vx - vy) + TRED_RADIUS * targetSpeedD;
    speeds[2] = SQRT2/2 * (+ vx - vy) + TRED_RADIUS * targetSpeedD;
    speeds[3] = SQRT2/2 * (+ vx + vy) + TRED_RADIUS * targetSpeedD;
    
    
    for (int i=0;i<4;i++){
        motors[i]->rotate(speeds[i]);
    }

}


#endif



//現在決められている目標に向かって進む
//PID制御で目標値に向かうが，速度・加速度に制限を設けることで台形制御を実現する
void DriveBase::goTowardTargetAccDcc(float movement_threshold, float movement_threshold_rad, bool stop){
    float differenceX = target_X-localization.posX;
    float differenceY = target_Y-localization.posY;
    float differenceR = sqrtf(differenceX*differenceX + differenceY*differenceY);
    float differenceD = 0;

    float targetSpeedR = pidController.calculate(differenceR);


    float targetSpeedX, targetSpeedY;

    if(differenceR == 0){
        //0除算の回避
        targetSpeedX = 0;
        targetSpeedY = 0;
    }else{
        targetSpeedX = targetSpeedR*(differenceX/differenceR);
        targetSpeedY = targetSpeedR*(differenceY/differenceR);
    }

    float targetSpeedD = 0;

    //targetSpeedD = 0;

    go(targetSpeedX, targetSpeedY, 0);

    if (differenceR < movement_threshold ) 
    {
        stopMovement(stop);
    }
}


//モーターの停止
void DriveBase::stopMovement(bool stop){
    movementTicker.detach();
    moving = false;
    if(stop){
        for(int i=0;i<4;i++){
            motors[i]->stop();
        }
    }
}


//目標位置に向かって直線移動する stopをflaseにすればとまらずに経路設計できる
void DriveBase::goTo(float X, float Y, float D, bool idle, bool stop){
    if(!moving){
        resetPID();
        moving = true;
    }
        //目標位置の設定
        target_X = X;
        target_Y = Y;
        target_D = D;

        //割り込みの設定
        movementTicker.attach([this] {goTowardTargetAccDcc();}, std::chrono::milliseconds(1000)/SPEED_ADJUSTMENT_FREQUENCY);
    
    //idle=trueなら移動が終わるまで待機
    if(idle){
        while(moving) {
            loop();
            wait_ns(1);
        }
    }
}


//エンコーダーなしでロボットを動かす．主にデバッグ用
void DriveBase::runNoEncoder(float pwmX, float pwmY, float dir, float pwmD, float time){
    if(moving){
        movementTicker.detach();
    }

    float vx = cos(dir)*pwmX + sin(dir)*pwmY;
    float vy = -sin(dir)*pwmX + cos(dir)*pwmY;

    //各モーターの速度
    float speeds[4]; //モーターの速度
    speeds[0] = SQRT2/2 * (- vx + vy) + pwmD;
    speeds[1] = SQRT2/2 * (- vx - vy) + pwmD;
    speeds[2] = SQRT2/2 * (+ vx - vy) + pwmD;
    speeds[3] = SQRT2/2 * (+ vx + vy) + pwmD;

    timer.reset();
    timer.start();

    for(int i=0;i<4;i++){
        motors[i]->setPWM(speeds[i]);
    }
    

    while(chrono::duration<float>(timer.elapsed_time()).count() < time){
        loop();
        wait_ns(1);
    }

    stopMovement();
}

//超信地旋回
void DriveBase::rotateTo(float D, bool idle){
    goTo(localization.posX, localization.posY, D, idle);
}

//平行移動
void DriveBase::goParallelTo(float X, float Y, bool idle){
    goTo(X, Y, localization.direction, idle);
}

//private
void DriveBase::runAlongArch(float radius, float centerX, float centerY, float start_dir, float end_dir, float D, bool stop, int num){
    for(int i=0;i<num+1;i++){
        float X = centerX + radius*sin(start_dir + radiansMod(end_dir-start_dir) * (float(i)/num));
        float Y = centerY - radius*cos(start_dir + radiansMod(end_dir-start_dir) * (float(i)/num));
        
        if(i != 0 || X != localization.posX || Y != localization.posY){
            goTo(X, Y, D, true, i == num && stop);
        }
    }
}

//曲線移動．8つの点に分けて直線移動しているイメージ
void DriveBase::goCurveTo(float start_dir, float end_dir, float X, float Y, float D, bool stop, int num){
    if(radiansMod(start_dir - end_dir) == 0){
        goTo(X, Y, D, true, stop);
        return;
    }

    float deltaX = X - localization.posX;
    float deltaY = Y - localization.posY;

    //カーブの半径
    float radius1 = (+ sin(end_dir) * deltaX - cos(end_dir) * deltaY)/(1-cos(end_dir - start_dir));
    float radius2 = (- sin(start_dir)*deltaX + cos(start_dir)*deltaY)/(1-cos(end_dir - start_dir));

    if(abs(radius1) < abs(radius2)){
        float centerX = localization.posX - radius1*sin(start_dir);
        float centerY = localization.posY + radius1*cos(start_dir);
        runAlongArch(radius1, centerX, centerY, start_dir, end_dir, D, false, num);
        goTo(X, Y, D, true, stop);
    }else{
        float centerX = X - radius2*sin(end_dir);
        float centerY = Y + radius2*cos(end_dir);
        runAlongArch(radius2, centerX, centerY, start_dir, end_dir, D, stop, num);
    }
}

//private
void DriveBase::goPtr(bool absolute){
    go(*targetSpeedXPtr, *targetSpeedYPtr, *targetSpeedDPtr, absolute);
}

//目標速度をポインタとして渡す．コントローラーでの操作で使用．
void DriveBase::goPtrStart(float* targetSpeedX, float* targetSpeedY, float* targetSpeedD, bool absolute, bool idle){
    targetSpeedXPtr = targetSpeedX;
    targetSpeedYPtr = targetSpeedY;
    targetSpeedDPtr = targetSpeedD;
    movementTicker.attach([this, absolute] {goPtr(absolute);}, std::chrono::milliseconds(1000)/SPEED_ADJUSTMENT_FREQUENCY);
    if(idle){
        while(1){
            loop();
            wait_ns(1);
        }
    }
}

//ロボットが動いている間に繰り返し行う動作を，関数渡しにより設定
//analoginやprintfなど，タイマー割り込みで行えないものを書く．
void DriveBase::attachLoop(function<void(void)> loop_func){
    loop = loop_func;
}

void DriveBase::senkai(float pwm2,float time2)
{
  if(moving){
        movementTicker.detach();
    }

    //各モーターの速度
    float speeds[4]; //モーターの速度
    speeds[0] = -pwm2;
    speeds[1] = -pwm2;
    speeds[2] = -pwm2;
    speeds[3] = -pwm2;

    timer.reset();
    timer.start();

    for(int i=0;i<4;i++){
        motors[i]->setPWM(speeds[i]);
    }
    

    while(chrono::duration<float>(timer.elapsed_time()).count() < time2){
        loop();
        wait_ns(1);
    }

    stopMovement();
}