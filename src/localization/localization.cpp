#include <mbed.h>
#include <math.h>
#include "parameters.hpp"
#include "localization.hpp"
//ここでst35をまとめてObservation型の変数を宣言
Localization::Localization(Encoder* encoder1, Encoder* encoder2)
{
    encoders[0] = encoder1;
    encoders[1] = encoder2;
    ticker.attach([this] {Update();}, std::chrono::milliseconds(1000)/ENCODER_LOCALIZATION_FREQUENCY);
    flag = false;
    kalman = true;
}
 
void Localization::Update()
{

    float Moveincrement[2]={0,0};
    
    Moveincrement[0]=(-1)*(encoders[0]->getAmount())-beforeMove[0];//エンコーダ1(x方向)の移動距離
    Moveincrement[1]=encoders[1]->getAmount()-beforeMove[1];//エンコーダ2(y方向)の移動距離
    


    posX=(-1)*(encoders[0]->getAmount());//位置はエンコーダ1の値をそのまま使う
    posY=encoders[1]->getAmount();//位置はエンコーダ2の値をそのまま使う
   

    speedX=Moveincrement[0]*ENCODER_LOCALIZATION_FREQUENCY;
    speedY=Moveincrement[1]*ENCODER_LOCALIZATION_FREQUENCY;

    beforeMove[0]=(-1)*(encoders[0]->getAmount());
    beforeMove[1]=encoders[1]->getAmount();

    beforeX=posX;
    beforeY=posY;

    //x,y,thetaの速度から　各モーターの速度[mm/s] をもとめる
    for(int i=0;i<4;i++)
     {
    motorSpeed[i]=-sin(direction+PI/4+i*(PI/2))*cos(direction)*speedX
                   +cos(direction+PI/4+i*(PI/2))*cos(direction)*speedY;//計算後で確認
     }

     direction=0;//回転しないと仮定
     rotateSpeed=0;
}


void Localization::setPosition(float X, float Y, float D){
    //位置を強制的に設定
    posX = X;
    posY = Y;
    direction = D;
}

void Localization::addLocalization(function<void(float*, float*, float*)> f, int tag, bool activate){
    functions[tag] = f;
    activations[tag] = activate;
}

void Localization::activateLocalization(int tag){
    activations[tag] = true;
}

void Localization::inactivateLocalization(int tag){
    activations[tag] = false;
}

void Localization::loop(){
    for(auto i = functions.begin(); i != functions.end(); ++i){
        if(activations[i->first]){
            (i->second)(&posX, &posY, &direction);
        }
    }
}

void Localization::resetAll(){

    posX = 0.0f;
    posY = 0.0f;
    direction = 0.0f;
    speedX = 0.0f;
    speedY = 0.0f;
    rotateSpeed = 0.0f;
    for(int i=0;i<4;i++){
        motorSpeed[i] = 0.0f;
    }
    beforeX = 0.0f;
    beforeY = 0.0f;
    beforeTheta = 0.0f;
    beforeMove[0] = 0.0f;
    beforeMove[1] = 0.0f;
}   
