#pragma once 

#include "Encoder/Encoder.hpp"
#include <map>
#include <functional>

using namespace std;

//自己位置推

class Localization{
    public:
        Encoder* encoders[2]; //エンコーダー
  
        //DT35 dt1, dt2, dt3;
        //Observation observation;をいれるとエラー このときはobservationを構成するDT35をもってくる

        Localization(Encoder* encoder1, Encoder* encoder2);

        //推定された位置
        float posX = 0.0f; //位置X[mm]
        float posY = 0.0f; //位置Y[mm]
        float direction = 0.0f; //方角D[rad](ずっと0と仮定)

        //推定された速度
        float rotateSpeed = 0.0f;
        float speedX = 0.0f;
        float speedY = 0.0f;

        float motorSpeed[4]={0,0,0,0};

        void setPosition(float X, float Y, float D); //外部から位置を強制的に設定する．
        void loop();
        void addLocalization(function<void(float*, float*, float*)> f, int tag, bool activate=true);
        void activateLocalization(int tag);
        void inactivateLocalization(int tag);
        
        void resetAll();
    

        

        int incrementedNumBefore[4] = {0,0,0,0};

        int _s1;



    private:
        void encoderLocalization(); //エンコーダーによる自己位置推定
        void Update();//もろもろの自己位置推定

        float beforeX=0;
        float beforeY=0;
        float beforeTheta=0;

        float beforeMove[2]={0,0};//エンコーダーの前の位置

        bool flag=false;
        bool kalman=true;//カルマンフィルタのオンオフ

        map<int, function<void(float*, float*, float*)>> functions;
        map<int, bool> activations;

        Ticker ticker;

        
};