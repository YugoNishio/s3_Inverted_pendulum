// 台車の倒立制御（P制御）

#include "mbed.h"
#include "adrobo.h"
#include "Motor.h"
#include "QEI.h"


#define THETA_REF    0              //振子の目標値(rad表記)
#define ZERO_ADV    500             //棒の角度が0になる時のAD値（機体により異なります）
#define ADV_TO_RAD  0.0046          //
#define PULSE_TO_METER  0.0005;
#define MAX_V       7.2             //駆動系の最大電圧
//#define left 1
//#define right 1
#define m 0.025
#define M 0.7
#define g 9.81
#define l 0.145
#define f0 7.847
#define f1 -3.92
#define KP          100             //Pゲイン
#define KD          10              //Dゲイン
#define KI          150             //Iゲイン
//#define K1 0
//#define K2 1+((4*f1)/(4*M+m))+((3*f1)/((4*M+m)*l))
//#define K3 ((-3*m*g)/(4*M+m))+((3(M+m)*g)/((4*M+m)*l))
//#define K4 1

BusOut led(D2,D4,D5,D7,D8);        //基板LED用IO設定
AnalogIn pen(A0);                  //ポテンショメータ用IO設定
Ticker pen_control;                //台車の制御用タイマー割り込み
Serial pc(USBTX, USBRX);           //デバッグ用シリアル通信

//モータ制御用オブジェクト
Motor motor_left(MOTOR11, MOTOR12);       //左モータ
Motor motor_right(MOTOR21, MOTOR22);      //右モータ

//***************　台車の制御　ここから　***************//
int theta_adv = 0;                                           //振子についているポテンショメータのAD値格納用
double theta, e;                                             //振子の角度
double ed, ei, e0 = 0, T = 0.01;
double v_ref,v_ref_2, duty_ratio;                                    //電圧指令値,デューティー比
double x, x0 = 0, dx, dtheta, theta0;
double K1, K2, K3, K4;

void pen_control_handler(){                                 //制御の大元デザイン
    theta_adv = pen.read_u16()>>6;                          //ADCを通してポテンショメータのAD値を取得
                                                            //搭載されているLPC1114のADCは10bitのため6bit右にシフト
    
    theta = (double)(theta_adv - ZERO_ADV) * ADV_TO_RAD;
    e = THETA_REF - theta;                                  //誤差
    ed = (e - e0) / T;
    ei += e * T;
    e0 = e;

    if(ei > 10000) ei = 10000;
    if(ei < -10000) ei = -10000;

    //  Calculate PID control
    v_ref = (e * KP + ei * KI + ed * KD);

    //  Introduce x, dx, theta, dtheta
    //x = (float)(left + right) / 2 * PULSE_TO_METER;
    //dx = (x - x0) / T;
    //x0 = x;
    theta = e;
    dtheta = ed;
    theta0 = theta;

    //  Calculate state feedback control
    K1 = 0;
    K2 = 1+((4*f1)/(4*M+m))+((3*f1)/((4*M+m)*l));
    K3 = ((-3*m*g)/(4*M+m))+((3*(M+m)*g)/((4*M+m)*l));
    K4 = 1;
    v_ref_2 = v_ref - (K1 + K2 + K3 + K4);
    duty_ratio = v_ref_2 / MAX_V;
    if (duty_ratio > 1) duty_ratio = 1;
    else if (duty_ratio < -1) duty_ratio = -1;
    
    //指令値の頭打ち処理
    if(duty_ratio > 1.0) duty_ratio = 1.0;
    if(duty_ratio < -1.0) duty_ratio = -1.0;
    
    //**** 指令値によって発光するLEDを変える ****//
        if(duty_ratio > 0.8 ){
            led = 8;
        }else if(duty_ratio <= 0.8 && duty_ratio >= 0){
            led = 4;
        }else if(duty_ratio < 0 && duty_ratio >= -0.8){
            led = 2;
        }else if(duty_ratio < -0.8){
            led = 1;
        }
    //**** 指令値によって発光するLEDを変える　ここまで ****//
    
    //計算結果をモータの速度指令に反映

    motor_left =  duty_ratio;
    motor_right = -duty_ratio;

}
//***************　台車の制御　ここまで　***************//

//***************　main関数　ここから　***************//
int main() {
    //モータの最大電圧範囲を設定
    motor_left.setMaxRatio(1.0);
    motor_right.setMaxRatio(1.0);
    
    pen_control.attach(&pen_control_handler, 0.001);        //台車の制御用のタイマー関数を設定
    
    led = 1;                                                //LEDの値を設定　動作確認用
    
    wait(1.0);                                              //なんとなく1秒待つ
    
    while(1) {                                              //無限ループ
        printf("theta_adv:%d duty_ratio:%2.2f \r\n", theta_adv ,duty_ratio);
        
        wait(0.08);
    }
}
