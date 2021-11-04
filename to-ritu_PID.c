// 台車の倒立制御

#include "mbed.h"
#include "adrobo.h"
#include "Motor.h"
#include "QEI.h"

#define THETA_REF 0           //振子の目標値[rad]
#define ZERO_ADV 423          //棒の角度が0になる時のAD値
#define ADV_TO_RAD 0.003627705//ポテンショメータの値1当たりに傾く角度[rad]
#define PULSE_TO_METER 0.0005 //1パルス当たりに進む距離[m]
#define MAX_V 7.2             //駆動系の最大電圧[V] 1.2V×6本
#define T 0.001               //Δt(微笑時間t)
#define m 0.025               //振子質量
#define M 0.7                 //車体質量
#define g 9.81                //重力加速度
#define l 0.145               //振子の端から重心まで
#define f0 7.847              //フィードバック制御に必要な奴
#define f1 -3.92              //フィードバック制御に必要な奴
//#define KP 70                 //Pゲイン
#define KD 9                 //Dゲイン
#define KI 0.0001               //Iゲイン
//#define left 1
//#define right 1
#define K1 -0.31131390654529
#define K2 -0.7110620772229
//#define K3 -12.143656313343
//#define K4 -1.537260393318

BusOut led(D2, D4, D5, D7, D8); //基板LED用IO設定
AnalogIn pen(A0);               //ポテンショメータ用IO設定
Ticker pen_control;             //台車の制御用タイマー割り込み
Serial pc(USBTX, USBRX);        //デバッグ用シリアル通信

//モータ制御用オブジェクト
Motor motor_left(MOTOR11, MOTOR12);  //左モータ
Motor motor_right(MOTOR21, MOTOR22); //右モータ
BusIn in(GPIO1,GPIO2,GPIO3,GPIO4);
QEI qei_right(GPIO3, GPIO4, NC, 48, QEI::X4_ENCODING);
QEI qei_left(GPIO2, GPIO1, NC, 48, QEI::X4_ENCODING);

//***************　台車の制御　ここから　***************//
int KP = 70;
short theta_adv = 0, theta_adv0 = 0;//theta_adv=振子についているポテンショメータのAD値格納用
float left, right;                  //left＆right=ロータリーエンコーダーの値
double theta, e;                    //振子の角度
double ed, ei, e0 = 0;              //ed=角度の微分,ei=角度の積分,e0=0.01[s]前の角度差
double v_ref, duty_ratio;           //v_ref=電圧指令値,duty_ratio=デューティー比
double duty_ratio_0 = 0, check = 0; //duty_ratio_0=一つ前のduty_ratioの値,check=あまりにもかけ離れたduty_ratio値が出たときにerrorで返す用の変数
double vs_error_duty = 0;           //あまりにもかけ離れたduty_ratio値が出たときに正常な動作をモータにさせる用の変数
double x, x0 = 0, dx, dtheta, theta0;                     //x=台車位置(開始地点を0とする),x0=0.01[s]前の台車位置,dx=速度,dtheta=角速度,theta0=0.01[s]前の角度差

float duty_ratioo(short th_ad)
{
    theta = (double)(th_ad - ZERO_ADV) * ADV_TO_RAD; //鉛直なす角から振子までの角度[rad?]
    e = THETA_REF - theta;                           //目標角と現在の角の誤差
    if (e0 == 0)
        ed = 0;
    else
        ed = (e - e0) / T;
    ei += (e + e0) * T;
    e0 = e;
    if (ei > 1000)
        ei = 1000; //頭打ち処理
    if (ei < -1000)
        ei = -1000;

    //if (e < 0) ei -= 0.00001;
    if (e >= -2 || e <= 2) KP = 80;
    //if (e > 0) ei += 0.00001;
    //if (e >= 0.03) ei += 0.000032;

    v_ref = (e * KP) + (ei * KI) + (ed * KD); //  Calculate PID control ＆　(Calculate state feedback control)
/*
    left = qei_left.getPulses();
    right = qei_right.getPulses();
    qei_left.reset();
    qei_right.reset();
    x = (double)(left + right) / 2.0 * PULSE_TO_METER;
    //printf("%lf\r\n", x);
    dx = (x - x0) / T;
    x0 = x;
    //theta = e;
    //dtheta = ed;
    //theta0 = theta;  
    //v_ref = -(x*K1 + dx*K2 + theta*K3 + dtheta*K4);
    v_ref = (e*KP) + (ei*KI) + (ed*KD) + x*K1 + dx*K2;
*/
    duty_ratio = v_ref / MAX_V;
    if (duty_ratio >= 1)
        duty_ratio = 1;
    else if (duty_ratio <= -1)
        duty_ratio = -1;

    return duty_ratio;
}

void motor()
{
    if (duty_ratio_0 == 0)
        duty_ratio_0 = duty_ratio;
    check = duty_ratio_0 - duty_ratio;
    if (theta_adv > 520 || theta_adv < 250){
        duty_ratio = 0;
        motor_left = 0;
        motor_right = 0;
        }
    else if (check == 2.0 || check == -2.0)
    {
        motor_left = vs_error_duty; //計算結果をモータの速度指令に反映
        motor_right = -vs_error_duty;
        duty_ratio = vs_error_duty;
    }
    else
    {
        motor_left = duty_ratio;
        motor_right = -duty_ratio;
    }
    vs_error_duty = duty_ratio_0;
    duty_ratio_0 = duty_ratio;
    /*
    if (theta_adv > 600 || theta_adv < 200){
        motor_left = 0;
        motor_right = 0;
        }
    else{
    motor_left = duty_ratio; //計算結果をモータの速度指令に反映
    motor_right = -duty_ratio;
    }
    */
}

void duty_and_adv_control()
{
    while (1)
    {
        if ((ZERO_ADV > theta_adv && duty_ratio < 0) || (ZERO_ADV < theta_adv && duty_ratio > 0) || (ZERO_ADV == theta_adv && duty_ratio != 0))
        {
            theta_adv = pen.read_u16() >> 6;
            duty_ratio = duty_ratioo(theta_adv);
        }
        if ((theta_adv >= theta_adv0 && duty_ratio >= duty_ratio_0) || (theta_adv <= theta_adv0 && duty_ratio <= duty_ratio_0)) {
            theta_adv = pen.read_u16() >> 6;
            duty_ratio = duty_ratioo(theta_adv);
        }
        if (ZERO_ADV > theta_adv && duty_ratio > 0 || ZERO_ADV < theta_adv && duty_ratio < 0)
        {
            break;
        }
        if (ZERO_ADV == theta_adv && duty_ratio == 0)
        {
            break;
        }
        theta_adv0 = theta_adv;
    }
}

void pen_control_handler()
{                                    //制御の大元デザイン
    theta_adv = pen.read_u16() >> 6; //ADCを通してポテンショメータのAD値を取得 //搭載されているLPC1114のADCは10bitのため6bit右にシフト
    duty_ratio = duty_ratioo(theta_adv);

    duty_and_adv_control();

    //**** 指令値によって発光するLEDを変える ****//
    if (duty_ratio > 0.8)
        led = 8;
    else if (duty_ratio <= 0.8 && duty_ratio >= 0)
        led = 4;
    else if (duty_ratio < 0 && duty_ratio >= -0.8)
        led = 2;
    else if (duty_ratio < -0.8)
        led = 1;

    motor();
}

//***************　台車の制御　ここまで　***************//
//***************　main関数　ここから　***************//
int main()
{
    in.mode(PullUp);
    motor_left.setMaxRatio(1.0); //モータの最大電圧範囲を設定
    motor_right.setMaxRatio(1.0);
    pen_control.attach(&pen_control_handler, 0.001); //台車の制御用のタイマー関数を設定
    led = 1;                                       //LEDの値を設定　動作確認用
    //wait(0.01);
                                                //なんとなく1秒待つ
    while (1)
    {            
    //printf("%lf,%lf\r\n", left, right);                                                           //無限ループ
        printf("theta_adv:%d duty_ratio:%2.2f \r\n", theta_adv, duty_ratio); //teraterm出力用
        //wait(0.08);
    }

}
