#include "mbed.h"

#define MAX_SPEED 0.4       //本体移動最高速度
#define MAX_ROTATE_X 0.003  //発射機構X軸回転最高速度
#define MAX_ROTATE_Y 0.002  //発射機構Y軸回転最高速度
#define MAX_FIRING 0.006    //発射機構打ち出し最高速度

void Init_Robot(void);
void Init_L(void);
void Init_R(void);
void Init_C(void);
void Init_PWM(void);
inline void Roll_L(void);
inline void Roll_R(void);
inline void Roll_C_A(void);
inline void Roll_C_M(void);
inline void Firing_L(void);
inline void Firing_R(void);
inline void Firing_C(void);
inline void MoveChassis(void);
inline void MovePWM(int status);

inline void CheckKey(void);

void Config(void);
inline void PS3Data(void);


RawSerial PC(USBTX, USBRX);             //PC通信用ポート
RawSerial SBDBT(PC_10, PC_11, 2400);    //SBDBT(Bluetooth無線モジュール)通信用ポート

DigitalOut RFL_L(PC_2);                 //左側発射機構 X軸回転or発射左モータ 回転方向指定ピン
DigitalOut RFR_L(PC_3);                 //左側発射機構 Y軸回転or発射右モータ 回転方向指定ピン
DigitalOut RFC_L[2] = {PC_1, PC_0};     //左側発射機構 回転・発射モード切替ピン(リレー信号)

DigitalOut RFL_R(PD_7);                 //右側発射機構 X軸回転or発射左モータ 回転方向指定ピン
DigitalOut RFR_R(PE_3);                 //右側発射機構 Y軸回転or発射右モータ 回転方向指定ピン
DigitalOut RFC_R[2] = {PE_2, PE_4};     //右側発射機構 回転・発射モード切替ピン(リレー信号)

DigitalOut RX_C(PC_4);                  //中央発射機構 X軸回転 回転方向指定ピン
DigitalOut RY_C(PF_5);                  //中央発射機構 Y軸回転 回転方向指定ピン
DigitalOut Roller_C[2] = {PF_4, PE_8};  //中央発射機構 発射モータ 回転方向指定ピン


PwmOut RFL_L_p(PC_9);                   //左側発射機構 X軸回転or発射左モータ 回転速度指定ピン
PwmOut RFR_L_p(PC_8);                   //左側発射機構 Y軸回転or発射右モータ 回転速度指定ピン

PwmOut RFL_R_p(PD_13);                  //右側発射機構 X軸回転or発射左モータ 回転速度指定ピン
PwmOut RFR_R_p(PE_10);                  //右側発射機構 Y軸回転or発射右モータ 回転速度指定ピン

PwmOut RX_C_p(PD_14);                   //中央発射機構 X軸回転 回転速度指定ピン
PwmOut RY_C_p(PD_15);                   //中央発射機構 Y軸回転 回転速度指定ピン
PwmOut Roller_C_p[2] = {PB_1, PB_15};   //中央発射機構 発射モータ 回転速度指定ピン


DigitalOut Chassis[4] = {PF_6, PA_14, PA_13, PF_7}; //足回りモータ 回転方向指定ピン
PwmOut Chassis_p[4] = {PA_6, PB_6, PC_7, PA_7};     //足回りモータ 回転速度指定ピン


int PS3[7];                 //コントローラ信号(バイナリ)
int flag_L, flag_R, flag_C; //発射フラグ
int flag_Command;           //コマンドモードフラグ
int counter;                //中央発射機構自動首振り用カウンタ
int status;                 //移動・発射モード判定

double L_rx, L_ry;          //左側発射機構 X・Y軸回転PWM値(デューティー比)
double R_rx, R_ry;          //右側発射機構 X・Y軸回転PWM値(デューティー比)
double C_rx, C_ry;          //中央発射機構 X・Y軸回転PWM値(デューティー比)




int main(void){
    PC.printf("The programme was successfully written.\r\n");   //プログラム書き込み完了確認
    
    Init_Robot();
    
    while(1){

        //左右の発射フラグを、そのままリレー信号として発射・移動切替リレーに突っ込む
        RFC_L[0] = flag_L;                                      
        RFC_L[1] = flag_L;
        
        RFC_R[0] = flag_R;
        RFC_R[1] = flag_R;
        

        if(!status){                                            //statusの確認(falseなら移動モード)
            MoveChassis();

            //◯ボタンが押下されたら発射モードに切替
            if(PS3[2] == 0x40)                                  
                status = 1;
            
        }else{                                                  //statusの確認(trueなら発射モード)

            //中央発射機構の速度指定初期化()
            RX_C_p = 0;
            RY_C_p = 0;

            //左側発射機構フラグ確認処理(trueなら発射、falseなら首振り)
            if(flag_L)                                          
                Firing_L();
            else                                                
                Roll_L();

            //右側発射機構フラグ確認処理(trueなら発射、falseなら首振り)
            if(flag_R)
                Firing_R();
            else
                Roll_R();

            //中央発射機構フラグ確認処理(trueなら発射)
            if(status && flag_C)
                Firing_C();
            //コマンドモードフラグ確認処理(trueなら手動首振り、falseなら自動首振り)
            else if(flag_Command)
                Roll_C_M();
            else
                Roll_C_A();
            


            //左側機構発射コマンドの確認(☐＋L1)
            if(PS3[1] == 0x03)
                flag_L = 1;
            
            //右側機構発射コマンドの確認(☐＋R1)
            if(PS3[1] == 0x09)
                flag_R = 1;
            
            //自動・手動切替コマンド入力モードコマンドの確認(START)
            if(PS3[2] == 0x03)
                CheckKey();
            
            //中央機構発射コマンドの確認(手動モードかつ、☐＋☓)
            if(flag_Command && (PS3[1] == 0x01) && (PS3[2] == 0x20))
                flag_C = 1;
            
            //◯＋☓ボタンが押下されたら移動モードに切替
            if(PS3[2] == 0x60){
                status = 0;
                wait_ms(500);
            }
        }
    }
}


//ロボット全体の初期化
void Init_Robot(){
    
    Config();
    
    PC.printf("Enter down with the left and right sticks.\r\n");
    do{wait_ms(1);}while((PS3[4] != 0x7F) && (PS3[6] != 0x7F));         //両スティック下方向入力待ち
    
    Init_L();
    Init_R();
    Init_C();
    Init_PWM();

    PC.printf("Initialisation of Robot is completed.\r\n\r\n\r\n\r\n");
    wait_ms(500);
    PC.printf("Press CIRCLE button to exit\r\n");    
    do{wait_ms(1);}while(PS3[2] != 0x40);                               //◯ボタン入力待ち
    wait_ms(1000);
}


//左側発射機構の初期化
void Init_L(){

    //各方向指定初期化
    RFL_L = 0;
    RFR_L = 0;
    RFC_L[0] = 0;
    RFC_L[1] = 0;
    wait_ms(100);
    
    //各速度指定初期化
    RFL_L_p = 0.0;
    RFR_L_p = 0.0;
    wait_ms(100);
    
    //PWM周期の調整(速さ・トルクが変化する)
    RFL_L_p.period_ms(12);
    RFR_L_p.period_ms(12);  
    
    PC.printf("Initialisation of UNIT_L is completed.\r\n");
    wait_ms(100);
}

//右側発射機構の初期化
void Init_R(){

    //各方向指定初期化
    RFL_R = 0;
    RFR_R = 0;
    RFC_R[0] = 0;
    RFC_R[1] = 0;
    wait_ms(100);
    
    //各速度指定初期化
    RFL_R_p = 0.0;
    RFR_R_p = 0.0;
    wait_ms(100);
    
    //PWM周期の調整(速さ・トルクが変化する)
    RFL_R_p.period_ms(12);
    RFR_R_p.period_ms(12);
    
    PC.printf("Initialisation of UNIT_R is completed.\r\n");
    wait_ms(100);
}

//中央発射機構の初期化
void Init_C(){

    //各方向指定初期化
    RX_C = 0;
    RY_C = 0;
    Roller_C[0] = 0;
    Roller_C[1] = 0;
    wait_ms(100);
    
    //各速度指定初期化
    RX_C_p = 0.0;
    RY_C_p = 0.0;
    Roller_C_p[0] = 0.0;
    Roller_C_p[1] = 0.0;
    wait_ms(100);
    
    //PWM周期の調整(速さ・トルクが変化する)
    RX_C_p.period_ms(12);
    RY_C_p.period_ms(12);
    Roller_C_p[0].period_ms(5);
    Roller_C_p[1].period_ms(5);
    
    flag_Command = 0;
    counter = 0;
    
    PC.printf("Initialisation of UNIT_C is completed.\r\n");
    wait_ms(100);
}


//足回り用PWM信号の初期化
void Init_PWM(){

    //PWM周期の調整(速さ・トルクが変化する
    Chassis_p[0].period_us(100);
    Chassis_p[1].period_us(100);
    Chassis_p[2].period_us(100);
    Chassis_p[3].period_us(100); 
    
    PC.printf("Initialisation of PWM is completed.\r\n");
    wait_ms(200);
}


//左側発射機構の首振り
inline void Roll_L(){

    //コントローラーのLスティック入力(X,Yともに0~128)を速度フラグ(1~-1)への値域変換(※やってることはmap関数と同じ)
    L_rx = ((double)(PS3[3] - 64) / 64);
    L_ry = ((double)(PS3[4] - 64) / -64);
    
    //方向指定フラグの判定(速度フラグが負数なら方向逆転)
    int RX_L_flag = (L_rx >= 0);
    int RY_L_flag = (L_ry <= 0);
    
    //方向指定フラグを方向指定ピンへ
    RFL_L = RX_L_flag;
    RFR_L = RY_L_flag;
    
    //Y軸速度指定(首を上に上げる時は、重心の関連で速度40%制限)、速度フラグの絶対値と首振り最高速度を掛け合わせる
    if(!RY_L_flag){
        RFR_L_p = fabs(L_ry)* MAX_ROTATE_Y * 0.4;
    }else{
        RFR_L_p = fabs(L_ry)* MAX_ROTATE_Y;
    }

    //X軸速度指定、速度フラグの絶対値と首振り最高速度を掛け合わせる
    RFL_L_p = fabs(L_rx)* MAX_ROTATE_X;
}


//右側発射機構の首振り
inline void Roll_R(){

    //コントローラーのRスティック入力(X,Yともに0~128)を1~-1への値域変換(※やってることはmap関数と同じ)
    R_rx = ((double)(PS3[5] - 64) / 64);
    R_ry = ((double)(PS3[6] - 64) / -64);
    
    //方向指定フラグの判定(負数なら方向逆転)
    int RX_R_flag = (R_rx >= 0);
    int RY_R_flag = (R_ry <= 0);
    
    //方向指定フラグを方向指定ピンへ
    RFL_R = RX_R_flag;
    RFR_R = RY_R_flag;
    
    //Y軸速度指定(首を上に上げる時は、重心の関連で速度40%制限)、速度フラグの絶対値と首振り最高速度を掛け合わせる
    if(!RY_R_flag){
        RFR_R_p = fabs(R_ry)* MAX_ROTATE_Y * 0.4;
    }else{
        RFR_R_p = fabs(R_ry)* MAX_ROTATE_Y;
    }

    //X軸速度指定、速度フラグの絶対値と首振り最高速度を掛け合わせる
    RFL_R_p = fabs(R_rx)* MAX_ROTATE_X;
}


//中央発射機構の手動首振り
inline void Roll_C_M(){

    //速度指定初期化
    RX_C_p = 0;
    RY_C_p = 0;
    

    switch(PS3[2]){
        //△＋上キーの入力で、上向きに首振り
        case 0x11:
            RY_C = 0;
            RY_C_p = MAX_ROTATE_Y * 0.4;
            PC.printf("UNIT_C is rotating to the Up.\r\n");
            break;
        //△＋下キーの入力で、上向きに首振り
        case 0x12:
            RY_C = 1;
            RY_C_p = MAX_ROTATE_Y;
            PC.printf("UNIT_C is rotating to the Down.\r\n");
            break;
        //△＋右キーの入力で、上向きに首振り
        case 0x14:
            RX_C = 1;
            RX_C_p = MAX_ROTATE_X;
            PC.printf("UNIT_C is rotating to the Right.\r\n");
            break;
        //△＋左キーの入力で、上向きに首振り
        case 0x18:
            RX_C = 0;
            RX_C_p = MAX_ROTATE_X;
            PC.printf("UNIT_C is rotating to the Left.\r\n");
            break;
        //定常時は停止
        default:
            RX_C_p = 0;
            RY_C_p = 0;
    }
}


//中央発射機構の自動首振り
inline void Roll_C_A(void){

    //自動首振り用カウンタ加算
    ++counter;

    //上方向に首振り(0~4ステップ)
    if(counter < 5){
        RY_C = 0;
        RY_C_p = MAX_ROTATE_Y * 0.4;
        PC.printf("UNIT_C is rotating to the Up.\r\n");

    //右方向に首振り(5~9ステップ)
    }else if((counter >= 5) && (counter < 10)){
        RX_C = 1;
        RX_C_p = MAX_ROTATE_X;
        PC.printf("UNIT_C is rotating to the Right.\r\n");
    
    //下方向に首振り(10~14ステップ)
    }else if((counter >= 10) && (counter < 15)){
        RY_C = 1;
        RY_C_p = MAX_ROTATE_Y;
        PC.printf("UNIT_C is rotating to the Down.\r\n");
    
    //左方向に首振り(15~19ステップ)
    }else if((counter >= 15) && (counter < 20)){
        RX_C = 0;
        RX_C_p = MAX_ROTATE_X;
        PC.printf("UNIT_C is rotating to the Left.\r\n");

    //カウンタリセット(20ステップ)
    }else if(counter == 20)
        counter = 0;
}
        
        

//左側発射機構の発射
inline void Firing_L(void){

    //リレー切替用のマージ
    wait_ms(20);

    //方向・速度を指定し、発射(300ms)
    RFL_L = 1;
    RFR_L = 0;
    RFL_L_p = MAX_FIRING;
    RFR_L_p = MAX_FIRING;
    PC.printf("UNIT_L is Firing!!\r\n");
    wait_ms(300);
    
    //方向・速度を初期化し、休止(500ms)
    RFL_L_p = 0.00;
    RFR_L_p = 0.00;
    RFL_L = 0;
    RFR_L = 1;
    PC.printf("UNIT_L is Cooling.\r\n");
    wait_ms(500);
    
    //発射時とは逆方向、速度半分で収納(550ms)
    RFL_L_p = MAX_FIRING * 0.5;
    RFR_L_p = MAX_FIRING * 0.5;
    PC.printf("UNIT_L is Restoring...\r\n");
    wait_ms(550);
    
    //方向・速度を初期化し、発射フラグを戻し終了
    RFL_L_p = 0.00;
    RFR_L_p = 0.00;
    RFL_L = 1;
    RFR_L = 0;
    PC.printf("UNIT_L is Ready.\r\n");
    flag_L = 0;
}


//右側発射機構の発射
inline void Firing_R(void){

    //リレー切替用のマージ
    wait_ms(20);

    //方向・速度を指定し、発射(200ms)
    RFL_R = 1;
    RFR_R = 0;
    RFL_R_p = MAX_FIRING;
    RFR_R_p = MAX_FIRING;
    PC.printf("UNIT_R is Firing!!\r\n");
    wait_ms(200);
    
    //方向・速度を初期化し、休止(500ms)
    RFL_R_p = 0.00;
    RFR_R_p = 0.00;
    RFL_R = 0;
    RFR_R = 1;
    PC.printf("UNIT_R is Cooling.\r\n");
    wait_ms(500);
    
    //発射時とは逆方向、速度半分で収納(450ms)
    RFL_R_p = MAX_FIRING * 0.5;
    RFR_R_p = MAX_FIRING * 0.5;
    PC.printf("UNIT_R is Restoring...\r\n");
    wait_ms(450);
    
    //方向・速度を初期化し、発射フラグを戻し終了
    RFL_R_p = 0.00;
    RFR_R_p = 0.00;
    RFL_R = 1;
    RFR_R = 0;
    PC.printf("UNIT_R is Ready.\r\n");
    flag_R = 0;
}


//中央発射機構の発射
inline void Firing_C(void){
    
    //方向・速度を指定し、発射(100ms)
    Roller_C[0] = 1;
    Roller_C[1] = 0;
    Roller_C_p[0] = MAX_FIRING;
    Roller_C_p[1] = MAX_FIRING;
    PC.printf("UNIT_C is Firing!!\r\n");
    wait_ms(100);
    
    //方向逆転、速度を初期化し、休止(500ms)
    Roller_C_p[0] = 0.00;
    Roller_C_p[1] = 0.00;
    Roller_C[0] = 0;
    Roller_C[1] = 1;
    PC.printf("UNIT_C is Cooling.\r\n");
    wait_ms(500);
    
    //速度半分で収納(225ms)
    Roller_C_p[0] = MAX_FIRING * 0.5;
    Roller_C_p[1] = MAX_FIRING * 0.5;
    PC.printf("UNIT_C is Restoring...\r\n");
    wait_ms(225);
    
    //方向・速度を初期化し、発射フラグを戻し終了
    Roller_C_p[0] = 0.00;
    Roller_C_p[1] = 0.00;
    Roller_C[0] = 1;
    Roller_C[1] = 0;
    PC.printf("UNIT_C is Ready.\r\n");
    flag_C = 0;
}




//足回り駆動部
inline void MoveChassis(void){
    switch(PS3[2]){
        
        //上キー入力で前移動
        case 0x01:
            Chassis[0] = 0;
            Chassis[1] = 1;
            Chassis[2] = 1;
            Chassis[3] = 0;
            PC.printf("Robot is moving Forward.\r\n");
            MovePWM(1);
            break;
        
        //下キー入力で後移動
        case 0x02:
            Chassis[0] = 1;
            Chassis[1] = 0;
            Chassis[2] = 0;
            Chassis[3] = 1;
            PC.printf("Robot is moving Back.\r\n");
            MovePWM(1);
            break;

        //右キー入力で右移動
        case 0x04:
            Chassis[0] = 0;
            Chassis[1] = 0;
            Chassis[2] = 1;
            Chassis[3] = 1;
            MovePWM(1);
            PC.printf("Robot is moving Right.\r\n");
            break;
        
        //左キー入力で左移動
        case 0x08:
            Chassis[0] = 1;
            Chassis[1] = 1;
            Chassis[2] = 0;
            Chassis[3] = 0;
            MovePWM(1);
            PC.printf("Robot is moving Left.\r\n");
            break;

        default:
            PC.printf("%d\r\n", PS3[1]);

            //△キーが押されているか？(本体回転モード)
            if(PS3[2] == 0x10){
                switch(PS3[1]){

                    //△＋L2で左回転(反時計回り)
                    case 0x04:
                        Chassis[0] = 1;
                        Chassis[1] = 1;
                        Chassis[2] = 1;
                        Chassis[3] = 1;
                        MovePWM(1);
                        PC.printf("Robot is rotating Left.\r\n");
                        break;

                    //△＋R2で右回転(時計回り)
                    case 0x10:
                        Chassis[0] = 0;
                        Chassis[1] = 0;
                        Chassis[2] = 0;
                        Chassis[3] = 0;
                        MovePWM(1);
                        PC.printf("Robot is rotating Right.\r\n");
                        break;
                    
                    //本体回転モード中かつ無入力時は駆動停止
                    default:
                        MovePWM(0);
                }
            }

            //無入力時は駆動停止
            else
                MovePWM(0);
    }
}


//足回り速度指定部
inline void MovePWM(int status_chassis){

    //速度値に本体移動最高速度を掛け合わせ、PWM信号で出力
    Chassis_p[0] = status_chassis * MAX_SPEED;
    Chassis_p[1] = status_chassis * MAX_SPEED;
    Chassis_p[2] = status_chassis * MAX_SPEED;
    Chassis_p[3] = status_chassis * MAX_SPEED;
    
    PC.printf("%x\tstatus = %d\r\n", PS3[2], status_chassis);
}



//自動・手動切替コマンド入力モード
inline void CheckKey(void){
    PC.printf("Command Mode\r\n");
    wait_ms(1000);

    //再びSTARTが押されるまで待機
    do{
        //L1＋R1＋左キー＋上キー同時入力時に、手動モードフラグを立てる
        if((flag_Command == 0) && (PS3[1] == 0x0A) && (PS3[2] == 0x09)){
            flag_Command = 1;
            PC.printf("Welcome to Secret(Manual) Mode!!!\r\n");
        }
    }while(PS3[2] != 0x03);

    PC.printf("Control Mode\r\n");
    wait_ms(1000);
}


//Serial通信(UART)の設定
void Config(void){
    SBDBT.attach(&PS3Data, Serial::RxIrq);      //受信割込みの時にPS3Data関数を呼び出すよう設定
}


//PS3コントローラーからの入力処理
inline void PS3Data(void)
{
    
    //生データ受け取り
    int SBDBT_Data = SBDBT.getc();
    static int bits = 0;
    
    if(SBDBT_Data == 128){
        bits = 0;
    }

    //各配列にデータを格納
    if (SBDBT_Data >= 0){
        PS3[bits] = SBDBT_Data;
        
        if (bits == 7){
            bits = 0;
        }
        else {
            bits++;
        }
    }
}
