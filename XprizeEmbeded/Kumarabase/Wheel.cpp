#include <mbed.h>
#include <Wheel.h>

using namespace baseDrive;

// Wheel::Wheel(PinName in_pwmf,PinName in_pwmb){
//     pwmf = new PwmOut(in_pwmf);
//     pwmb = new PwmOut(in_pwmb);
//     // pwm = new PwmOut(in_pwm);
// };
#define avg_t 19
Wheel::Wheel(PinName ina, PinName inb, PinName pwm){
    outa = new DigitalOut(ina);
    outb = new DigitalOut(inb);
    pwmf = new PwmOut(pwm);
    // pwmf->period(0.0013);
    pwmf->period(0.0013);
    // pwm = new PwmOut(in_pwm);
};

// void Wheel::drive(float value){
//     if (value>0){
//       // *outf = 1;
//       // *outb = 0;
//       pwmf->write(value);
//     }
//     else if (value<0){
//       // *outf=0;
//       // *outb=1;
//       pwmb->write(-value);
//     }
//     else if(value == 0){
//       // *outf=0;
//       // *outb=0;
//       // pwm->write(0);
//       pwmf->write(value);
//       pwmb->write(-value);
//     }
// }

void Wheel::drive(float value){
  if (value>0){
      *outa = 1;
      *outb = 0;
      // pwmf->write(value);
    }
    else if (value<0){
      *outa=0;
      *outb=1;
      // pwmb->write(-value);
    }
    else if(value == 0){
      *outa=0;
      *outb=0;
      // pwm->write(0);
    }
    // if(value - lasted_pwm > 0.01 & value > 0.5){
    //   lasted_pwm += 0.01;
    // }
    // else{
    //   lasted_pwm = value;
    // }
  pwmf->write(abs(value));
}

void Driver::setWheel(int position,PinName ina, PinName inb, PinName pwm) {
    wheel[position] = new Wheel(ina,inb,pwm);
}
// void Driver::hwsetWheel(int position,PinName ina, PinName inb, PinName pwm){
//     wheel[position] = new Wheel(ina, inb, pwm);
// }

void Driver::drive(float Vx,float Vy,float Wz,float speed_satuated){
    
     if (abs(Vx) >speed_satuated){
      Vx = speed_satuated * (abs(Vx)/Vx);
    }
    if (abs(Vy) >speed_satuated){
      Vy = speed_satuated * (abs(Vy)/Vy);
    }
    // float value[4];
    value[0] =((value[0]*(avg_t-1))+(Vx - Vy + (Wz*(-0.5868))))/(avg_t);  //  sum xy จากหลางล้อ ถึง origin
    value[1] =((value[1]*(avg_t-1))+(Vx + Vy + (Wz*(0.5868))))/avg_t;  // รถใหม่ 0.77472 m. moving average19/20
    value[2] =((value[2]*(avg_t-1))+(Vx + Vy + (Wz*(-0.5868))))/avg_t; 
    value[3] =((value[3]*(avg_t-1))+(Vx - Vy + (Wz*(0.5868))))/avg_t;
    

    // wheel[0]->hwdrive(value[0]);

    for(int i=0;i<4;i++){
        wheel[i]->drive(value[i]);
    }
}