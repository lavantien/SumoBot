#include <regx52.h>
#include <intrins.h>

sbit PWM_MOTOR = P2 ^ 7; //timer2, PWM for motors
sbit PWM_SONIC = P2 ^ 6; //timer1, PWM for sonic
sbit ECHO = P3 ^ 2; //ultrasonic sensor echo pin

sbit B0 = P2 ^ 0; //motor B
sbit B1 = P2 ^ 1;
sbit A0 = P2 ^ 2; //motor A
sbit A1 = P2 ^ 3;

sbit LFL = P1 ^ 5; //line detector at front, left
sbit LFR = P1 ^ 6; //line detector at front, right
sbit LB = P1 ^ 7; //line detector at back, left

sbit IF = P1 ^ 1; //infrared enemy detector at front
sbit IL = P1 ^ 2; //infrared enemy detector at left
sbit IR = P1 ^ 3; //infrared enemy detector at right
sbit IB = P1 ^ 4; //infrared enemy detector at back

sfr16 DPTR = 0x82;

const unsigned char PERCENT_SPEED = 33;

unsigned t2, T2on, T2off, Tstable;
unsigned char T2on_high_reload, T2on_low_reload, T2off_high_reload, T2off_low_reload;
unsigned T1on, T1off;
unsigned char T1on_high_reload, T1on_low_reload, T1off_high_reload, T1off_low_reload;

unsigned char sent = 0; //ultrasonic pulses sent flag

// Timer 2, PWM for motors
void PWM2_start(unsigned ck); //micro sec T with 50% duty cycle
void PWM2_less_speed(void); //percenetage 1 to 99
void PWM2_max_speed(void); //100% duty cycle
void PWM2_pause(void); //0% duty cycle
void PWM2_stop(void);
void Timer2_ISR(void); /*interrupt 5*/

// Timer 1, Timing for sonic
void PWM1_start(void); //micro sec
void PWM1_stop(void);
void Timer1_ISR(void); /*interrupt 3*/

// Timer 0, Receive echo from sonic radar and calculate distance
void radar_start(void);

// Robot control
void delayms(unsigned dl); //milli sec, ratio = 1.19
void run_forward(void);
void run_back(void);
void spin_l(void);
void spin_r(void);
void turn_l(void);
void turn_r(void);
void leftIRres(void);
void rightIRres(void);

void main(void) {
	PT2 = 1;
	P1 = 0xFF; //enable input of port 1
	PWM_MOTOR = 0;
	PWM_SONIC = 0;
	PWM1_start();
	Tstable = 0x7FFF; //T = 0.033 s -> f = 30 Hz
	delayms(2400);
	PWM2_start(Tstable); //start at 50% speed
	spin_l();
	radar_start();
	for (;;) {
		//line detected
		if (!LFL) {
			run_back();
			PWM2_max_speed();
			delayms(750);
			PWM2_less_speed();
			turn_r();
		} else if (!LFR) {
			run_back();
			PWM2_max_speed();
			delayms(750);
			PWM2_less_speed();
			turn_l();
		} else if (!LB || !IF) { //line at back or enemy in front
			run_forward();
			PWM2_max_speed();
		} else if (!IL) {
			leftIRres();
			PWM2_max_speed();
		} else if (!IR) {
			rightIRres();
			PWM2_max_speed();
		} else if (!IB) { //enemy kiss my ass
			spin_r();
			PWM2_max_speed();
		} else if (ECHO && !sent) { //sonic send pulses
			TH0 = TL0 = 0x00;
			TR0 = 1;
			sent = 1;
		} else if (!ECHO && sent) { //return pulses recived
			TR0 = 0;
			TF0 = 0;
			sent = 0;
			DPH = TH0;
			DPL = TL0;
			if (DPTR < 3000) {
				run_forward();
				PWM2_max_speed();
			}
		}
	}
}

// Timer 2, PWM for motors
void PWM2_start(unsigned ck) {
	T2MOD = 0x00;
	T2CON = 0x00;
	t2 = ck;
	T2on = (unsigned long)t2 * PERCENT_SPEED / 100;
	T2off = t2 - T2on;
	T2on_high_reload = (65536 - T2on) >> 8;
	T2on_low_reload = (65536 - T2on) & 0x00FF;
	T2off_high_reload = (65536 - T2off) >> 8;
	T2off_low_reload = (65536 - T2off) & 0x00FF;
	if (!EA) {
		EA = 1;
	}
	PWM2_less_speed();
	RCAP2H = T2on_high_reload;
	RCAP2L = T2on_low_reload;
	TR2 = 1;
}
void PWM2_less_speed(void) {
	ET2 = 1;
}
void PWM2_max_speed(void) {
	PWM_MOTOR = 1;
	ET2 = 0;
}
void PWM2_pause(void) {
	PWM_MOTOR = 0;
	ET2 = 0;
}
void PWM2_stop(void) {
	TR2 = 0;
}
void Timer2_ISR(void) interrupt 5 {
	PWM_MOTOR = !PWM_MOTOR;
	if (PWM_MOTOR == 0) {
		RCAP2H = T2off_high_reload;
		RCAP2L = T2off_low_reload;
	} else {
		RCAP2H = T2on_high_reload;
		RCAP2L = T2on_low_reload;
	}
	TF2 = 0;
}

// Timer 1, PWM for sonic
void PWM1_start(void) {
	TMOD &= 0x0F;
	TMOD |= 0x10;
	T1on = 1;
	T1off = 65535;
	T1on_high_reload = (65536 - T1on) >> 8;
	T1on_low_reload = (65536 - T1on) & 0x00FF;
	T1off_high_reload = (65536 - T1off) >> 8;
	T1off_low_reload = (65536 - T1off) & 0x00FF;
	if (!EA) {
		EA = 1;
	}
	ET1 = 1;
	TH1 = T1on_high_reload;
	TL1 = T1on_low_reload;
	TR1 = 1;
}
void PWM1_stop(void) {
	TR1 = 0;
}
void Timer1_ISR(void) interrupt 3 {
	PWM_SONIC = !PWM_SONIC;
	if (PWM_SONIC == 0) {
		TH1 = T1off_high_reload;
		TL1 = T1off_low_reload;
	} else {
		TH1 = T1on_high_reload;
		TL1 = T1on_low_reload;
	}
}

// Timer 0, Receive echo from sonic radar and calculate distance
void radar_start(void) {
	TMOD &= 0xF0;
	TMOD |= 0x01;
}

// Robot control
void delayms(unsigned dl) {
	unsigned i, j;
	for (i = 0; i < dl; ++i) {
		for (j = 0; j < 10; ++j) {
			_nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_();
			_nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_();
			_nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_();
			_nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_();
			_nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_();
			_nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_();
			_nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_();
			_nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_();
			_nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_();
			_nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_(); _nop_();
		}
	}
}
void run_forward(void) {
	A0 = 1;
	A1 = 0;
	B0 = 1;
	B1 = 0;
}
void run_back(void) {
	A0 = 0;
	A1 = 1;
	B0 = 0;
	B1 = 1;
}
void spin_l(void) {
	A0 = 0;
	A1 = 1;
	B0 = 1;
	B1 = 0;
}
void spin_r(void) {
	A0 = 1;
	A1 = 0;
	B0 = 0;
	B1 = 1;
}
void turn_l(void) {
	A0 = 0;
	A1 = 0;
	B0 = 1;
	B1 = 0;
}
void turn_r(void) {
	A0 = 1;
	A1 = 0;
	B0 = 0;
	B1 = 0;
}
void leftIRres(void) {
	A0 = 0;
	A1 = 0;
	B0 = 0;
	B1 = 1;
}
void rightIRres(void) {
	A0 = 0;
	A1 = 1;
	B0 = 0;
	B1 = 0;
}

































































// Writen by La Van Tien - 15520878 - MTCL2015.2 - UIT - HCM VNU - Viet Nam