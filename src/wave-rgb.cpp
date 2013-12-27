#include <Arduino.h>
#include "HardwareSerial.h"

typedef struct _int_bresenham {
	short ante, conseq;
	short ante_err, conseq_err;
} int_bresenham;

/* 1.2 microsecs */
static inline byte dither_bresenham(int_bresenham *br) {
	byte ret;
	if (br->ante == 0)
		return 1;
	if (br->conseq == 0)
		return 2;
	ret = 0;
	if (br->ante_err < br->ante && br->conseq_err < br->conseq) {
		br->conseq_err += br->ante;
		br->ante_err += br->conseq;
	}
	if (br->ante <= br->ante_err) {
		++ret;
		br->ante_err -= br->ante;
	}
	if (br->conseq <= br->conseq_err) {
		ret |= 2;
		br->conseq_err -= br->conseq;
	}
	return ret;
}

// the setup routine runs once when you press reset:
void setup_timer1() {
	noInterrupts();
	// disable all interrupts
	TCCR1A = 0;
	TCCR1B = 0;
	TCNT1 = 0;

	OCR1A = 1;
	TCCR1B |= (1 << WGM12);   // CTC mode
//	TCCR1B |= (1 << CS10) | (1 << CS11);    // 64 prescaler
	TCCR1B |= (1 << CS12);    // 256 prescaler
	TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
	interrupts();
}

typedef struct _pwm_state {
	int_bresenham br;
	byte state;
} t_pwm_state;

t_pwm_state pwm_state[14];

void setup() {
	// initialize the digital pin as an output.
	for (byte pcnt = 11; pcnt < 14; ++pcnt)
		pinMode(pcnt, OUTPUT);
	setup_timer1();
	for (int i = 0; i < 13; ++i) {
		pwm_state[i].br.ante = pwm_state[i].br.ante_err = pwm_state[i].br.conseq = pwm_state[i].br
				.conseq_err = 0;
		pwm_state[i].state = 0;
	}
//	Serial.begin(115200);
}

int tbl_sinus_scale = 255;
byte tbl_sinus[] = { 0, 4, 9, 13, 18, 22, 27, 31, 35, 40, 44, 49, 53, 57, 62, 66, 70, 75, 79, 83,
		87, 91, 96, 100, 104, 108, 112, 116, 120, 124, 127, 131, 135, 139, 143, 146, 150, 153, 157,
		160, 164, 167, 171, 174, 177, 180, 183, 186, 190, 192, 195, 198, 201, 204, 206, 209, 211,
		214, 216, 219, 221, 223, 225, 227, 229, 231, 233, 235, 236, 238, 240, 241, 243, 244, 245,
		246, 247, 248, 249, 250, 251, 252, 253, 253, 254, 254, 254, 255, 255, 255, 255 };

short irsinus(int grad) {
	short ret;
	int ag;
	grad %= 360;
	if (grad < 0)
		grad += 360;
	ag = grad % 180;
	if (ag > 90)
		ag = 180 - ag;
	ret = tbl_sinus[ag];
	if (grad > 180)
		ret = -ret;
	return ret;
}

/* 1.25 microsecs */
static inline void maskWriteB(byte set, byte mask) {
	byte state = PORTB;
	byte old_state = state;
	state &= ~mask;
	state |= set;
	if (old_state != state)
		PORTB = state;
}

int current_grad = 1;
unsigned long next_turn = 0;

void loop() {
	short pot_value = analogRead(A0);
	unsigned long time = millis();
	if (time > next_turn) {
		++current_grad;
		if (current_grad >= 360)
			current_grad -= 360;
		short cs = irsinus(current_grad);
		if (cs < 0)
			cs = -cs;
		pwm_state[11].br.ante = cs;
		pwm_state[11].br.conseq = tbl_sinus_scale - cs;
		cs = irsinus(current_grad + pot_value / 4);
		if (cs < 0)
			cs = -cs;
		pwm_state[12].br.ante = cs;
		pwm_state[12].br.conseq = tbl_sinus_scale - cs;
		cs = irsinus(current_grad + pot_value / 2);
		if (cs < 0)
			cs = -cs;
		pwm_state[13].br.ante = cs;
		pwm_state[13].br.conseq = tbl_sinus_scale - cs;
		next_turn = time + 10;
	}
}

ISR(TIMER1_COMPA_vect)          // timer compare interrupt service routine
{
	byte set = 0;
	for (byte pcnt = 11; pcnt < 14; ++pcnt) {
#define	bpin  (pcnt-8)
#define cpwm  pwm_state[pcnt]
		if (cpwm.state == 0) {
			cpwm.state = dither_bresenham(&cpwm.br);
		}
		if (cpwm.state & 1) {
			set |= (HIGH << bpin);
			--cpwm.state;
		} else if (cpwm.state & 2) {
			cpwm.state -= 2;
		}
	}
	maskWriteB(set, 0b00111000);
}

int main(void) {
	init();
	setup();
	//endless loop
	for (;;) {
		loop();
	}
	return 0;
}

