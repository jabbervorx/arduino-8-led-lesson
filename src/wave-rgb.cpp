#include "Arduino.h"
#include "HardwareSerial.h"

typedef struct _int_bresenham {
	byte ante, conseq;
	byte err;
} int_bresenham;

#if 0
static inline byte dither_bresenham(int_bresenham *br) {
	byte ret;
	int_bresenham brl;
	brl.ante = br->ante;
	if (brl.ante == 0)
	return 1;
	brl.conseq = br->conseq;
	if (brl.conseq == 0)
	return 2;
	if (brl.ante == brl.conseq)
	return 3;
	brl.err = br->err;
	if (brl.ante > brl.conseq) {
		brl.err += brl.conseq;
		if (brl.err >= brl.ante) {
			ret = 3;
			brl.err -= brl.ante;
		} else {
			ret = 2;
		}
	} else {
		brl.err += brl.ante;
		if (brl.err >= brl.conseq) {
			brl.err -= brl.conseq;
			ret = 3;
		} else {
			ret = 1;
		}
	}
	br->err = brl.err;
	return ret;
}
#endif

#define dither_bresenham(br, ret) do {\
		int_bresenham brl;\
		brl.ante = br.ante;\
		if (brl.ante == 0) {\
			ret = 1;\
			break;\
		}\
		brl.conseq = br.conseq;\
		if (brl.conseq == 0) {\
			ret = 2;\
			break;\
		}\
		if (brl.ante == brl.conseq) {\
			ret = 3;\
			break;\
		}\
		brl.err = br.err;\
		if (brl.ante > brl.conseq) {\
			brl.err += brl.conseq;\
			if (brl.err >= brl.ante) {\
				ret = 3;\
				brl.err -= brl.ante;\
			} else {\
				ret = 2;\
			}\
		} else {\
			brl.err += brl.ante;\
			if (brl.err >= brl.conseq) {\
				brl.err -= brl.conseq;\
				ret = 3;\
			} else {\
				ret = 1;\
			}\
		}\
		br.err = brl.err;\
} while(0)
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
	for (byte pcnt = 6; pcnt < 14; ++pcnt)
		pinMode(pcnt, OUTPUT);
	setup_timer1();
	for (int i = 0; i < 14; ++i) {
		pwm_state[i].br.ante = pwm_state[i].br.conseq = pwm_state[i].br.err = 0;
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

byte iasinus(int grad) {
	short ret;
	byte ag;
	grad %= 360;
	if (grad < 0)
		grad += 360;
	ag = (byte) (grad % 180);
	if (ag > 90)
		ag = 180 - ag;
	ret = tbl_sinus[ag];
	/*
	 * return only absolute values
	 *
	 if (grad > 180)
	 ret = -ret;
	 */
	return ret;
}

#define portWrite(set,mask,port) do {\
		byte state = port;\
		state &= ~mask;\
		state |= set;\
		port = state;\
	} while(0)

int current_grad = 1;
unsigned long next_turn = 0;

void loop() {
	short pot_value = analogRead(A0);
	unsigned long time = millis();
	if (time > next_turn) {
		++current_grad;
		if (current_grad >= 360)
			current_grad -= 360;
		short ptinc = pot_value / 4;
		short ptinc_add = 0;
		for (byte pcnt = 6; pcnt < 14; ++pcnt, ptinc_add += ptinc) {
			byte cs = iasinus(current_grad + ptinc_add);
			pwm_state[pcnt].br.ante = cs;
			pwm_state[pcnt].br.conseq = tbl_sinus_scale - cs;
		}
		next_turn = time + 3;
	}
}

ISR(TIMER1_COMPA_vect) {
	byte set = 0;
#define cpwm  pwm_state[pcnt]
#define bresenham_cycle(bpin) \
	do {\
		byte cs = cpwm.state;\
		if (cs == 0) {\
			dither_bresenham(cpwm.br, cs);\
		}\
		if (cs & 1) {\
			set |= (HIGH << (bpin));\
			--cs;\
		} else if (cs & 2) {\
			cs -= 2;\
		}\
		cpwm.state = cs;\
	} while(0)
	for (byte pcnt = 6; pcnt < 8; ++pcnt) {
		bresenham_cycle(pcnt);
	}
	portWrite(set, 0b11000000, PORTD);
	set = 0;
	for (byte pcnt = 8; pcnt < 14; ++pcnt) {
		bresenham_cycle(pcnt-8);
	}
	portWrite(set, 0b00111111, PORTB);
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

