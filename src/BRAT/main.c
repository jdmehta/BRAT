/*
 * BRAT.c
 *
 * Created: 03-11-2017 09:18:42 AM
 * Author : JDM
 */ 

#include <avr/delay.h>
#include <avr/io.h>
#include <string.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

//Defines
#define F_CPU	16000000UL
#define TRUE 1
#define FALSE 0
#define NO_OF_MOTORS 8
//Servo pins-Array index map
#define LAT 0 //Left Ankle Transverse
#define LKV 1 //Left Knee Vertical
#define LHV 2 //Left Hip Vertical
#define LHT 3 //Left Hip Transverse
#define RAT 4 //Right Ankle Transverse
#define RKV 5 //Right Knee Vertical
#define RHV 6 //Right Hip Vertical
#define RHT 7 //Right Hip Transverse
//Rest Servo positions
#define LATInit 95
#define LKVInit 65
#define LHVInit 77
#define LHTInit 109
#define RATInit 110
#define RKVInit 68
#define RHVInit 99
#define RHTInit 110
//Angle movements
#define TDiff 20 //Transverse Angle Diff
#define TDiffS2 32 //Transverse Angle Diff Step 2
#define TDiffS3 37 //Transverse Angle Diff Step 3
#define VDiff 25 //Vertical Angle Diff
#define VDiffS2 35 //Vertical Angle Diff Step 2
#define VDiffS3 48 //Vertical Angle Diff Step 3
#define MSDiff 5 //Micro-Step Angle Diff

//Global Variables
int PosCurrent[NO_OF_MOTORS] = {0};
int PosNext[NO_OF_MOTORS] = {0};
int CurrentPWMOnT1[NO_OF_MOTORS] = {0};
int CurrentPWMOffT1[NO_OF_MOTORS] = {0};
int iTimerActionSelector = 0;
int iTimerActionBufferDelay = 60735;
int iError = 0;
int iDirection = 0;
int iDelayPerDegMs = 20;
int iDelayPerDegMsSlow = 40;
int iInterStepDelayMs = 500;
char bContinueWalking;
//Angle movements
int iTDiff;
int iTDiffS2;
int iTDiffS3;
int iTMSDiff;
int iVDiff;
int iVDiffS2;
int iVDiffS3;
int iVMSDiff;
	
//Pre-computed Timer Values (in EEPROM)
#pragma region TimerValues

PROGMEM const uint16_t t1h[] = {
	64135,
	64119,
	64104,
	64088,
	64073,
	64057,
	64042,
	64026,
	64011,
	63995,
	63979,
	63964,
	63948,
	63933,
	63917,
	63902,
	63886,
	63871,
	63855,
	63839,
	63824,
	63808,
	63793,
	63777,
	63762,
	63746,
	63731,
	63715,
	63699,
	63684,
	63668,
	63653,
	63637,
	63622,
	63606,
	63591,
	63575,
	63559,
	63544,
	63528,
	63513,
	63497,
	63482,
	63466,
	63451,
	63435,
	63419,
	63404,
	63388,
	63373,
	63357,
	63342,
	63326,
	63311,
	63295,
	63279,
	63264,
	63248,
	63233,
	63217,
	63202,
	63186,
	63171,
	63155,
	63139,
	63124,
	63108,
	63093,
	63077,
	63062,
	63046,
	63031,
	63015,
	62999,
	62984,
	62968,
	62953,
	62937,
	62922,
	62906,
	62891,
	62875,
	62859,
	62844,
	62828,
	62813,
	62797,
	62782,
	62766,
	62751,
	62735,
	62719,
	62704,
	62688,
	62673,
	62657,
	62642,
	62626,
	62611,
	62595,
	62579,
	62564,
	62548,
	62533,
	62517,
	62502,
	62486,
	62471,
	62455,
	62439,
	62424,
	62408,
	62393,
	62377,
	62362,
	62346,
	62331,
	62315,
	62299,
	62284,
	62268,
	62253,
	62237,
	62222,
	62206,
	62191,
	62175,
	62159,
	62144,
	62128,
	62113,
	62097,
	62082,
	62066,
	62051,
	62035,
	62019,
	62004,
	61988,
	61973,
	61957,
	61942,
	61926,
	61911,
	61895,
	61879,
	61864,
	61848,
	61833,
	61817,
	61802,
	61786,
	61771,
	61755,
	61739,
	61724,
	61708,
	61693,
	61677,
	61662,
	61646,
	61631,
	61615,
	61599,
	61584,
	61568,
	61553,
	61537,
	61522,
	61506,
	61491,
	61475,
	61459,
	61444,
	61428,
	61413,
	61397,
	61382,
	61366,
	61351,
	61335
};

PROGMEM const uint16_t t1l[] = {
	62735,
	62751,
	62766,
	62782,
	62797,
	62813,
	62828,
	62844,
	62859,
	62875,
	62891,
	62906,
	62922,
	62937,
	62953,
	62968,
	62984,
	62999,
	63015,
	63031,
	63046,
	63062,
	63077,
	63093,
	63108,
	63124,
	63139,
	63155,
	63171,
	63186,
	63202,
	63217,
	63233,
	63248,
	63264,
	63279,
	63295,
	63311,
	63326,
	63342,
	63357,
	63373,
	63388,
	63404,
	63419,
	63435,
	63451,
	63466,
	63482,
	63497,
	63513,
	63528,
	63544,
	63559,
	63575,
	63591,
	63606,
	63622,
	63637,
	63653,
	63668,
	63684,
	63699,
	63715,
	63731,
	63746,
	63762,
	63777,
	63793,
	63808,
	63824,
	63839,
	63855,
	63871,
	63886,
	63902,
	63917,
	63933,
	63948,
	63964,
	63979,
	63995,
	64011,
	64026,
	64042,
	64057,
	64073,
	64088,
	64104,
	64119,
	64135,
	64151,
	64166,
	64182,
	64197,
	64213,
	64228,
	64244,
	64259,
	64275,
	64291,
	64306,
	64322,
	64337,
	64353,
	64368,
	64384,
	64399,
	64415,
	64431,
	64446,
	64462,
	64477,
	64493,
	64508,
	64524,
	64539,
	64555,
	64571,
	64586,
	64602,
	64617,
	64633,
	64648,
	64664,
	64679,
	64695,
	64711,
	64726,
	64742,
	64757,
	64773,
	64788,
	64804,
	64819,
	64835,
	64851,
	64866,
	64882,
	64897,
	64913,
	64928,
	64944,
	64959,
	64975,
	64991,
	65006,
	65022,
	65037,
	65053,
	65068,
	65084,
	65099,
	65115,
	65131,
	65146,
	65162,
	65177,
	65193,
	65208,
	65224,
	65239,
	65255,
	65271,
	65286,
	65302,
	65317,
	65333,
	65348,
	65364,
	65379,
	65395,
	65411,
	65426,
	65442,
	65457,
	65473,
	65488,
	65504,
	65519,
	65535
};

#pragma endregion TimerValues

int main(void)
{
	//Set input and output ports
	DDRC = 0xFF;
	DDRA = 0xFF;
	DDRB = 0xFF;
	PORTC = 0x00;
	
	//Write INIT angle values
	PosNext[LAT] = LATInit;
	PosNext[LKV] = LKVInit;
	PosNext[LHV] = LHVInit;
	PosNext[LHT] = LHTInit;
	PosNext[RAT] = RATInit;
	PosNext[RKV] = RKVInit;
	PosNext[RHV] = RHVInit;
	PosNext[RHT] = RHTInit;
	PerformServoMotionConstSpeed(0);
	
	//Include logic to decide direction (FOrward or Backward)
	iDirection = 1; //1 = Forward; -1 = Backward
	iTDiff = TDiff;
	iTDiffS2 = TDiffS2;
	iTDiffS3 = TDiffS3;
	iTMSDiff = MSDiff;
	iVDiff = VDiff * iDirection;
	iVDiffS2 = VDiffS2 * iDirection;
	iVDiffS3 = VDiffS3 * iDirection;
	iVMSDiff = MSDiff * iDirection;
	
	
	//This provision is given in order to enable stopping the robot
	//This could be because of sensor / control inputs
	bContinueWalking = TRUE;
	
	_delay_ms(1000);
	
	Timer1Init();
	TCNT1 = (60000);
	
	sei(); //Enable interrupts
	
	Walk();
}

void Walk()
{
	//Stepf.i.1
	//Make parallelogram by transferring weight on right leg
	//Motors involved: rat, rht, lat, lht
	PosNext[RAT] = RATInit - iTDiffS2;
	PosNext[RHT] = RHTInit - iTDiff;
	PosNext[LAT] = LATInit - iTDiffS2;
	PosNext[LHT] = LHTInit - iTDiff;
	PerformServoMotionConstSpeed(iDelayPerDegMs);
	DelayMillis(iInterStepDelayMs);
	
	//Stepf.i.2
	//Move left leg away from body
	//Motors involved: lat, lht
	PosNext[LAT] = LATInit - iTDiffS3;
	PosNext[LHT] = LHTInit - iTDiffS3;
	PerformServoMotionConstSpeed(iDelayPerDegMs);
	DelayMillis(iInterStepDelayMs);
	
	while(bContinueWalking == TRUE)
	{
		//Stepf.1.1
		//Lift left leg
		//Motors involved: lkv, lhv
		PosNext[LKV] = PosCurrent[LKV] + iVMSDiff;
		PerformServoMotionConstSpeed(iDelayPerDegMsSlow);
		PosNext[LHV] = LHVInit - iVDiffS3;
		PosNext[LKV] = LKVInit + iVDiffS2;
		PerformServoMotionConstSpeed(iDelayPerDegMs);
		DelayMillis(iInterStepDelayMs);
		
		//Step f.1.2
		//Bend right leg
		//Motors involved: rhv, rkv
		PosNext[RHV] = RHVInit - iVDiff;
		PosNext[RKV] = RKVInit + iVDiff;
		PerformServoMotionConstSpeed(iDelayPerDegMsSlow);
		
		//Step f.1.3
		//Bring left leg closer to body
		//Motors involved: lat, lht
		PosNext[LHT] = LHTInit - iTDiff;
		PerformServoMotionConstSpeed(iDelayPerDegMs);
		DelayMillis(iInterStepDelayMs);
		
		//Step f.1.4
		//Lower left leg
		//Motors involved: lhv, lkv, rat
		PosNext[LHV] = LHVInit - iVDiff;
		PosNext[LKV] = LKVInit + iVDiff;
		PosNext[RAT] = RATInit - iTDiff;
		PosNext[LAT] = LATInit - iTDiff;
		PerformServoMotionConstSpeed(iDelayPerDegMsSlow);
		DelayMillis(iInterStepDelayMs);
		
		//Stepf.1.5
		//Make parallelogram by transferring weight on left leg
		//Motors involved: rat, rht, lat, lht
		PosNext[RAT] = RATInit + iTDiff;
		PosNext[RHT] = RHTInit + iTDiff;
		PosNext[LAT] = LATInit + iTDiff;
		PosNext[LHT] = LHTInit + iTDiff;
		PerformServoMotionConstSpeed(iDelayPerDegMs);
		DelayMillis(iInterStepDelayMs);
		
		//Stepf.1.6
		//Straighten left leg
		//Motors involved: lat, rat, lhv, lkv
		PosNext[LAT] = LATInit + iTDiffS2;
		PosNext[LHV] = LHVInit;
		PosNext[LKV] = LKVInit;
		PosNext[RHT] = RHTInit + iTDiffS2;
		PerformServoMotionConstSpeed(iDelayPerDegMs);
		DelayMillis(iInterStepDelayMs);
		
		//Step f.1.7
		//Move right leg away from body and straighten right leg
		//Motors involved: rat, rht, rkv, rhv
		PosNext[RAT] = RATInit + iTDiffS3;
		PosNext[RHT] = RHTInit + iTDiffS3;
		PosNext[RKV] = RKVInit;
		PosNext[RHV] = RHVInit;
		PerformServoMotionConstSpeed(iDelayPerDegMsSlow);
		DelayMillis(iInterStepDelayMs);
		
		//Step f.2.1
		//Lift right leg
		//Motors involved: rhv, rkv
		PosNext[RHV] = PosCurrent[RHV] + iVMSDiff;
		PerformServoMotionConstSpeed(iDelayPerDegMsSlow);
		PosNext[RKV] = RKVInit - iVDiffS2; //May have to sub ~10 deg
		PosNext[RHV] = RHVInit + iVDiffS3; //May have to add ~7 deg
		PerformServoMotionConstSpeed(iDelayPerDegMsSlow);
		DelayMillis(iInterStepDelayMs);
		
		//Step f.2.2
		//Bend left leg
		//Motors involved: lhv, lkv
		PosNext[LKV] = LKVInit - iVDiff; //May have to add ~5 deg
		PosNext[LHV] = LHVInit + iVDiff; //May have to sub ~5 deg
		PerformServoMotionConstSpeed(iDelayPerDegMsSlow);
		DelayMillis(iInterStepDelayMs);
		
		//Step f.2.3
		//Bring right leg closer to body
		//Motors involved: rat, rht
		PosNext[RHT] = = RHTInit + iTDiff;
		PerformServoMotionConstSpeed(iDelayPerDegMs);
		DelayMillis(iInterStepDelayMs);
		
		//Step f.2.4
		//Lower right leg
		//Motors involved: rhv, rkv, lat
		PosNext[RKV] = RKVInit - iVDiff; //May have to sub ~7 deg
		PosNext[RHV] = RHVInit + iVDiff; //May have to add ~5 deg
		PerformServoMotionConstSpeed(iDelayPerDegMsSlow);
		DelayMillis(iInterStepDelayMs);
		
		//Step f.2.5
		//Make parallelogram by transferring weight on right leg
		//Motors involved: rat, rht, lat, lht
		PosNext[RAT] = RATInit - iTDiff; //May have to sub ~2 deg
		PosNext[RHT] = RHTInit - iTDiff; //May have to sub ~2 deg
		PosNext[LAT] = LATInit - iTDiff; //May have to sub ~2 deg
		PosNext[LHT] = LHTInit - iTDiff; //May have to sub ~2 deg
		PerformServoMotionConstSpeed(iDelayPerDegMs);
		DelayMillis(iInterStepDelayMs);
		
		//Step f.2.6
		//Straighten right leg
		//Motors involved: rat, rhv, rkv
		PosNext[RAT] = RATInit - iTDiffS2; //May have to sub ~2 deg
		PosNext[RKV] = RKVInit;
		PosNext[RHT] = RHTInit;
		PerformServoMotionConstSpeed(iDelayPerDegMsSlow);
		DelayMillis(iInterStepDelayMs);
		
		//Step f.2.7
		//Move left leg away from body and straighten left leg
		//Motors involved: lat, lht, lkv, lhv
		PosNext[LAT] = LATInit - iTDiffS3;
		PosNext[LHT] = LHTInit - iTDiffS3;
		PosNext[LKV] = LKVInit;
		PosNext[LHV] = LHVInit;
		PerformServoMotionConstSpeed(iDelayPerDegMs);
		DelayMillis(iInterStepDelayMs);
	}
	
	//Include logic to move back to rest position when bContinueWalking becomes FALSE
	
	return;
}

void Timer1Init(){
	TCCR1A = (0<<WGM11)|(0<<WGM10);
	TCCR1B = (0<<WGM13)|(0<<WGM12)|(0<<CS12)|(1<<CS11)|(0<<CS10);
	TIMSK|= (1<<TOIE1);
	return;
}

//The _delay_ms function expects compile time integer constant
//DelayMillis is workaround to have dynamic delay
//There would be some inaccuracy as _delay_ms is CPU based delay - loop delay will be added
void DelayMillis(int iDelayMillis)
{
	for(int i=0; i < iDelayMillis; i++)
	{
		_delay_ms(1);
	}
	return;
}

//Writes Timer high and low values to array for CURRENT position
void ServoWrite(){
	for(int i=0; i < NO_OF_MOTORS; i++)
	{
		CurrentPWMOnT1[i] = pgm_read_word_near(t1h + PosCurrent[i]);
		CurrentPWMOffT1[i] = pgm_read_word_near(t1l + PosCurrent[i]);
	}
	return;
}

//Writes current required servo angle to array
//This is a blocking function, will return after completing motion
void PerformServoMotionConstSpeed(int iDelayMsPerDeg)
{
	char bMotionCompleted;
	do 
	{
		bMotionCompleted = TRUE;
		for(int i=0; i < NO_OF_MOTORS; i++)
		{
			if(PosCurrent[i] < PosNext[i])
			{
				PosCurrent[i]++;
				bMotionCompleted = FALSE;
			}
			else if(PosCurrent[i] > PosNext[i])
			{
				PosCurrent[i]--;
				bMotionCompleted = FALSE;
			}
		}
		ServoWrite();
		if(iDelayMsPerDeg > 0)
		{
			DelayMillis(iDelayMsPerDeg);
		}
	} while (bMotionCompleted == FALSE);
	return;
}

//Generates PWM output for servo motors
ISR (TIMER1_OVF_vect){
	switch(iTimerActionSelector){
		//LAT
		case 0:
		TCNT1 = (CurrentPWMOnT1[LAT]);
		PORTC = (1 << LAT);
		iTimerActionSelector = 1;
		break;
		case 1:
		TCNT1 = (CurrentPWMOffT1[LAT]);
		PORTC ^= (1 << LAT);
		iTimerActionSelector = 2;
		break;
		//LKV
		case 2:
		TCNT1 = (CurrentPWMOnT1[LKV]);
		PORTC = (1 << LKV);
		iTimerActionSelector = 3;
		break;
		case 3:
		TCNT1 = (CurrentPWMOffT1[LKV]);
		PORTC ^= (1 << LKV);
		iTimerActionSelector = 4;
		break;
		//LHV
		case 4:
		TCNT1 = (CurrentPWMOnT1[LHV]);
		PORTC = (1 << LHV);
		iTimerActionSelector = 5;
		break;
		case 5:
		TCNT1 = (CurrentPWMOffT1[LHV]);
		PORTC ^= (1 << LHV);
		iTimerActionSelector = 6;
		break;
		//LHT
		case 6:
		TCNT1 = (CurrentPWMOnT1[LHT]);
		PORTC = (1 << LHT);
		iTimerActionSelector = 7;
		break;
		case 7:
		TCNT1 = (CurrentPWMOffT1[LHT]);
		PORTC ^= (1 << 3);
		iTimerActionSelector = LHT;
		break;
		//RAT
		case 8:
		TCNT1 = (CurrentPWMOnT1[RAT]);
		PORTC = (1 << RAT);
		iTimerActionSelector = 9;
		break;
		case 9:
		TCNT1 = (CurrentPWMOffT1[RAT]);
		PORTC ^= (1 << RAT);
		iTimerActionSelector = 10;
		break;
		//RKV
		case 10:
		TCNT1 = (CurrentPWMOnT1[RKV]);
		PORTC = (1 << RKV);
		iTimerActionSelector = 11;
		break;
		case 11:
		TCNT1 = (CurrentPWMOffT1[RKV]);
		PORTC ^= (1 << RKV);
		iTimerActionSelector = 12;
		break;
		//RHV
		case 12:
		TCNT1 = (CurrentPWMOnT1[RHV]);
		PORTC = (1 << RHV);
		iTimerActionSelector = 13;
		break;
		case 13:
		TCNT1 = (CurrentPWMOffT1[RHV]);
		PORTC ^= (1 << RHV);
		iTimerActionSelector = 14;
		break;
		//RHT
		case 14:
		TCNT1 = (CurrentPWMOnT1[RHT]);
		PORTC = (1 << RHT);
		iTimerActionSelector = 15;
		break;
		case 15:
		TCNT1 = (CurrentPWMOffT1[RHT]);
		PORTC ^= (1 << RHT);
		iTimerActionSelector = 16;
		break;
		
		case 16:
		TCNT1 = (iTimerActionBufferDelay);
		PORTC = 0x00;
		iTimerActionSelector = 0;
		break;
		default:
		iTimerActionSelector = 0;
		iError = 1;
		break;
	}
}