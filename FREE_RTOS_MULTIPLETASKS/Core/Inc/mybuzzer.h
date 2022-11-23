#ifndef INC_MYBUZZER_H_
#define INC_MYBUZZER_H_
#include "main.h"


typedef struct _BUZZER{
	int *song;
	int *frames;
}BUZZER;

typedef enum  _NOTES{
   NOTE_1 = 381,
   NOTE_2 = 340,
   NOTE_3 = 322,
   NOTE_4 = 286,
   NOTE_5 = 255,
   NOTE_6 = 227,
   NOTE_7 = 202,

}NOTES;
typedef enum _FRAME{
	FRAME_0 = 1000,
	FRAME_1 = 500,
	FRAME_2 = 255
}FRAME;

void buzzerStartPlay(BUZZER* buzzer,int* song, int* frames);




#endif /* INC_MYBUZZER_H_ */
