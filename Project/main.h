#ifndef MAIN_H_
#define MAIN_H_



void system_clock_init(void);
void Safety_first(void);

extern int helios_stop;

enum Stats{INIT, IDLE, PARK, CONTROL, RESeT, STOP, EMERCENCY_STOP};  //RESeT is written like that, because Coocox defines ITStatus values {SET,RESET}
enum Stats Ballbot_stats;
// Mode control
enum control{C_POSITION, C_VELOCITY, C_ACCELERATION, C_STOP, C_FREEZE, C_ROTONDO, C_HULL_POSITION, C_ACCELERATION_WZ, C_MODE_4,
			 C_MODE_5, C_MODE_6, C_MODE_7, C_MODE_8, C_MODE_9, C_MODE_10, C_MODE_11};
enum control Control_Mode;

#endif /* MAIN_H_ */
