namespace selector{

#define HUE 0 // hue from 0-360
#define DEFAULT_AUTON 1 // auton that starts selected
#define USING_CONTROLLER false // true/false if a controller can select auton
#define CONTROLLER_BUTTON ButtonUp // the button that cycles through the options

extern int auton;

void init();

}