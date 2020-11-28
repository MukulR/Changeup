#include "main.h"



/**
 * Autonomous name documentation:
 * P-O-M: Pickup ball from platform, shoot our flags, 
 * 			pickup two balls from cap, shoot middle pole flags.
 * 
 * U-O-M: Pickup ball from under the cap, shoot our flags,
 * 			pickup two balls from cap, shoot middle pole flags.
 * 
 * Back: Shoot middle top flag using preload, pick up ball from under the cap,
 *          shoot opponent's top flag, climb on platform
 */
 
static const char *btnm_map[] = {"Skills", "Normal Auton", "\n", ""};
static const char *auton_names[] = {"Skills", "Normal Auton"};
static const char *alliance_map[] = {"Red", "Blue", ""};

int autonSelected = 2;  // by default no auton
bool redAlliance = false;

pros::Imu *inertial_sensor;
pros::Vision *vision_sensor;
pros::Optical *optical_sensor;

static lv_res_t btnm_action(lv_obj_t *btnm, const char *txt) {
   printf("btnm_action Called \n");
   for (int i = 0; i < sizeof(auton_names) / sizeof(auton_names[0]); i++) {
      printf("%s\n", auton_names[i]);
      printf("%s\n", txt);
      printf("-----------\n");
      if (strcmp(auton_names[i], txt) == 0) {
         autonSelected = i;
         break;
      }
      lv_btnm_set_toggle(btnm, true, autonSelected);
   }

   return LV_RES_OK; /*Return OK because the button matrix is not deleted*/
}

static lv_res_t btnm_action_color(lv_obj_t *btnm, const char *txt) {
   lv_btnm_set_toggle(btnm, true, 1);
   lv_btnm_set_toggle(btnm, true, 2);
   printf("btnm_action_color Called\n");
   if (strcmp(txt, "Red") == 0) {
      	printf("Red alliance\n");
    	redAlliance = true;
   }
   else if (strcmp(txt, "Blue") == 0) {
      	printf("Blue alliance\n");
		redAlliance = false;
   }

   return LV_RES_OK; /*Return OK because the button matrix is not deleted*/
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
   std::cout << "Initializing IMU" << std::endl;
   // Initialize and calibrate imu/gyro here.
   inertial_sensor = new pros::Imu(11);
   inertial_sensor->reset();

   vision_sensor = new pros::Vision(18);
   vision_sensor->clear_led();

   optical_sensor = new pros::Optical(6);

   pros::Task::delay(3000);
   std::cout << "Finished initializing IMU" << std::endl;
   std::cout << inertial_sensor->get_heading() << std::endl;
  

	lv_theme_alien_init(0, NULL);
	lv_obj_t *title = lv_label_create(lv_scr_act(), NULL);
	lv_label_set_text(title, "Auton Selection");
	lv_obj_align(title, NULL, LV_ALIGN_IN_TOP_MID, 0, 10);

	lv_obj_t *btnm = lv_btnm_create(lv_scr_act(), NULL);
	lv_btnm_set_map(btnm, btnm_map);
	lv_btnm_set_action(btnm, btnm_action);
	lv_obj_set_size(btnm, LV_HOR_RES - 40, LV_VER_RES / 3);
	lv_obj_align(btnm, title, LV_ALIGN_OUT_BOTTOM_MID, 0, 20);

	lv_obj_t *allianceM = lv_btnm_create(lv_scr_act(), NULL);
	lv_btnm_set_map(allianceM, alliance_map);
	lv_btnm_set_action(allianceM, btnm_action_color);
	lv_obj_set_size(allianceM, LV_HOR_RES - 40, 50);
	lv_obj_align(allianceM, btnm, LV_ALIGN_OUT_BOTTOM_MID, 0, 20);
}