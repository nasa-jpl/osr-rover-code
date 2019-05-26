#ifndef OsrScreen
#define OsrScreen

#include "../RGB_matrix_Panel/RGBmatrixPanel.h"


#define BIT_MAX  				15
#define RED 					BIT_MAX,0,0
#define GREEN 					0,BIT_MAX,0
#define BLUE					0,0,BIT_MAX
#define BLACK					0,0,0
#define WHITE					BIT_MAX,BIT_MAX,BIT_MAX

#define YELLOW_GREEN 			7,BIT_MAX,0
#define YELLOW					BIT_MAX,BIT_MAX,0
#define YELLOW_RED				BIT_MAX,7,0

#define PREAMBLE_HIGH 			0xAB
#define PREAMBLE_LOW  			0xCD

#define CONNECTED_POS			2
#define BATTERY_POS				3
#define STATUS_POS				4
#define TEMP_POS				5
#define DRIVE_POS				8
#define STEER_POS				11
#define FACE_POS				13
#define CHKSUM_POS				14
#define MAX_MILLIS_TO_WAIT		10000
#define RUNNING					0x00
#define IDLE					0x01


class Screen: public RGBmatrixPanel {
	int timeout;
	int l_eye[2];
	int r_eye[2];
	int mouth[2];
	int l_cheek[2];
	int r_cheek[2];
	int l_brow[2];
	int r_brow[2];
	int* data = new int[16];

    int TEST_MODE			= 0;
    int state				= IDLE;
    int BATTERY[2] 			= {0,0};
    int CONNECT[2] 			= {0,1};
    int STATUS[2]  			= {6,0};
    int TEMP[2]    			= {12,0};
    int DRIVE_CURRENT[2] 	= {19,0};
    int STEER_CURRENT[2] 	= {26,0};
    int DEBUG[2]         	= {0,8};
    int STATE[2]          	= {1,1};

public:
    
    unsigned long time;

    Screen(int);
    void init_display();
    void init_comm();
    void testpixel(int);
    int * get_data();
    void connected_status(int);
    int preamble_check(int[]);
    int chksum_check(int[]);
    void affirm_message();
    int get_state();

    void display_state();
    void display_battery(int);
    int* get_color(int);
    void display_face(int);
    void display_temp(int[]);
    void display_status(int);
    void display_currents(int[], int[]);

    void happy_eye(int, int, int[]);
    void sleepy_eye(int, int, int[]);
    void happy_mouth(int, int, int[]);
    void circle_cheek(int,int,int[]);
    void cute_mouth(int,int,int[]);
    void cute_cheeks(int,int,int[]);
    void eight_bit_eye(int,int);
    void eight_bit_face();
    void happy_face();
    void sleepy_face();
    void clear_face();

    void update_screen(int[]);

};
#endif




