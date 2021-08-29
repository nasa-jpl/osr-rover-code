#include "OsrScreen.h"

#include "Arduino.h"
#include "../RGB_matrix_Panel/RGBmatrixPanel.h"



Screen::Screen(int t) : RGBmatrixPanel(14,15,16,8,10,9,false)
{
	timeout = t;
}

void Screen::init_display()
{
	RGBmatrixPanel::begin();

	int temp[]  = {0xFF,0xFF,0xFF};
	int drive[] = {0xFF,0xFF,0xFF};
	int steer[] = {0xFF,0xFF,0xFF};
	
	display_state();
	connected_status(0x0);
	display_battery(0xFF);
	display_status(0xFF);
	display_temp(temp);
	display_currents(drive,steer);
	sleepy_face();
	
	RGBmatrixPanel::begin();
	Serial.flush();
}

void Screen::init_comm()
{
	while (1) {
	  if (Serial.read() == 0xA){
	    RGBmatrixPanel::drawPixel(DEBUG[0]+1,DEBUG[1], RGBmatrixPanel::Color444(BLUE));
	    Serial.print(205,HEX);
	    break;
	  }
	  RGBmatrixPanel::drawPixel(DEBUG[0]+1,DEBUG[1], RGBmatrixPanel::Color444(WHITE));
	}
}

void Screen::testpixel(int c)
{
	RGBmatrixPanel::drawPixel(1,1,RGBmatrixPanel::Color444(c,c,c));
}


int Screen::get_state()
{
	return state;
}

int* Screen::get_data()
{	
	if (state)
	{
		RGBmatrixPanel::drawPixel(30,5,RGBmatrixPanel::Color444(5,5,5));
		Screen::init_comm();
		state = RUNNING;
	}
	else 
	{
		RGBmatrixPanel::drawPixel(30,5,RGBmatrixPanel::Color444(BLACK));
		time = millis();
		//while ( (Serial.available()<16) && ((millis() - time) < MAX_MILLIS_TO_WAIT) )
		while ( Serial.available() < 16)
		{
			//do nothing  
		}
		if (Serial.available() > 0)
		{
			for(int n=0; n<16; n++)
			{
				data[n] = Serial.read();
			}
			if (data[CONNECTED_POS] == 0xFF)
			{
				state = IDLE;
			}
			return data;
		}
	}
	return 0;
}


void Screen::display_state(){
  if (state == RUNNING){
    RGBmatrixPanel::drawPixel(STATE[0], STATE[1], RGBmatrixPanel::Color444(GREEN));        
  }
  else if (state == IDLE){
    RGBmatrixPanel::drawPixel(STATE[0], STATE[1], RGBmatrixPanel::Color444(RED));
  } else if (state == 0x00){
    RGBmatrixPanel::drawPixel(STATE[0], STATE[1], RGBmatrixPanel::Color444(WHITE));
  }
}


void Screen::connected_status(int status){
  if (status) {
    RGBmatrixPanel::drawPixel(CONNECT[0], CONNECT[1], RGBmatrixPanel::Color444(GREEN));
  }
  else {
    RGBmatrixPanel::drawPixel(CONNECT[0], CONNECT[1], RGBmatrixPanel::Color444(RED));
  }
}

int Screen::preamble_check(int message[]){
  if (message[0] == PREAMBLE_HIGH && message[1] == PREAMBLE_LOW){
    RGBmatrixPanel::drawPixel(DEBUG[0],DEBUG[1], RGBmatrixPanel::Color444(BLACK));
    return 1;
  }
  else {
    RGBmatrixPanel::drawPixel(DEBUG[0],DEBUG[1], RGBmatrixPanel::Color444(BLUE));
    return 0;
  }
}


int Screen::chksum_check(int message[]){
  int chksum = 0;
  for (int i=2; i < 14; i++){
    chksum += message[i];
  }
  int low = chksum & 0x00FF;
  int high = (chksum & 0xFF00) >> 8;
  if ((high == message[CHKSUM_POS]) && (low == message[CHKSUM_POS +1])){
    RGBmatrixPanel::drawPixel(DEBUG[0],DEBUG[1], RGBmatrixPanel::Color444(BLACK));
    Screen::affirm_message();
    return 1;
  }
  RGBmatrixPanel::drawPixel(DEBUG[0],DEBUG[1], RGBmatrixPanel::Color444(RED));
  return 0;
}

void Screen::affirm_message()
{
	Serial.print(205,HEX);
}

void Screen::display_battery(int battery_level){
  RGBmatrixPanel::drawLine(BATTERY[0],BATTERY[1],BATTERY[0]+4,BATTERY[1],RGBmatrixPanel::Color444(BLACK));
  if (battery_level == 0xFF){
    RGBmatrixPanel::drawLine(BATTERY[0],BATTERY[1],BATTERY[0]+4,BATTERY[1],RGBmatrixPanel::Color444(WHITE));    
  }
  else{
    if (battery_level & 0x10){
      RGBmatrixPanel::drawPixel(BATTERY[0]+4,BATTERY[1], RGBmatrixPanel::Color444(GREEN));
    }
    if (battery_level & 0x08){
      RGBmatrixPanel::drawPixel(BATTERY[0]+3,BATTERY[1], RGBmatrixPanel::Color444(YELLOW_GREEN));
    }
    if (battery_level & 0x04){
      RGBmatrixPanel::drawPixel(BATTERY[0]+2,BATTERY[1], RGBmatrixPanel::Color444(YELLOW));
    }
    if (battery_level & 0x02){
      RGBmatrixPanel::drawPixel(BATTERY[0]+1,BATTERY[1], RGBmatrixPanel::Color444(YELLOW_RED));
    }
    if (battery_level & 0x01){
      RGBmatrixPanel::drawPixel(BATTERY[0],BATTERY[1], RGBmatrixPanel::Color444(RED));
    }   
  }
}


int* Screen::get_color(int num){
  int* color = new int[3];
  if (num == 0x04){
    color[0] = BIT_MAX;
    color[1] = 0;
    color[2] = 0;
  }
  else if (num == 0x03){
    color[0] = BIT_MAX;
    color[1] = 7;
    color[2] = 0;
  }
  else if (num == 0x02){
    color[0] = BIT_MAX;
    color[1] = BIT_MAX;
    color[2] = 0;
  }
  else if (num == 0x01){
    color[0] = 7;
    color[1] = BIT_MAX;
    color[2] = 0;
  }
  else if (num == 0x00){
    color[0] = 0;
    color[1] = BIT_MAX;
    color[2] = 0;
  }
  else {
    color[0] = BIT_MAX;
    color[1] = BIT_MAX;
    color[2] = BIT_MAX;
  }
  return color;
}

void Screen::display_face(int face){
  if (face == 0x01){
    eight_bit_face();
  }
  else if (face == 0x00){
    happy_face();
  }
}

void Screen::display_temp(int temp[]){
  int* color;
  for (int i =0; i < 5; i ++){
    color = get_color(temp[i]);
    RGBmatrixPanel::drawPixel(TEMP[0]+i,TEMP[1],RGBmatrixPanel::Color444(color[0],color[1],color[2]));
  }
}

void Screen::display_status(int error_status){
  RGBmatrixPanel::drawLine(STATUS[0],STATUS[1],STATUS[0]+4,STATUS[1],RGBmatrixPanel::Color444(GREEN));
  if (error_status == 0xFF){
    RGBmatrixPanel::drawLine(STATUS[0],STATUS[1],STATUS[0]+4,STATUS[1],RGBmatrixPanel::Color444(WHITE));
  }
  else{
    if (error_status & 0x01){
      RGBmatrixPanel::drawPixel(STATUS[0],STATUS[1],RGBmatrixPanel::Color444(RED));
    }
    if (error_status & 0x02){
      RGBmatrixPanel::drawPixel(STATUS[0]+1,STATUS[1],RGBmatrixPanel::Color444(RED));
    }
    if (error_status & 0x04){
      RGBmatrixPanel::drawPixel(STATUS[0]+2,STATUS[1],RGBmatrixPanel::Color444(RED));
    }
    if (error_status & 0x08){
      RGBmatrixPanel::drawPixel(STATUS[0]+3,STATUS[1],RGBmatrixPanel::Color444(RED));
    }
    if (error_status & 0x10){
      RGBmatrixPanel::drawPixel(STATUS[0]+4,STATUS[1],RGBmatrixPanel::Color444(RED));
    }         
  }
}


void Screen::display_currents(int drive_current[],int steering_current[]){
  int* color;
  for (int i =0; i < 6; i ++){
    color = get_color(drive_current[i]);
    RGBmatrixPanel::drawPixel(DRIVE_CURRENT[0]+i,DRIVE_CURRENT[1],RGBmatrixPanel::Color444(color[0],color[1],color[2]));
  }

  for (int i =0; i < 4; i ++){
    color = get_color(steering_current[i]);
    RGBmatrixPanel::drawPixel(STEER_CURRENT[0]+i,STEER_CURRENT[1],RGBmatrixPanel::Color444(color[0],color[1],color[2]));
  }
}




void Screen::happy_eye(int x, int y, int rgb[]){
  RGBmatrixPanel::drawLine(x-2,y,x,y-2,RGBmatrixPanel::Color444(rgb[0],rgb[1],rgb[2]));
  RGBmatrixPanel::drawLine(x+1,y-2,x+3,y,RGBmatrixPanel::Color444(rgb[0],rgb[1],rgb[2]));
  
}


void Screen::sleepy_eye(int x, int y, int rgb[]){
  RGBmatrixPanel::drawLine(x-2,y,x+2,y,RGBmatrixPanel::Color444(rgb[0],rgb[1],rgb[2]));
}

void Screen::happy_mouth(int x, int y, int rgb[]){
  RGBmatrixPanel::drawLine(x-4,y,x-2,y+2,RGBmatrixPanel::Color444(rgb[0],rgb[1],rgb[2]));
  RGBmatrixPanel::drawLine(x-1,y+2,x+2,y+2,RGBmatrixPanel::Color444(rgb[0],rgb[1],rgb[2]));
  RGBmatrixPanel::drawLine(x+3,y+2,x+5,y,RGBmatrixPanel::Color444(rgb[0],rgb[1],rgb[2]));
  RGBmatrixPanel::drawPixel(x-4,y-1,RGBmatrixPanel::Color444(rgb[0],rgb[1],rgb[2]));
  RGBmatrixPanel::drawPixel(x+5,y-1,RGBmatrixPanel::Color444(rgb[0],rgb[1],rgb[2]));
}

void Screen::circle_cheek(int x, int y, int rgb[]){
  RGBmatrixPanel::drawLine(x-1,y,x-1,y+1,RGBmatrixPanel::Color444(rgb[0],rgb[1],rgb[2]));
  RGBmatrixPanel::drawLine(x+2,y,x+2,y+1,RGBmatrixPanel::Color444(rgb[0],rgb[1],rgb[2]));
  RGBmatrixPanel::drawLine(x,y-1,x+1,y-1,RGBmatrixPanel::Color444(rgb[0],rgb[1],rgb[2]));
  RGBmatrixPanel::drawLine(x,y+2,x+1,y+2,RGBmatrixPanel::Color444(rgb[0],rgb[1],rgb[2]));
}


void Screen::cute_mouth(int x, int y, int rgb[]){
  RGBmatrixPanel::drawLine(x-4,y,x-2,y+2,RGBmatrixPanel::Color444(rgb[0],rgb[1],rgb[2]));
  RGBmatrixPanel::drawLine(x-1,y+2,x,y+1,RGBmatrixPanel::Color444(rgb[0],rgb[1],rgb[2]));
  RGBmatrixPanel::drawLine(x+1,y+2,x+2,y+2,RGBmatrixPanel::Color444(rgb[0],rgb[1],rgb[2]));
  RGBmatrixPanel::drawLine(x+3,y+1,x+4,y,RGBmatrixPanel::Color444(rgb[0],rgb[1],rgb[2]));
  RGBmatrixPanel::drawPixel(x-4,y-1,RGBmatrixPanel::Color444(rgb[0],rgb[1],rgb[2]));
  RGBmatrixPanel::drawPixel(x+4,y-1,RGBmatrixPanel::Color444(rgb[0],rgb[1],rgb[2]));
  RGBmatrixPanel::drawPixel(x,y,RGBmatrixPanel::Color444(rgb[0],rgb[1],rgb[2]));
  
  //RGBmatrixPanel::drawLine(x+3,y+2,x+5,y,RGBmatrixPanel::Color444(r,g,b));
  //RGBmatrixPanel::drawPixel(x-4,y-1,RGBmatrixPanel::Color444(r,g,b));
  //RGBmatrixPanel::drawPixel(x+5,y-1,RGBmatrixPanel::Color444(r,g,b));
}
    
void Screen::cute_cheeks(int x, int y, int rgb[]){
   RGBmatrixPanel::drawLine(x-2,y-1,x+1,y-1,RGBmatrixPanel::Color444(rgb[0],rgb[1],rgb[2]));
   RGBmatrixPanel::drawLine(x-2,y,x+1,y,RGBmatrixPanel::Color444(rgb[0],rgb[1],rgb[2]));
}
    
void Screen::eight_bit_eye(int x, int y){
  RGBmatrixPanel::fillCircle(x,y,2,RGBmatrixPanel::Color444(0,0,15));
  RGBmatrixPanel::drawLine(x-1,y-1,x-1,y+1,RGBmatrixPanel::Color444(3,0,15));
  RGBmatrixPanel::drawLine(x,y+1,x+1,y+1,RGBmatrixPanel::Color444(3,0,15));
  RGBmatrixPanel::drawLine(x,y-1,x+1,y-1,RGBmatrixPanel::Color444(WHITE));
  RGBmatrixPanel::drawLine(x,y,x+1,y,RGBmatrixPanel::Color444(WHITE));
}
    
    
void Screen::eight_bit_face(){
	//clear_face();
	int cheek_color[] = {15,0,0};
	int mouth_color[] = {12,0,15};
	eight_bit_eye(10,5);
	eight_bit_eye(18,5);
	circle_cheek(5,9,cheek_color);
	circle_cheek(23,9,cheek_color);
	happy_mouth(14,11,mouth_color);
}
    
void Screen::happy_face(){
	//clear_face();
	int eye_color[] = {3,0,15};
	int mouth_color[] = {3,0,15};
	int cheek_color[] = {12,0,15};

	happy_eye(10,5,eye_color);
	happy_eye(18,5,eye_color);
	cute_mouth(14,12,mouth_color);
	cute_cheeks(7,9,cheek_color);
	cute_cheeks(22,9,cheek_color);
  
}
    
void Screen::sleepy_face(){
  int eye_color[] = {3,0,15};
  int mouth_color[] = {3,0,15};
  int cheek_color[] = {12,0,15};
  sleepy_eye(10,5,eye_color);
  sleepy_eye(18,5,eye_color);
  cute_mouth(14,12,mouth_color);
  cute_cheeks(7,9,cheek_color);
  cute_cheeks(22,9,cheek_color);
}
    
void Screen::clear_face(){
  RGBmatrixPanel::fillRect(0,2,31,15,RGBmatrixPanel::Color444(BLACK));
}
    
void Screen::update_screen(int message[]){
  if ((preamble_check(message) && chksum_check(message)) || TEST_MODE){
    connected_status(message[CONNECTED_POS]);
    display_status(message[STATUS_POS]);
    display_battery(message[BATTERY_POS]);

    int temp[]  = {(message[5] & 0x0F),(message[6] & 0xF0) >> 4,(message[6] & 0x0F),(message[7] & 0xF0) >> 4,(message[7] & 0x0F)};
    int drive[] = {(message[8] & 0xF0) >> 4,(message[8] & 0x0F),(message[9] & 0xF0) >> 4,(message[9] & 0x0F),(message[10] & 0xF0) >> 4,(message[10] & 0x0F)};
    int steer[] = {(message[11] & 0xF0) >> 4,(message[11] & 0x0F),(message[12] & 0xF0) >> 4,(message[12] & 0x0F),(message[13] & 0xF0) >> 4,(message[13] & 0x0F)};

    display_temp(temp);
    display_currents(drive,steer);
    //display_face(message[FACE_POS]);
    //happy_face();
  }
}