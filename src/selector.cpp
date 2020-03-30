#include "vex.h"

namespace selector{

using namespace vex;

int auton;
int bg_color;

const int offset = 8;
const int fontSize = 10;
const int btn_size = 90;
const int btn_gap = 20;
const int label_pos_x = btn_size/2 + btn_gap - fontSize;
const int label_pos_y = btn_size/2 + btn_gap + fontSize;

void drawButton(int count){

  int x = count;
  int y = 1;

  if(count > 4){
    x -= 4;
    y ++;
  }

  bool selected = count == auton ? true : false;

  if(selected){
    Brain.Screen.setFillColor(bg_color);
  }else{
    Brain.Screen.setFillColor(black);
  }

  Brain.Screen.setFont(fontType::mono40);
  Brain.Screen.setPenWidth(5);
  Brain.Screen.drawRectangle(btn_gap*x + btn_size*(x-1), btn_gap*y + btn_size*(y-1), btn_size, btn_size);
  Brain.Screen.printAt(label_pos_x*x + label_pos_x*(x-1), label_pos_y*y + label_pos_y/2*(y-1), "%d", count);
}

void drawAllButtons(){
  for(int i = 1; i <= 8; i++){
    drawButton(i);
  }
}

void update(){
  int press = Brain.Screen.xPosition() / (btn_size + btn_gap) + 1;
  press += 4*(Brain.Screen.yPosition()/ (btn_size + btn_gap));
  auton = press;
  drawAllButtons();
}

void cycle(){
  auton ++;
  if(auton > 8)
    auton = 1;
  drawAllButtons();
}

void init(){
  Brain.Screen.setOrigin(offset, 0);
  Brain.Screen.clearScreen(color::black);
  Brain.Screen.setPenColor(white);
  
  bg_color = HUE;
  auton = DEFAULT_AUTON;

  drawAllButtons();

  Brain.Screen.pressed(update);

  if(USING_CONTROLLER)
    Controller1.CONTROLLER_BUTTON.pressed(cycle);
}

} // namespace selector