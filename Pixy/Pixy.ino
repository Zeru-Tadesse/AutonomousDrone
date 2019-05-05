
#include <SPI.h>
#include <Pixy.h>

Pixy pixy;
String blockName = "DEFAULT";
uint16_t x = 0;
uint16_t y = 0;
uint16_t width = 0;
uint16_t height = 0;
const int increment = 2000; //Default 1000; 1000 = 1 second
const uint16_t s_width = 319;
const uint16_t s_height = 199;
const uint16_t idealW = 75;
const uint16_t idealH = 75;
//======LED PINS========//
uint8_t b_center, b_t1, b_t2, b_t3, b_b1, b_b2, b_b3, b_l1, b_l2, b_l3, b_r1, b_r2, b_r3;
//uint8_t f_center, f_f1, f_f2, f_b1, f_b2;

int f_center = 13;
uint8_t f_f1 = 12;
uint8_t f_f2 = 8;
uint8_t f_b1 = 7;
uint8_t f_b2 = 4;

//======================//
void setup()
{
  //======LED PINS========//
  pinMode(f_center, OUTPUT);
  //pinMode(f_f1, OUTPUT);
  //pinMode(f_f2, OUTPUT);
  //pinMode(f_b1, OUTPUT);
  //pinMode(f_b2, OUTPUT);
  Serial.begin(9600);
  Serial.print("Starting...\n");
  Serial.println("Screen Size: 0-319,0-199");
  pixy.init();

}
void loop()
{
  digitalWrite(f_center, HIGH);
  delay(500);
  digitalWrite(f_center,LOW);
  delay(500);
  int count = 0;

  updateBlocks();
  delay(increment);
  count++;


}


void updateBlocks()
{
  uint16_t blocks = pixy.getBlocks();
  if (blocks) {
    for (int i = 0; i < blocks; i ++)
    {
      blockName = pixy.blocks[i].signature;
      x = pixy.blocks[i].x;
      y = pixy.blocks[i].y;
      width = pixy.blocks[i].width;
      height = pixy.blocks[i].height;
    }
    printB();
    printInstructions();
  }

}
void printB() {
  int hDist = s_height / 2 - y;
  int wDist = s_width / 2 - x;
  if (true) {
    Serial.println("====================");
    Serial.print("Block:  ");
    Serial.println(blockName);
    Serial.print("CENTER: (");
    Serial.print(x);
    Serial.print(" ,");
    Serial.print(y);
    Serial.println(")");
    Serial.print("Width:  ");
    Serial.println(width);
    Serial.print("Height: ");
    Serial.println(height);
    Serial.println();
  }


  Serial.print("Distance from Center Y: ");
  Serial.println(hDist);
  Serial.print("Distance from Center X: ");
  Serial.println(wDist);
  //TODO:
}
void printInstructions()
{
  if (width < idealW) {
    Serial.println("Move forward");
  }
  else
  {
    Serial.println("Move back");
  }



}
