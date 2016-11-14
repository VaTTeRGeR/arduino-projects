#include "U8glib.h"

U8GLIB_PCD8544 u8g(/*52, 51, */53, 2, 4);		// SPI Com: SCK = 13, MOSI = 11, CS = 10, A0 = 9, Reset = 8

#define SECONDS 10
uint8_t flip_color = 0;
uint8_t draw_color = 1;

void draw_set_screen(void) {
  // graphic commands to redraw the complete screen should be placed here  
  if ( u8g.getMode() == U8G_MODE_HICOLOR ) {
    if ( flip_color == 0 ) 
      u8g.setHiColorByRGB(0,0,0);
    else
      u8g.setHiColorByRGB(255,255,255);
  }
  else {
    u8g.setColorIndex(flip_color);
  }
  u8g.drawBox( 0, 0, u8g.getWidth(), u8g.getHeight() );
}

void draw_clip_test(void) {
  u8g_uint_t i, j, k;
  char buf[3] = "AB";
  k = 0;
  if ( u8g.getMode() == U8G_MODE_HICOLOR ) {
    u8g.setHiColorByRGB(255,255,255);
  }
  else {
    u8g.setColorIndex(draw_color);
  }
  u8g.setFont(u8g_font_6x10);
  
  for( i = 0; i  < 6; i++ ) {
    for( j = 1; j  < 8; j++ ) {
      u8g.drawHLine(i-3, k, j);
      u8g.drawHLine(i-3+10, k, j);
      
      u8g.drawVLine(k+20, i-3, j);
      u8g.drawVLine(k+20, i-3+10, j);
      
      k++;
    }
  }
  u8g.drawStr(0-3, 50, buf);
  u8g.drawStr180(0+3, 50, buf);
  
  u8g.drawStr(u8g.getWidth()-3, 40, buf);
  u8g.drawStr180(u8g.getWidth()+3, 40, buf);

  u8g.drawStr90(u8g.getWidth()-10, 0-3, buf);
  u8g.drawStr270(u8g.getWidth()-10, 3, buf);

  u8g.drawStr90(u8g.getWidth()-20, u8g.getHeight()-3, buf);
  u8g.drawStr270(u8g.getWidth()-20, u8g.getHeight()+3, buf);
  
}

void draw_char(void) {
  char buf[2] = "@";
  u8g_uint_t i, j;
  // graphic commands to redraw the complete screen should be placed here  
  if ( u8g.getMode() == U8G_MODE_HICOLOR ) {
    u8g.setHiColorByRGB(255,255,255);
  }
  else {
    u8g.setColorIndex(draw_color);
  }
  u8g.setFont(u8g_font_6x10);
  j = 8;
  for(;;) {
    i = 0;
    for(;;) {
      u8g.drawStr( i, j, buf);
      i += 8;
      if ( i > u8g.getWidth() )
        break;
    }
    j += 8;
    if ( j > u8g.getHeight() )
      break;
  }
  
}

void draw_pixel(void) {
  u8g_uint_t x, y, w2, h2;
  if ( u8g.getMode() == U8G_MODE_HICOLOR ) {
    u8g.setHiColorByRGB(255,255,255);
  }
  else {
    u8g.setColorIndex(draw_color);
  }
  w2 = u8g.getWidth();
  h2 = u8g.getHeight();
  w2 /= 2;
  h2 /= 2;
  for( y = 0; y < h2; y++ ) {
    for( x = 0; x < w2; x++ ) {
      if ( (x + y) & 1 ) {
        u8g.drawPixel(x,y);
        u8g.drawPixel(x,y+h2);
        u8g.drawPixel(x+w2,y);
        u8g.drawPixel(x+w2,y+h2);
      }
    }
  }
}

// returns unadjusted FPS
uint16_t picture_loop_with_fps(void (*draw_fn)(void)) {
  uint16_t FPS10 = 0;
  uint32_t time;
  
  time = millis() + SECONDS*1000;
  
  // picture loop
  do {
    u8g.firstPage();  
    do {
      draw_fn();
    } while( u8g.nextPage() );
    FPS10++;
    flip_color = flip_color ^ 1;
  } while( millis() < time );
  return FPS10;  
}

const char *convert_FPS(uint16_t fps) {
  static char buf[6];
  strcpy(buf, u8g_u8toa( (uint8_t)(fps/10), 3));
  buf[3] =  '.';
  buf[4] = (fps % 10) + '0';
  buf[5] = '\0';
  return buf;
}

void show_result(const char *s, uint16_t fps) {
  // assign default color value
  if ( u8g.getMode() == U8G_MODE_HICOLOR ) {
    u8g.setHiColorByRGB(255,255,255);
  }
  else {
    u8g.setColorIndex(draw_color);
  }
  u8g.setFont(u8g_font_8x13B);
  u8g.firstPage();  
  do {
    u8g.drawStr(0,12, s);
    u8g.drawStr(0,24, convert_FPS(fps));
  } while( u8g.nextPage() );
}

void setup(void) {
  // flip screen, if required
  // u8g.setRot180();
  
  // assign default color value
  if ( u8g.getMode() == U8G_MODE_R3G3B2 ) 
    draw_color = 255;     // white
  else if ( u8g.getMode() == U8G_MODE_GRAY2BIT )
    draw_color = 3;         // max intensity
  else if ( u8g.getMode() == U8G_MODE_BW )
    draw_color = 1;         // pixel on
  else if ( u8g.getMode() == U8G_MODE_HICOLOR ) {
    u8g.setHiColorByRGB(255,255,255);
  }
}

void loop(void) {
  uint16_t fps;
  fps = picture_loop_with_fps(draw_clip_test);
  show_result("draw clip test", fps);
  delay(5000);
  fps = picture_loop_with_fps(draw_set_screen);
  show_result("clear screen", fps);
  delay(5000);
  fps = picture_loop_with_fps(draw_char);
  show_result("draw @", fps);
  delay(5000);  
  fps = picture_loop_with_fps(draw_pixel);
  show_result("draw pixel", fps);
  delay(5000);
}

