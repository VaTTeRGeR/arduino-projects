#include <U8glib.h>

U8GLIB_PCD8544 u8g(52, 51, 22, 0, 2);

void setup() {
  u8g.setColorIndex(1);
}

void loop() {
  // picture loop
  u8g.firstPage();  
  do {
    u8g.setFont(u8g_font_unifont);
    u8g.drawStr( 0, 22, "Hello World!");
  } while( u8g.nextPage() );
  
  // rebuild the picture after some delay
  digitalWrite(13, HIGH);
  delay(50);
  digitalWrite(13, LOW);
}
