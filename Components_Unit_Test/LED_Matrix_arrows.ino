#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_DotStarMatrix.h>
#include <Adafruit_DotStar.h>
#ifndef PSTR
 #define PSTR 
#endif

#define DATAPIN  10
#define CLOCKPIN 11

Adafruit_DotStarMatrix matrix = Adafruit_DotStarMatrix(
  8, 8, DATAPIN, CLOCKPIN,
  DS_MATRIX_TOP     + DS_MATRIX_RIGHT +
  DS_MATRIX_COLUMNS + DS_MATRIX_PROGRESSIVE,
  DOTSTAR_BRG);

const uint16_t colors[] = {
  matrix.Color(125, 125, 0), // Red for right arrow
  matrix.Color(125, 125, 0)  // Green for left arrow
};

// Bitmap for a right-pointing arrow
const uint8_t rightArrow[] = {
  0b00000000, // Row 1: Empty
  0b00001000, // Row 2: Small arrow tip
  0b00001100, // Row 3: Arrowhead part 1
  0b11111111, // Row 4: Line with arrowhead
  0b11111111, // Row 5: Line with arrowhead
  0b00001100, // Row 6: Arrowhead part 2
  0b00001000, // Row 7: Small arrow tip
  0b00000000  // Row 8: Empty
};

// Bitmap for a left-pointing arrow
const uint8_t leftArrow[] = {
  0b00000000, // Row 1: Empty
  0b00010000, // Row 2: Small arrow tip
  0b00110000, // Row 3: Arrowhead part 1
  0b11111111, // Row 4: Line with arrowhead
  0b11111111, // Row 5: Line with arrowhead
  0b00110000, // Row 6: Arrowhead part 2
  0b00010000, // Row 7: Small arrow tip
  0b00000000  // Row 8: Empty
};

void setup() {
  matrix.begin();
  matrix.setBrightness(40);
}

void loop() {
  // Display right arrow
  matrix.fillScreen(0);
  matrix.drawBitmap(0, 0, rightArrow, 8, 8, colors[0]);
  matrix.show();
  delay(1000);

  // Display left arrow
  matrix.fillScreen(0);
  matrix.drawBitmap(0, 0, leftArrow, 8, 8, colors[1]);
  matrix.show();
  delay(1000);
}
