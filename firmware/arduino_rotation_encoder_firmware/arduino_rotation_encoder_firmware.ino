#include <Arduino.h>
#include <limits.h>

// Built for Arduino Nano

const byte PIN_A = 2;  // KY-040 CLK
const byte PIN_B = 3;  // KY-040 DT

// Configure the Serial output rate here.
#define PUBLISH_HZ 20UL

#if PUBLISH_HZ == 0
#error "PUBLISH_HZ must be greater than zero"
#endif

const unsigned long PUBLISH_PERIOD_US = 1000000UL / PUBLISH_HZ;

// Updated inside the interrupt routine.
volatile int encoderCount = 0;
volatile byte previousState = 0;
volatile int8_t transitionAccumulator = 0;

// Quadrature transition lookup table.
// Invalid transitions and contact bounce contribute zero.
const int8_t TRANSITION_TABLE[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

void updateEncoder()
{
  // Combine A and B into a two-bit value:
  // 00, 01, 10 or 11.
  byte currentState =
      (digitalRead(PIN_A) << 1) |
       digitalRead(PIN_B);

  byte tableIndex = (previousState << 2) | currentState;
  int8_t movement = TRANSITION_TABLE[tableIndex];

  previousState = currentState;

  if (movement == 0) {
    return;
  }

  transitionAccumulator += movement;

  // A complete encoder cycle normally contains four transitions.
  if (transitionAccumulator >= 4) {
    transitionAccumulator = 0;

    // Prevent signed integer overflow.
    if (encoderCount < INT_MAX) {
      encoderCount--;
    }
  }
  else if (transitionAccumulator <= -4) {
    transitionAccumulator = 0;

    // Prevent signed integer underflow.
    if (encoderCount > INT_MIN) {
      encoderCount++;
    }
  }
}

void setup()
{
  Serial.begin(9600);

  // Keep the signals at a defined HIGH level when the encoder
  // contacts are open.
  pinMode(PIN_A, INPUT_PULLUP);
  pinMode(PIN_B, INPUT_PULLUP);

  // Read the starting state before enabling interrupts.
  previousState =
      (digitalRead(PIN_A) << 1) |
       digitalRead(PIN_B);

  attachInterrupt(
      digitalPinToInterrupt(PIN_A),
      updateEncoder,
      CHANGE
  );

  attachInterrupt(
      digitalPinToInterrupt(PIN_B),
      updateEncoder,
      CHANGE
  );
}

void loop()
{
  static unsigned long nextPublishTime = micros();

  unsigned long now = micros();

  // Signed subtraction makes this safe across micros() overflow.
  if ((long)(now - nextPublishTime) >= 0) {
    nextPublishTime += PUBLISH_PERIOD_US;

    int countSnapshot;

    // encoderCount is modified by an interrupt. Temporarily disabling
    // interrupts prevents reading it while it is being changed.
    noInterrupts();
    countSnapshot = encoderCount;
    interrupts();

    Serial.println(countSnapshot);

    // If the program falls far behind, avoid sending a burst of
    // delayed measurements.
    now = micros();

    if ((long)(now - nextPublishTime) >= 0) {
      nextPublishTime = now + PUBLISH_PERIOD_US;
    }
  }
}