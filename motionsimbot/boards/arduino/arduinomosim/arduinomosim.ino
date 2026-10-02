// Potentiometer noise characterisation — Arduino UNO
// 115200 baud. Open: Tools > Serial Plotter.
//
// Three traces:
//   raw      — instantaneous ADC reading (shows spikes in real time)
//   avg      — EMA-smoothed position (the "true" pot angle)
//   noise_pp — rolling peak-to-peak over the last 64 samples  ← KEY metric
//
// Procedure:
//   1. Run with no motor → note noise_pp baseline
//   2. Run motor nearby  → any jump in noise_pp = EMI coupling
//
// LED blinks faster/slower with pot position (sanity check).

#define RING_SIZE  64         // rolling window for noise stats (~1.3 s at 50 Hz)
#define PRINT_MS   20         // 50 Hz print rate — smooth plotter, not flooding
#define EMA_ALPHA  0.05f      // position smoothing (lower = smoother)
#define LED_PIN    PB5        // pin 13

volatile uint16_t adcISR = 0;
volatile bool     newVal  = false;

uint16_t ring[RING_SIZE];
uint8_t  ringHead = 0;
bool     ringFull = false;
float    ema      = 512.0f;

// ---------------------------------------------------------------------------
ISR(ADC_vect) {
  adcISR = ADC;
  newVal = true;
}

// ---------------------------------------------------------------------------
void setup() {
  DDRB |= _BV(LED_PIN);

  // Free-running ADC on A0, AVcc ref, prescaler 128, interrupt enabled
  ADMUX  = _BV(REFS0);                                   // A0, AVcc ref
  ADCSRB = 0;                                            // free-running mode
  ADCSRA = _BV(ADEN) | _BV(ADSC) | _BV(ADATE) | _BV(ADIE) |
           _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);        // prescaler 128
  DIDR0 |= _BV(ADC0D);                                   // disable digital buffer on A0
  sei();

  Serial.begin(115200);

  // Seed ring buffer with midpoint so noise_pp starts at 0
  for (uint8_t i = 0; i < RING_SIZE; i++) ring[i] = 512;
}

// ---------------------------------------------------------------------------
static uint16_t rollingPeakToPeak() {
  uint16_t mn = 1023, mx = 0;
  uint8_t  len = ringFull ? RING_SIZE : ringHead;
  if (len == 0) return 0;
  for (uint8_t i = 0; i < len; i++) {
    if (ring[i] < mn) mn = ring[i];
    if (ring[i] > mx) mx = ring[i];
  }
  return mx - mn;
}

// ---------------------------------------------------------------------------
void loop() {
  // --- Consume latest ISR sample into ring buffer + EMA ---
  if (newVal) {
    uint16_t raw;
    uint8_t sreg = SREG; cli();
    raw    = adcISR;
    newVal = false;
    SREG   = sreg;

    ring[ringHead] = raw;
    ringHead = (ringHead + 1) % RING_SIZE;
    if (ringHead == 0) ringFull = true;

    ema += EMA_ALPHA * ((float)raw - ema);
  }

  // --- Serial Plotter output at PRINT_MS rate ---
  static unsigned long lastPrint = 0;
  unsigned long now = millis();
  if (now - lastPrint >= PRINT_MS) {
    lastPrint = now;

    uint16_t raw;
    uint8_t sreg = SREG; cli();
    raw = adcISR;
    SREG = sreg;

    uint16_t pp = rollingPeakToPeak();

    // Labelled format: Arduino 2.x Serial Plotter draws one trace per label
    Serial.print("raw:");      Serial.print(raw);
    Serial.print(" avg:");     Serial.print((uint16_t)ema);
    Serial.print(" noise_pp:"); Serial.println(pp);
  }

  // --- LED blink speed tracks pot position (physical sanity check) ---
  static unsigned long lastToggle = 0;
  uint16_t val;
  uint8_t sreg = SREG; cli();
  val = adcISR;
  SREG = sreg;
  unsigned long interval = 1000UL - ((unsigned long)val * 950UL) / 1023UL;
  if (millis() - lastToggle >= interval) {
    lastToggle = millis();
    PORTB ^= _BV(LED_PIN);
  }
}
