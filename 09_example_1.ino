#define PIN_LED  9
#define PIN_TRIG 12
#define PIN_ECHO 13

#define SND_VEL 346.0
#define INTERVAL 25
#define PULSE_DURATION 10
#define _DIST_MIN 100
#define _DIST_MAX 300

#define TIMEOUT ((INTERVAL / 2) * 1000.0)
#define SCALE (0.001 * 0.5 * SND_VEL)

#define _EMA_ALPHA 0.2

// median filter window size: change to 3, 10, 30 for the screenshots
#define _MEDIAN_N 30

unsigned long last_sampling_time;
float dist_ema = _DIST_MAX;

// circular buffer holding the latest samples
float samples[_MEDIAN_N];
int sample_idx = 0;   // next write position
int sample_cnt = 0;   // number of valid samples (<= _MEDIAN_N)

void setup() {
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);

  Serial.begin(57600);
}

void loop() {
  float dist_raw, dist_median;

  if (millis() < last_sampling_time + INTERVAL)
    return;

  dist_raw = USS_measure(PIN_TRIG, PIN_ECHO);

  // the raw value goes straight into the median filter (no range filter)
  dist_median = median_filter(dist_raw);

  // EMA is computed from the raw value for comparison
  dist_ema = _EMA_ALPHA * dist_raw + (1.0 - _EMA_ALPHA) * dist_ema;

  Serial.print("Min:");     Serial.print(_DIST_MIN);
  Serial.print(",raw:");    Serial.print(min(dist_raw, _DIST_MAX + 100));
  Serial.print(",ema:");    Serial.print(min(dist_ema, _DIST_MAX + 100));
  Serial.print(",median:"); Serial.print(min(dist_median, _DIST_MAX + 100));
  Serial.print(",Max:");    Serial.print(_DIST_MAX);
  Serial.println("");

  if ((dist_raw < _DIST_MIN) || (dist_raw > _DIST_MAX))
    digitalWrite(PIN_LED, 1);
  else
    digitalWrite(PIN_LED, 0);

  last_sampling_time += INTERVAL;
}

float median_filter(float new_val)
{
  // store the new sample, overwriting the oldest one when full
  samples[sample_idx] = new_val;
  sample_idx = (sample_idx + 1) % _MEDIAN_N;
  if (sample_cnt < _MEDIAN_N)
    sample_cnt++;

  // copy valid samples so the buffer order stays intact
  float sorted[_MEDIAN_N];
  for (int i = 0; i < sample_cnt; i++)
    sorted[i] = samples[i];

  // insertion sort
  for (int i = 1; i < sample_cnt; i++) {
    float key = sorted[i];
    int j = i - 1;
    while (j >= 0 && sorted[j] > key) {
      sorted[j + 1] = sorted[j];
      j--;
    }
    sorted[j + 1] = key;
  }

  // odd count: middle value, even count: average of the two middle values
  if (sample_cnt % 2 == 1)
    return sorted[sample_cnt / 2];
  else
    return (sorted[sample_cnt / 2 - 1] + sorted[sample_cnt / 2]) / 2.0;
}

float USS_measure(int TRIG, int ECHO)
{
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);

  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE;
}
