// Arduino pin assignment
#define PIN_IR A0

// Curve fitting 설정
#define NUM_POINTS  7       // 0, 50, 100, ..., 300 mm
#define DIST_STEP   50      // 측정 간격 (mm)
#define MAX_DEGREE  5       // 허용하는 최대 차수
#define X_SCALE     1024.0  // 센서 값 정규화 상수

unsigned int sensor_val[NUM_POINTS];  // 거리별 센서 전압 값 (0 ~ 1023)
float actual_dist[NUM_POINTS];        // 탁구공의 실제 위치 (mm)
float coef[MAX_DEGREE + 1];           // fitting 결과 계수 c0 ~ cm

int method = 1;     // 1: 다항식, 2: 역수 다항식
int degree = 3;     // 다항식 차수
int n_repeat = 10;  // ir_sensor_filtered()의 측정 반복 횟수 n

void setup()
{
  Serial.begin(1000000);  // initialize serial port
}

void loop()
{
  Serial.println();
  Serial.println(F("##### IR sensor curve fitting #####"));

  // Step 1. 설정 값 입력
  method   = ask_int(F("Fitting method (1: polynomial, 2: inverse polynomial)"), 1, 2, 1);
  degree   = ask_int(F("Degree"), 1, MAX_DEGREE, (method == 1) ? 3 : 2);
  n_repeat = ask_int(F("Number of measurements n for median filter"), 1, 100, 10);

  // Step 2. 거리별 전압 측정
  Serial.println();
  Serial.println(F("Move the ball and press ENTER at each position."));
  for (int i = 0; i < NUM_POINTS; i++) {
    actual_dist[i] = (float)i * DIST_STEP;

    Serial.print(F("Place the ball at "));
    Serial.print(actual_dist[i] / 10.0, 1);
    Serial.println(F(" cm, then press ENTER"));
    read_line();

    // Take a median value from multiple measurements
    sensor_val[i] = ir_sensor_filtered(n_repeat, 0.5, 0);
    Serial.print(F("FLT:")); Serial.println(sensor_val[i]);
  }

  // Step 3. Curve fitting 및 결과 출력
  if (curve_fit())
    print_result();
  else
    Serial.println(F("Curve fitting failed. Lower the degree or measure again."));

  Serial.println();
  Serial.println(F("Press ENTER to start again."));
  read_line();
}

String read_line()
{
  // 엔터키가 들어올 때까지 대기한 뒤 한 줄을 읽어 옴
  while (Serial.available() == 0)
    ;
  String s = Serial.readStringUntil('\n');
  s.trim();
  return s;
}

int ask_int(const __FlashStringHelper *msg, int lo, int hi, int def)
{
  // 범위(lo ~ hi) 안의 정수가 입력될 때까지 반복해서 질문
  // 그냥 엔터키를 입력하면 기본값(def) 사용
  while (1) {
    Serial.print(msg);
    Serial.print(F(" [")); Serial.print(lo);
    Serial.print(F("~"));  Serial.print(hi);
    Serial.print(F(", default ")); Serial.print(def);
    Serial.println(F("]:"));

    String s = read_line();
    if (s.length() == 0) {
      Serial.print(F("  => ")); Serial.println(def);
      return def;
    }
    int v = s.toInt();
    if (v >= lo && v <= hi) {
      Serial.print(F("  => ")); Serial.println(v);
      return v;
    }
    Serial.println(F("  Invalid input. Try again."));
  }
}

float feature(unsigned int a_value)
{
  // 센서 값을 fitting에 사용하는 변수로 변환
  // method 1: t = a_value / 1024,  method 2: z = 1 / t
  float t = ((a_value < 1) ? 1 : a_value) / X_SCALE;  // 0 나누기 방지
  return (method == 1) ? t : 1.0 / t;
}

float volt_to_distance(unsigned int a_value)
{
  // fitting으로 구한 계수를 이용해 전압 값을 거리(mm)로 변환 (Horner 방식)
  float z = feature(a_value);
  float result = coef[degree];
  for (int i = degree - 1; i >= 0; i--)
    result = result * z + coef[i];
  return result;
}

bool gauss_solve(float a[][MAX_DEGREE + 2], int n, float *x)
{
  // 가우스 소거법(partial pivoting)으로 연립방정식 풀이
  // a: n x (n+1) 확대행렬, 성공하면 true
  for (int col = 0; col < n; col++) {
    int piv = col;
    for (int r = col + 1; r < n; r++)
      if (fabs(a[r][col]) > fabs(a[piv][col]))
        piv = r;
    if (fabs(a[piv][col]) < 1e-12)
      return false;

    if (piv != col) {
      for (int c = 0; c <= n; c++) {
        float tmp = a[col][c];
        a[col][c] = a[piv][c];
        a[piv][c] = tmp;
      }
    }
    for (int r = col + 1; r < n; r++) {
      float f = a[r][col] / a[col][col];
      for (int c = col; c <= n; c++)
        a[r][c] -= f * a[col][c];
    }
  }
  for (int r = n - 1; r >= 0; r--) {
    float s = a[r][n];
    for (int c = r + 1; c < n; c++)
      s -= a[r][c] * x[c];
    x[r] = s / a[r][r];
  }
  return true;
}

bool curve_fit()
{
  // 최소제곱법: 정규방정식 (A^T A) c = A^T d 를 구성해서 푼다
  int n = degree + 1;
  float a[MAX_DEGREE + 1][MAX_DEGREE + 2];

  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
      float sum = 0;
      for (int k = 0; k < NUM_POINTS; k++)
        sum += pow(feature(sensor_val[k]), i + j);
      a[i][j] = sum;
    }
    float b = 0;
    for (int k = 0; k < NUM_POINTS; k++)
      b += actual_dist[k] * pow(feature(sensor_val[k]), i);
    a[i][n] = b;
  }
  return gauss_solve(a, n, coef);
}

void print_result()
{
  // 1) 측정 데이터와 fitting 오차, R^2
  Serial.println();
  Serial.println(F("===== Result ====="));
  Serial.println(F("Sensor reading, Actual (mm), Fitted (mm), Error (mm)"));

  float mean = 0;
  for (int i = 0; i < NUM_POINTS; i++)
    mean += actual_dist[i];
  mean /= NUM_POINTS;

  float ss_res = 0, ss_tot = 0;
  for (int i = 0; i < NUM_POINTS; i++) {
    float fitted = volt_to_distance(sensor_val[i]);
    float err = fitted - actual_dist[i];
    ss_res += err * err;
    ss_tot += (actual_dist[i] - mean) * (actual_dist[i] - mean);

    Serial.print(sensor_val[i]);     Serial.print(F(", "));
    Serial.print(actual_dist[i], 1); Serial.print(F(", "));
    Serial.print(fitted, 1);         Serial.print(F(", "));
    Serial.println(err, 1);
  }
  float r2 = (ss_tot > 0) ? 1.0 - ss_res / ss_tot : 0.0;
  Serial.print(F("R^2 = "));
  Serial.println(r2, 6);
  if (r2 < 0.995)
    Serial.println(F("(R^2 is low. Try a higher degree or the other method.)"));

  // 2) 아두이노에서 바로 실행 가능한 volt_to_distance() 출력
  Serial.println();
  Serial.println(F("===== Copy the function below into your sketch ====="));
  Serial.println(F("float volt_to_distance(unsigned int a_value)"));
  Serial.println(F("{"));
  Serial.println(F("  float t = ((a_value < 1) ? 1 : a_value) / 1024.0;"));
  if (method == 2)
    Serial.println(F("  float z = 1.0 / t;"));

  // c0 + v * (c1 + v * (c2 + ... )) 형태로 출력
  Serial.print(F("  return "));
  for (int i = 0; i < degree; i++) {
    Serial.print(coef[i], 6);
    if (method == 1)
      Serial.print(F(" + t * ("));
    else
      Serial.print(F(" + z * ("));
  }
  Serial.print(coef[degree], 6);
  for (int i = 0; i < degree; i++)
    Serial.print(F(")"));
  Serial.println(F(";  // mm"));
  Serial.println(F("}"));
  Serial.println(F("===================================================="));
}

int compare(const void *a, const void *b) {
  return (*(unsigned int *)a - *(unsigned int *)b);
}

unsigned int ir_sensor_filtered(unsigned int n, float position, int verbose)
{
  // Eliminate spiky noise of an IR distance sensor by repeating measurement and taking a middle value
  // n: number of measurement repetition
  // position: the percentile of the sample to be taken (0.0 <= position <= 1.0)
  // verbose: 0 - normal operation, 1 - observing the internal procedures, and 2 - calculating elapsed time.
  // Example 1: ir_sensor_filtered(n, 0.5, 0) => return the median value among the n measured samples.
  // returns 0, if any error occurs

  unsigned int *ir_val, ret_val;
  unsigned int start_time;

  if (verbose >= 2)
    start_time = millis();

  if ((n == 0) || (n > 100) || (position < 0.0) || (position > 1))
    return 0;

  if (position == 1.0)
    position = 0.999;

  if (verbose == 1) {
    Serial.print(F("n: ")); Serial.print(n);
    Serial.print(F(", position: ")); Serial.print(position);
    Serial.print(F(", ret_idx: "));  Serial.println((unsigned int)(n * position));
  }

  ir_val = (unsigned int *)malloc(sizeof(unsigned int) * n);
  if (ir_val == NULL)
    return 0;

  if (verbose == 1)
    Serial.print(F("IR:"));

  for (int i = 0; i < n; i++) {
    ir_val[i] = analogRead(PIN_IR);
    if (verbose == 1) {
      Serial.print(F(" "));
      Serial.print(ir_val[i]);
    }
  }

  if (verbose == 1)
    Serial.print(F("  => "));

  qsort(ir_val, n, sizeof(unsigned int), compare);
  ret_val = ir_val[(unsigned int)(n * position)];

  if (verbose == 1) {
    for (int i = 0; i < n; i++) {
      Serial.print(F(" "));
      Serial.print(ir_val[i]);
    }
    Serial.print(F(" :: "));
    Serial.println(ret_val);
  }
  free(ir_val);

  if (verbose >= 2) {
    Serial.print(F("Elapsed time: ")); Serial.print(millis() - start_time); Serial.println(F("ms"));
  }

  return ret_val;
}
