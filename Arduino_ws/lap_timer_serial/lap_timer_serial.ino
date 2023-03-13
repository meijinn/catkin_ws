float kCheckDistance = 5.0;  // ミニ四駆が通過したと判定する距離
int duration = 0;                  // 超音波が反射するまでの片道時間
float distance = 0;                // 測定物までの距離(cm)
float base_time = 0;               // 1ラップ目のスタート時間
float lap_start = 0;               // 各ラップのスタート時間
float lap_end = 0;                 // 各ラップのゴール時間
float lap_time = 0;                // ラップタイム
int lap_count = 0;                 // ラップ数

void setup() {
  Serial.begin(115200);
  pinMode(2, OUTPUT);
  pinMode(3, INPUT);
}

void loop() {
  digitalWrite(2, HIGH);
  delayMicroseconds(10); // 10μs間出力
  digitalWrite(2, LOW);
  duration = pulseIn(3, HIGH) / 2; 
  
  // 時間から距離を算出
  // duration * 340 * 100 / 1000000が計算できない
  distance = (float) duration * 34  / 1000;

  if(distance > 0 && distance < kCheckDistance) { // 通過判定の場合
    if(lap_count == 0) {
      base_time = (float) millis() / 1000;
      Serial.println("Start!");
    } else {
      Serial.print("[Lap");
      Serial.print(lap_count);
      Serial.print("] "); 
      lap_end = (float) millis() / 1000 - base_time;
      lap_time = lap_end - lap_start;
      Serial.print("Start: ");
      Serial.print(lap_start, 3);
      Serial.print(" End: ");
      Serial.print(lap_end, 3);
      Serial.print(" Laptime: ");
      Serial.println(lap_time, 3);
  
      lap_start = lap_end; // ラップのゴール時間は次のラップのスタート時間
    }
    lap_count++;
    delay(2000); // 2秒間は判定しない
  }
  delay(10);
}
