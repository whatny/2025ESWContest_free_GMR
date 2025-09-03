#include "kproject_dwm1000.h"
#include <SoftwareSerial.h>
#include <math.h>

#define FILTER_SIZE 10
#define MOVING_AVG_SIZE 5

// 소프트웨어 시리얼 설정
SoftwareSerial soft(2, 3);
// 블루투스 모듈용 소프트웨어 시리얼
SoftwareSerial BT(8, 9);  // RX, TX (HC-05 연결)

kproject_dwm1000 dwm;

// 중앙값 필터 버퍼
double d1_buffer[FILTER_SIZE] = {0};
double d2_buffer[FILTER_SIZE] = {0};
double d3_buffer[FILTER_SIZE] = {0};
double d4_buffer[FILTER_SIZE] = {0}; // 앵커4 버퍼 추가
int buffer_index = 0;
int measurement_count = 0;

// 이동 평균 필터 버퍼
double d1_moving_avg_buffer[MOVING_AVG_SIZE] = {0};
double d2_moving_avg_buffer[MOVING_AVG_SIZE] = {0};
double d3_moving_avg_buffer[MOVING_AVG_SIZE] = {0};
double d4_moving_avg_buffer[MOVING_AVG_SIZE] = {0}; // 앵커4 이동평균 버퍼 추가
int moving_avg_index = 0;
int moving_avg_count = 0;

// 마지막 유효한 거리 값 저장 변수
double last_valid_d1 = 0;
double last_valid_d2 = 0;
double last_valid_d3 = 0;
double last_valid_d4 = 0;
bool has_valid_measurement = false;

// 위치 데이터 전송 함수 (XYZ 바이너리 형식)
void sendPositionBinary(double x, double y, double z) {
  // 블루투스로 전환
  BT.listen();
  
  // float로 변환 (double -> float, 4바이트)
  float fx = (float)x;
  float fy = (float)y;
  float fz = (float)z;
  
  // 바이너리 형식으로 전송
  BT.write(0xAA);          // 시작 바이트 (1바이트)
  BT.write((byte*)&fx, 4); // float x (4바이트)
  BT.write((byte*)&fy, 4); // float y (4바이트)
  BT.write((byte*)&fz, 4); // float z (4바이트) - Z좌표 추가
  
  // DWM1000으로 다시 전환
  soft.listen();
  
  // 디버깅용 시리얼 출력
  Serial.print("📥 X: ");
  Serial.println(x, 2);
  Serial.print("📥 Y: ");
  Serial.println(y, 2);
  Serial.print("📥 Z: ");
  Serial.println(z, 2);
  Serial.println("------------------");
}

// 중앙값 필터 함수
double median_filter(double buffer[]) {
    double temp[FILTER_SIZE];
    memcpy(temp, buffer, sizeof(double) * FILTER_SIZE);

    // 버블 정렬
    for (int i = 0; i < FILTER_SIZE - 1; i++) {
        for (int j = i + 1; j < FILTER_SIZE; j++) {
            if (temp[i] > temp[j]) {
                double temp_val = temp[i];
                temp[i] = temp[j];
                temp[j] = temp_val;
            }
        }
    }
    return temp[FILTER_SIZE / 2]; // 중앙값 반환
}

// 이동 평균 필터 함수
double moving_average_filter(double buffer[], int size) {
    double sum = 0;
    for (int i = 0; i < size; i++) {
        sum += buffer[i];
    }
    return sum / size;
}

double correct_distance_anchor1(double raw) {
  if (raw < 30) {
    return raw + 35;
  } else if (raw < 60) {
    return raw + (35 - (10.0 / 30.0) * (raw - 30));  // +35 → +25
  } else if (raw < 90) {
    return raw + (25 - (10.0 / 30.0) * (raw - 60));  // +25 → +15
  } else if (raw < 120) {
    return raw + (15 - (7.0 / 30.0) * (raw - 90));   // +15 → +8
  } else if (raw < 150) {
    return raw + (8 - (3.0 / 30.0) * (raw - 120));   // +8 → +5
  } else if (raw < 160) {
    return raw + (5 - (10.0 / 10.0) * (raw - 150));  // +5 → -5
  } else if (raw < 190) {
    return raw - 20;  // 과도 측정 → 강하게 끌어당김
  } else {
    return raw - 25;  // 매우 과도한 값일 경우 추가 보정
  }
}

double correct_distance_anchor2(double raw) {
  if (raw < 40) {
    return raw + 22;
  } else if (raw < 70) {
    return raw + (20 - (4.0 / 30.0) * (raw - 40));  // +22 → +18
  } else if (raw < 100) {
    return raw + (16 - (8.0 / 30.0) * (raw - 70));  // +18 → +10
  } else if (raw < 130) {
    return raw + (8 - (5.0 / 30.0) * (raw - 100)); // +10 → +5
  } else {
    return raw + 5;
  }
}

double correct_distance_anchor3(double raw) {
  if (raw < 130) {
    return raw + 20;
  } else if (raw < 155) {
    return raw + (20 - (5.0 / 20.0) * (raw - 130));  // +25 → +15
  } else if (raw < 170) {
    return raw + (15 - (6.0 / 15.0) * (raw - 155));  // +15 → +9
  } else if (raw < 185) {
    return raw + (9 - (4.0 / 9.0) * (raw - 170));  // +9 → +5
  } else if (raw < 195) {
    return raw + (5 - (5.0 / 5.0) * (raw - 185));   // +5 → 0
  } else {
    return raw;  // 보정 없음
  }
}

double correct_distance_anchor4(double raw) {
  if (raw < 100) {
    return raw + 15;
  } else if (raw < 120) {
    return raw + (10 - (4.0 / 20.0) * (raw - 100));  // +20 → +5
  } else if (raw < 140) {
    return raw + (5 - (5.0 / 20.0) * (raw - 120));   // +5 → +0
  } else if (raw < 145) {
    return raw - 5/*(5 - (5.0 / 10.0) * (raw - 140))*/;    // +5 → 0
  } else if (raw < 165) {
    return raw -5 - (5.0 / 20.0) * (raw - 145);         // -5 → -10
  } else {
    return raw - 10 - (15.0 / 10.0) * (raw - 165);     // -10 → -25
  }
}

// 4개의 앵커를 이용한 3D 좌표 계산 함수
void trilateration_3d(double d1, double d2, double d3, double d4, double &x, double &y, double &z) {
    const double x1 = 10.0, y1 = 105.0, z1 = 0.0;      // A앵커 1
    const double x2 = 10.0, y2 = 0.0, z2 = 45.0;        // 앵커 2
    const double x3 = 140.0, y3 = 105.0, z3 = 140.0;    // 앵커 3
    const double x4 = 140.0, y4 = 0.0, z4 = 90.0;    // 앵커 4 

    double A[3][3] = {
        {2*(x2-x1), 2*(y2-y1), 2*(z2-z1)},
        {2*(x3-x1), 2*(y3-y1), 2*(z3-z1)},
        {2*(x4-x1), 2*(y4-y1), 2*(z4-z1)}
    };
    
    double b[3] = {
        pow(d1,2) - pow(d2,2) - pow(x1,2) - pow(y1,2) - pow(z1,2) + pow(x2,2) + pow(y2,2) + pow(z2,2),
        pow(d1,2) - pow(d3,2) - pow(x1,2) - pow(y1,2) - pow(z1,2) + pow(x3,2) + pow(y3,2) + pow(z3,2),
        pow(d1,2) - pow(d4,2) - pow(x1,2) - pow(y1,2) - pow(z1,2) + pow(x4,2) + pow(y4,2) + pow(z4,2)
    };

    // 3x3 행렬식 계산
    double det = 
        A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
        A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
        A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);

    // 행렬식이 너무 작으면 계산 불가능
    if (fabs(det) < 1e-6) {
        Serial.println(" Warning: det 값이 너무 작아 3D 계산 불가능!");
        x = -9999;
        y = -9999;
        z = -9999;
        return;
    }

    // 크래머 법칙을 사용한 해 계산
    double det_x = 
        b[0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
        A[0][1] * (b[1] * A[2][2] - A[1][2] * b[2]) +
        A[0][2] * (b[1] * A[2][1] - A[1][1] * b[2]);

    double det_y = 
        A[0][0] * (b[1] * A[2][2] - A[1][2] * b[2]) -
        b[0] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
        A[0][2] * (A[1][0] * b[2] - b[1] * A[2][0]);

    double det_z = 
        A[0][0] * (A[1][1] * b[2] - b[1] * A[2][1]) -
        A[0][1] * (A[1][0] * b[2] - b[1] * A[2][0]) +
        b[0] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);

    x = det_x / det;
    y = det_y / det;
    z = det_z / det;

    // X 좌표에 +5 추가
    x = x + 5.0;

    // 결과 디버깅 출력
    Serial.print("[3D TRILATERATION] X: ");
    Serial.print(x, 2);
    Serial.print(" , Y: ");
    Serial.print(y, 2);
    Serial.print(" , Z: ");
    Serial.println(z, 2);
}

void setup() {
    Serial.begin(9600);  // USB 시리얼 통신
    soft.begin(9600);    // DWM1000 모듈 통신
    BT.begin(9600);      // 블루투스 모듈 통신
    
    // 처음에는 DWM1000 통신을 활성화
    soft.listen();
    
    dwm.begin(soft);     // DWM1000 모듈 시작
    Serial.println("시스템 시작. 3D 위치 측정 시작");
}

void loop() {
    // DWM1000 통신으로 전환
    soft.listen();
    
    int tag_no = 10; // 태그 번호 선언
    double d1 = dwm.get_anchor_tag_distance(1, 10);
    double d2 = dwm.get_anchor_tag_distance(2, 10);
    double d3 = dwm.get_anchor_tag_distance(3, 10);
    double d4 = dwm.get_anchor_tag_distance(4, 10); // 앵커 4와의 거리 측정

    // -10000 체크 - 오류 발생시 이전 값 사용
    if (d1 == -10000 && has_valid_measurement) d1 = last_valid_d1;
    if (d2 == -10000 && has_valid_measurement) d2 = last_valid_d2;
    if (d3 == -10000 && has_valid_measurement) d3 = last_valid_d3;
    if (d4 == -10000 && has_valid_measurement) d4 = last_valid_d4;

    // 유효한 값 저장
    if (d1 != -10000 && d2 != -10000 && d3 != -10000 && d4 != -10000) {
        last_valid_d1 = d1;
        last_valid_d2 = d2;
        last_valid_d3 = d3;
        last_valid_d4 = d4;
        has_valid_measurement = true;
    }

    d1_buffer[buffer_index] = d1;
    d2_buffer[buffer_index] = d2;
    d3_buffer[buffer_index] = d3;
    d4_buffer[buffer_index] = d4; // 앵커 4 버퍼에 저장
    buffer_index++;
    measurement_count++;

    if (measurement_count == FILTER_SIZE) {
        double filtered_d1 = median_filter(d1_buffer);
        double filtered_d2 = median_filter(d2_buffer);
        double filtered_d3 = median_filter(d3_buffer);
        double filtered_d4 = median_filter(d4_buffer); // 앵커 4 중앙값 필터링

        filtered_d1 = correct_distance_anchor1(filtered_d1);
        filtered_d2 = correct_distance_anchor2(filtered_d2);
        filtered_d3 = correct_distance_anchor3(filtered_d3);
        filtered_d4 = correct_distance_anchor4(filtered_d4);

        Serial.print("Corrected d1: ");
        Serial.println(filtered_d1);
        Serial.print("Corrected d2: ");
        Serial.println(filtered_d2);
        Serial.print("Corrected d3: ");
        Serial.println(filtered_d3);
        Serial.print("Corrected d4: "); // 앵커 4 보정값 출력
        Serial.println(filtered_d4);

        // 이동 평균 필터 버퍼에 추가
        d1_moving_avg_buffer[moving_avg_index] = filtered_d1;
        d2_moving_avg_buffer[moving_avg_index] = filtered_d2;
        d3_moving_avg_buffer[moving_avg_index] = filtered_d3;
        d4_moving_avg_buffer[moving_avg_index] = filtered_d4; // 앵커 4 이동평균 버퍼에 추가
        moving_avg_index++;
        moving_avg_count++;

        // 이동 평균 계산
        if (moving_avg_count == MOVING_AVG_SIZE) {
            double moving_avg_d1 = moving_average_filter(d1_moving_avg_buffer, MOVING_AVG_SIZE);
            double moving_avg_d2 = moving_average_filter(d2_moving_avg_buffer, MOVING_AVG_SIZE);
            double moving_avg_d3 = moving_average_filter(d3_moving_avg_buffer, MOVING_AVG_SIZE);
            double moving_avg_d4 = moving_average_filter(d4_moving_avg_buffer, MOVING_AVG_SIZE);

            Serial.print("Moving Average d1: ");
            Serial.println(moving_avg_d1);
            Serial.print("Moving Average d2: ");
            Serial.println(moving_avg_d2);
            Serial.print("Moving Average d3: ");
            Serial.println(moving_avg_d3);
            Serial.print("Moving Average d4: ");
            Serial.println(moving_avg_d4);

            // 거리값 정수 변환
            int int_d1 = (int)moving_avg_d1;
            int int_d2 = (int)moving_avg_d2;
            int int_d3 = (int)moving_avg_d3;
            int int_d4 = (int)moving_avg_d4;

            // 3D 삼변측량 계산
            double x_3d, y_3d, z_3d;
            trilateration_3d(int_d1, int_d2, int_d3, int_d4, x_3d, y_3d, z_3d);

            // 유효한 위치인 경우에만 전송
            if (x_3d > -9990 && y_3d > -9990 && z_3d > -9990) {
                // 최종 결과 출력
                Serial.println("### FINAL POSITION (3D) ###");
                Serial.print(" TAG NO = ");
                Serial.print(tag_no);
                Serial.print(" , X = ");
                Serial.print(x_3d, 2);
                Serial.print(" , Y = ");
                Serial.print(y_3d, 2);
                Serial.print(" , Z = ");
                Serial.print(z_3d, 2);
                Serial.println("");
                Serial.println("#####################");
                
                // 블루투스로 바이너리 형식 전송
                sendPositionBinary(x_3d, y_3d, z_3d);
            }

            moving_avg_index = 0;
            moving_avg_count = 0;
        }

        buffer_index = 0;
        measurement_count = 0;
    }

    delay(50);
}