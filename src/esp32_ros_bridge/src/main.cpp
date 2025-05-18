/* Tốc độ baud của cổng Serial */
#define BAUDRATE     115200

/* Tín hiệu PWM tối đa */
#define MAX_PWM        255

#include "commands.h"
#include "motor_driver.h"
#include "encoder_driver.h"
#include "diff_controller.h"

/* Chạy vòng lặp PID 100 lần mỗi giây */
#define PID_RATE           100     // Hz

/* Chuyển tốc độ PID sang khoảng thời gian tính bằng mili giây */
const int PID_INTERVAL = 1000 / PID_RATE;

/* Biến theo dõi thời điểm tiếp theo để tính PID */
unsigned long nextPID = PID_INTERVAL;

/* Dừng robot nếu không nhận được lệnh di chuyển
   trong khoảng thời gian tính bằng mili giây này */
#define AUTO_STOP_INTERVAL 10000
long lastMotorCommand = AUTO_STOP_INTERVAL;

/* Khởi tạo biến */

// Cặp biến dùng để phân tích cú pháp lệnh Serial (cảm ơn Fergs)
int argu = 0;
int idx = 0;

// Biến lưu ký tự đầu vào
char chr;

// Biến lưu ký tự lệnh (một ký tự duy nhất)
char cmd_in;

// Mảng ký tự lưu tham số thứ nhất và thứ hai
char arguv1[16];
char arguv2[16];

// Tham số được chuyển đổi thành số nguyên
int arg1;
int arg2;

/* Xóa các tham số lệnh hiện tại */
void resetCommand() {
  cmd_in = '\0'; 
  memset(arguv1, 0, 16);
  memset(arguv2, 0, 16);
  arg1 = 0;
  arg2 = 0;
  argu = 0;
  idx = 0;
}

/* Thực thi lệnh. Các lệnh được định nghĩa trong commands.h */
void runCommand() {
  arg1 = atoi(arguv1);
  arg2 = atoi(arguv2);

  switch (cmd_in) {
    case GET_BAUDRATE:
      Serial.println(BAUDRATE);
      break;
    case PING:
      Serial.println("0");
      break;
    case READ_ENCODERS:
      Serial.print(readEncoder(LEFT));
      Serial.print(" ");
      Serial.println(readEncoder(RIGHT));
      break;
    case RESET_ENCODERS:
      resetEncoders();
      resetPID();
      Serial.println("OK");
      break;
    case MOTOR_SPEEDS:
      /* Đặt lại bộ đếm thời gian tự động dừng */
      lastMotorCommand = millis();
      if (arg1 == 0 && arg2 == 0) {
        setMotorSpeeds(0, 0);
        resetPID();
        moving = 0;
      }
      else moving = 1;
      leftPID.TargetTicksPerFrame = arg1;
      rightPID.TargetTicksPerFrame = arg2;
      Serial.println("OK");
      break;
    case MOTOR_RAW_PWM:
      /* Đặt lại bộ đếm thời gian tự động dừng */
      lastMotorCommand = millis();
      resetPID();
      moving = 0; // Một cách "lách luật" để tạm tắt PID
      setMotorSpeeds(arg1, arg2);
      Serial.println("OK");
      break;
    default:
      Serial.print("Lệnh không hợp lệ: ");
      Serial.println(cmd_in);
      break;
  }
}

/* Hàm setup — chỉ chạy một lần khi khởi động */
void setup() {
  Serial.begin(BAUDRATE);

  // Khởi tạo encoder và điều khiển động cơ nếu sử dụng
  initEncoder();
  initMotorController();
  resetPID();
}

/* Vòng lặp chính — đọc và phân tích dữ liệu từ cổng Serial,
   và thực thi các lệnh hợp lệ. Thực hiện tính PID định kỳ
   và kiểm tra điều kiện tự động dừng. */
void loop() {
  
  while (Serial.available() > 0) {
    // Đọc ký tự tiếp theo
    chr = Serial.read();
    // Kết thúc một lệnh bằng ký tự CR (carriage return)
    if (chr == 13) {
      if (argu == 1) arguv1[idx] = '\0'; // Thay NULL bằng '\0'
      else if (argu == 2) arguv2[idx] = '\0'; // Thay NULL bằng '\0'
      runCommand();
      resetCommand();
    }
    // Dùng khoảng trắng để phân tách các phần của lệnh
    else if (chr == ' ') {
      // Chuyển sang đọc tham số
      if (argu == 0) argu = 1;
      else if (argu == 1)  {
        arguv1[idx] = '\0'; // Thay NULL bằng '\0'
        argu = 2;
        idx = 0;
      }
      continue;
    }
    else {
      if (argu == 0) {
        // Tham số đầu tiên là ký tự lệnh
        cmd_in = chr;
      }
      else if (argu == 1) {
        // Tham số sau có thể gồm nhiều ký tự
        arguv1[idx] = chr;
        idx++;
      }
      else if (argu == 2) {
        arguv2[idx] = chr;
        idx++;
      }
    }
  }

  // Nếu đang dùng điều khiển cơ sở, tính PID đúng khoảng thời gian đã định
  if (millis() > nextPID) {
    updatePID();
    nextPID += PID_INTERVAL;
  }

  // Kiểm tra xem đã quá thời gian tự động dừng chưa
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {
    setMotorSpeeds(0, 0);
    moving = 0;
  }
}
