/***************************************************************
   Các hàm và định nghĩa kiểu cho điều khiển PID.

   Lấy từ mã của Mike Ferguson trên ArbotiX, mã nguồn có tại:
   
   http://vanadium-ros-pkg.googlecode.com/svn/trunk/arbotix/
*/

/* Thông tin setpoint PID cho một động cơ */
typedef struct {
  double TargetTicksPerFrame;    // tốc độ mục tiêu tính bằng ticks mỗi khung hình
  long Encoder;                  // giá trị encoder hiện tại
  long PrevEnc;                  // giá trị encoder trước đó

  float loitr;                   // Lỗi trước đó
  float iloi;                    // Lỗi tích lũy
  long output;                   // Giá trị điều khiển động cơ
}
SetPointInfo;

SetPointInfo leftPID, rightPID;

/* Các tham số PID */
float Kp_high =  20, Ki_high = 1, Kd_high = 12;

unsigned char moving = 0;
unsigned long tgtrc = 0; 


void resetPID() {
   leftPID.TargetTicksPerFrame = 0.0;
   leftPID.Encoder = readEncoder(LEFT);
   leftPID.PrevEnc = leftPID.Encoder;
   leftPID.output = 0;
   leftPID.loitr = 0;
   leftPID.iloi = 0;

   rightPID.TargetTicksPerFrame = 0.0;
   rightPID.Encoder = readEncoder(RIGHT);
   rightPID.PrevEnc = rightPID.Encoder;
   rightPID.output = 0;
   rightPID.loitr = 0;
   rightPID.iloi = 0;
}

/* Hàm PID đơn giản để tính toán giá trị điều khiển */
float simplePID(float loi, float &loitr, float &iloi, float kp, float ki, float kd) {
  float dloi = loi - loitr;
  iloi += loi;

  // Giới hạn tích phân
  if (iloi > 100) iloi = 100;
  if (iloi < -100) iloi = -100;

  loitr = loi;

  float temp = kp * loi + ki * iloi + kd * dloi;

  // Giới hạn đầu ra
  if (temp > 255) temp = 255;
  if (temp < -255) temp = -255;

  return temp;
}

/* Hàm PID để tính toán các lệnh điều khiển động cơ tiếp theo */
void doPID(SetPointInfo *p) {
  int tocdo = p->Encoder - p->PrevEnc;
  int loi = p->TargetTicksPerFrame - tocdo;

  float kp, ki, kd;
  kp = Kp_high; ki = Ki_high; kd = Kd_high;
  p->output += simplePID(loi, p->loitr, p->iloi, kp, ki, kd);

  // Giới hạn giá trị đầu ra cuối cùng
  if (p->output > 255) p->output = 255;
  if (p->output < -255) p->output = -255;
}

/* Đọc các giá trị encoder và gọi hàm PID */
void updatePID() {

    /* Đọc các giá trị encoder */
    leftPID.Encoder = readEncoder(LEFT);
    rightPID.Encoder = readEncoder(RIGHT);

    /* Nếu không di chuyển thì không cần làm gì thêm */
    if (!moving) {
      if (leftPID.loitr != 0 || rightPID.loitr != 0) resetPID();
      return;
    }

    /* Tính toán cập nhật PID cho mỗi động cơ */
    doPID(&rightPID);
    doPID(&leftPID);

    /* Cập nhật lại giá trị encoder trước đó để đảm bảo tốc độ đúng */
    leftPID.PrevEnc = leftPID.Encoder;
    rightPID.PrevEnc = rightPID.Encoder;

    /* Đặt tốc độ động cơ theo giá trị PID */
    setMotorSpeeds(leftPID.output, rightPID.output);
  }