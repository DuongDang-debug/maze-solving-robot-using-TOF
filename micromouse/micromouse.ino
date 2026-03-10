
  #include <Wire.h>
  #include <Adafruit_VL53L0X.h>
  #include <math.h>
  #include "BluetoothSerial.h"
  BluetoothSerial SerialBT;


  // ======= 1. ENUM =======
  enum Heading { NORTH=0, EAST=1, SOUTH=2, WEST=3 };

  struct DistMM {
    int f, l, r;   // front, left, right
    bool f_ok, l_ok, r_ok;
  };

  #define SDA_pin 21
  #define SCL_pin 22

  // 3 chân XSHUT
  #define XSHUT1 4
  #define XSHUT2 23
  #define XSHUT3 19

  // địa chỉ mới cho 3 cảm biến
  #define ADDR_1 0x30
  #define ADDR_2 0x31
  #define ADDR_3 0x32

  #define C1_left 34
  #define C2_left 35
  #define C1_right 39
  #define C2_right 36

  #define PWMA 13
  #define AIN2 14
  #define AIN1 12
  #define STBY 27
  #define BIN1 26
  #define BIN2 25
  #define PWMB 33

  //--------------------------------------khởi tạo map--------------------------------------------

  const int MAZE_SIZE = 33;

      int maze[MAZE_SIZE][MAZE_SIZE] = {
    { 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1 },
    { 2, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 2 },
    { 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1 },
    { 2, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 2 },
    { 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1 },
    { 2, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 2 },
    { 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1 },
    { 2, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 2 },
    { 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1 },
    { 2, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 2 },
    { 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1 },
    { 2, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 2 },
    { 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1 },
    { 2, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 2 },
    { 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1 },
    { 2, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 2 },
    { 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1 },
    { 2, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 2 },
    { 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1 },
    { 2, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 2 },
    { 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1 },
    { 2, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 2 },
    { 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1 },
    { 2, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 2 },
    { 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1 },
    { 2, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 2 },
    { 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1 },
    { 2, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 2 },
    { 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1 },
    { 2, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 2 },
    { 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1 },
    { 2, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 2 },
    { 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1 }

  };

  //   int maze[MAZE_SIZE][MAZE_SIZE] = {
  //   { 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1 },
  //   { 2, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 2 },
  //   { 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1 },
  //   { 2, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 2 },
  //   { 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1 },
  //   { 2, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 2 },
  //   { 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1 },
  //   { 2, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 2 },
  //   { 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1 },
  //   { 2, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 2 },
  //   { 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1 },
  //   { 2, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 2 },
  //   { 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1 },
  //   { 2, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 2 },
  //   { 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1 },
  //   { 2, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 2 },
  //   { 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1 },
  //   { 2, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 2 },
  //   { 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1 },
  //   { 2, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 2 },
  //   { 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1 },
  //   { 2, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 2 },
  //   { 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1 },
  // };

  // int maze[MAZE_SIZE][21] = {
  //   { 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1 },
  //   { 2, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 2 },
  //   { 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1 },
  //   { 2, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 2 },
  //   { 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1 },
  //   { 2, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 2 },
  //   { 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1 },
  //   { 2, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 2 },
  //   { 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1 },
  //   { 2, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 2 },
  //   { 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1 },
  //   { 2, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 2 },
  //   { 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1 },
  //   { 2, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 2 },
  //   { 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1 },
  //   { 2, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 2 },
  //   { 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1 },
  //   { 2, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 2 },
  //   { 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1 },
  //   { 2, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 2 },
  //   { 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1, 5, 1 },
  //   { 2, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 4, 0, 2 },
  //   { 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1, 3, 1 },
  // };
  // 1 là o, 2 là |, 3 là ---, 4 là ?, 5 là ???

  // -----------------------------khai báo giải mê cung và vị trí robot---------------------------------


  int dist[MAZE_SIZE][MAZE_SIZE];
  int dx[4] = {-2, 0, 2, 0};
  int dy[4] = {0, 2, 0, -2};

  // =================== tự tạo queue====================
  struct Node {
    int x, y;
  };

  Node q[600];   // Hàng đợi
  int q_front = 0;
  int q_back = 0;

  void q_reset() {
    q_front = 0;
    q_back = 0;
  }

  bool q_empty() {
    return q_front == q_back;
  }

  void q_push(int x, int y) {
    q[q_back].x = x;
    q[q_back].y = y;
    q_back = (q_back + 1) % 600;
  }

  Node q_pop() {
    Node n = q[q_front];
    q_front = (q_front + 1) % 600;
    return n;
  }

  // -=========================================================
  int startDirection = 0;

  //đích số 1
  int goal_x1 = 1;
  int goal_y1 = 1;

  // đích số 2
  int goal_x2 = 7;
  int goal_y2 = 21;

  // ======= 2. PROTOTYPES =======
  bool wall_front();
  bool wall_left();
  bool wall_right();

  void update_frontWall(int cx, int cy, Heading h);
  void update_leftWall(int cx, int cy, Heading h);
  void update_rightWall(int cx, int cy, Heading h);

  // ======= 3. BIẾN =======
  Heading heading = SOUTH;

  // ======= 4. HÀM SENSOR =======
  bool wall_front() {
    DistMM d = read_sensor();
    return d.f_ok && d.f < 120;
  }

  bool wall_left() {
    DistMM d = read_sensor();
    return d.l_ok && d.l < 120;
  }

  bool wall_right() {
    DistMM d = read_sensor();
    return d.r_ok && d.r < 120;
  }

  // ======= 5. HÀM CẬP NHẬT WALL =======
  void update_frontWall(int cx, int cy, Heading h) {
      int mx = cx*2-1, my = cy*2-1;

      if(h==NORTH) maze[mx-1][my] = 3;
      else if(h==SOUTH) maze[mx+1][my] = 3;
      else if(h==EAST)  maze[mx][my+1] = 2;
      else if(h==WEST)  maze[mx][my-1] = 2;
  }

  void update_leftWall(int cx, int cy, Heading h) {
      int mx = cx*2-1, my = cy*2-1;
      Heading L = (Heading)((h+3)%4);

      if(L==NORTH) maze[mx-1][my] = 3;
      else if(L==SOUTH) maze[mx+1][my] = 3;
      else if(L==EAST)  maze[mx][my+1] = 2;
      else if(L==WEST)  maze[mx][my-1] = 2;
  }

  void update_rightWall(int cx, int cy, Heading h) {
      int mx = cx*2-1, my = cy*2-1;
      Heading R = (Heading)((h+1)%4);

      if(R==NORTH) maze[mx-1][my] = 3;
      else if(R==SOUTH) maze[mx+1][my] = 3;
      else if(R==EAST)  maze[mx][my+1] = 2;
      else if(R==WEST)  maze[mx][my-1] = 2;
  }

  // ======== hàm phát hiện có tường và vẽ==========
  void detect_update_maze(int cx, int cy, Heading h) {
      if (wall_front()) update_frontWall(cx, cy, h);
      if (wall_left())  update_leftWall(cx, cy, h);
      if (wall_right()) update_rightWall(cx, cy, h);
  }


  bool canMove(int x, int y, int nx, int ny) {

      // kiểm tra biên
      if (nx < 0 || ny < 0 || nx >= MAZE_SIZE || ny >= MAZE_SIZE)
          return false;

      // Tường nằm giữa ô hiện tại và ô đích
      int wallX = (x + nx) / 2;
      int wallY = (y + ny) / 2;

      // Nếu giữa 2 ô có tường (2,3) hoặc tường UNKNOWN (4,5) → KHÔNG ĐI
      int w = maze[wallX][wallY];
      if (w == 2 || w == 3)
          return false;

      // Ô đích phải là ô trống hợp lệ
      if (maze[nx][ny] != 0)
          return false;

      return true;
  }

  // floodfill từ vị trí cần đổ khi chưa có tường
  void floodFill(int startX, int startY) {
      memset(dist, -1, sizeof(dist));

      q_reset();
      dist[startX][startY] = 0;
      q_push(startX, startY);

      while (!q_empty()) {
          Node cur = q_pop();
          int x = cur.x;
          int y = cur.y;

          for (int d = 0; d < 4; d++) {
              int nx = x + dx[d];
              int ny = y + dy[d];

              if (nx < 0 || nx >= MAZE_SIZE || ny < 0 || ny >= MAZE_SIZE)
                continue;

              // if (maze[nx][ny] == 4 || maze[nx][ny] == 5)
              //   continue;

              if (!canMove(x, y, nx, ny))
                continue;

              if (dist[nx][ny] == -1) {
                  dist[nx][ny] = dist[x][y] + 1;
                  q_push(nx, ny);
              }
          }
      }
  }


  Heading chooseBestDirection(int cx, int cy, Heading h) {
      int mx = cx*2-1, my = cy*2-1;

      int best = 9999;
      Heading bestDir = h;

      for(int d=0;d<4;d++){
          int nx = mx + dx[d];
          int ny = my + dy[d];

          if(nx<0||ny<0||nx>=MAZE_SIZE||ny>=MAZE_SIZE) continue;
          if(!canMove(mx,my,nx,ny)) continue;

          // if(dist[nx][ny] < best && dist[nx][ny] >= 0){
          //     best = dist[nx][ny];
          //     bestDir = (Heading)d;
          // }
          // Chỉ xét các ô có giá trị dist hợp lệ
        if(dist[nx][ny] >= 0) {
            
            // Trường hợp 1: Tìm thấy đường ngắn hơn hẳn
            if(dist[nx][ny] < best){
                best = dist[nx][ny];
                bestDir = (Heading)d;
            }
            // Trường hợp 2: Khoảng cách BẰNG nhau, nhưng đây là hướng đi thẳng
            else if(dist[nx][ny] == best && (Heading)d == h){
                bestDir = (Heading)d;
            }
        }
      }
      return bestDir;
  }

  // void printMazeBT() {
  //     for (int i = 0; i < MAZE_SIZE; i++) {
  //         for (int j = 0; j < MAZE_SIZE; j++) {
  //             int cell = maze[i][j];

  //             if (cell == 1)        Serial.print("o");
  //             else if (cell == 2)   Serial.print("|");
  //             else if (cell == 3)   Serial.print("---");
  //             else if (cell == 4)   Serial.print("?");
  //             else if (cell == 5)   Serial.print("???");
  //             else if (cell == 0) {
  //                 int d = dist[i][j];
  //                 if (d < 0) Serial.print("   ");   // chưa reachable
  //                 else if (d < 10) {
  //                     Serial.print(" ");
  //                     Serial.print(d);
  //                     Serial.print(" ");
  //                 } else {
  //                     Serial.print(d);
  //                     Serial.print(" ");
  //                 }
  //             }
  //         }
  //         Serial.println();
  //     }
  //     Serial.println();
  // }

void printMazeBT() {
      for (int i = 0; i < MAZE_SIZE; i++) {
          for (int j = 0; j < MAZE_SIZE; j++) {
              int cell = maze[i][j];

              if (cell == 1)        SerialBT.print("o");
              else if (cell == 2)   SerialBT.print("|");
              else if (cell == 3)   SerialBT.print("---");
              else if (cell == 4)   SerialBT.print("?");
              else if (cell == 5)   SerialBT.print("???");
              else if (cell == 0) {
                  int d = dist[i][j];
                  if (d < 0) SerialBT.print("   ");   // chưa reachable
                  else if (d < 10) {
                      SerialBT.print(" ");
                      SerialBT.print(d);
                      SerialBT.print(" ");
                  } else {
                      SerialBT.print(d);
                      SerialBT.print(" ");
                  }
              }
          }
          SerialBT.println();
      }
      SerialBT.println();
  }

  const int buttonPin = 32;  // Chân nút nhấn D32

  // tạo 3 object VL53L0X
  Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
  Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
  Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();

  // số xung ở vị trí ban đầu
  volatile int encoder1_value = 0;
  volatile int encoder2_value = 0;

  volatile int angle1 = 0;
  volatile int angle2 = 0;

  float Kp;
  float Kd;
  float Ki;

  float last_z2;
  float last_z3;

  long prevT = 0; 
  float eprev = 0;
  float eintegral = 0;

  int speedA;
  int speedB;
  int BASED_SPEED1 = 135;
  int BASED_SPEED2 = 145;

  // khai báo 2 hàm liên quan đến tốc độ bánh xe
  int target2;
  int target1;
  int remaining1;
  int remaining2;
  float overshoot = 0;

  int n;

  // biến đánh dấu đến đích và chuyển vùng floodfill
  int m =0;

  // biến đánh dấu có 2 tường để PID cho 1 tường
  int both_wall; 

  // biến khoảng cách xe đến 1 tường
  int wall_only = 70;

  // đếm số bước chạy
  int leg_count = 0;
  bool is_speed_run = false; // Cờ báo hiệu chạy nhanh

  // stop motor khi gặp lỗi
  volatile int button_state_count = 0; // 0: Chờ khởi động, 1: Đang chạy, 2: Tạm dừng/Chờ reset, 3: Chạy lại...
  bool is_emergency_stop = false;      // Cờ báo dừng khẩn cấp
  
  // vị trí ban đầu robot
  int cx = 1, cy = 1;     // vị trí cell logic
  int count_goal = 0;
  int finish = 0;

  // -------------------------------------------------------------------------------------------

  void set_motor(int LPWM, int RPWM) {
    digitalWrite(STBY, HIGH);
    if(LPWM > 0) {
      digitalWrite(AIN1, HIGH);
      digitalWrite(AIN2, LOW);
    }
    else if(LPWM < 0) {
      digitalWrite(AIN1, LOW);
      digitalWrite(AIN2, HIGH);
    }
    analogWrite(PWMA, abs(LPWM));
    if(RPWM > 0) {
      digitalWrite(BIN1, HIGH);
      digitalWrite(BIN2, LOW);
    }
    else if(RPWM < 0) {
      digitalWrite(BIN1, LOW);
      digitalWrite(BIN2, HIGH);
    }
    analogWrite(PWMB, abs(RPWM));
  }

  void motorStop() {
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);
    digitalWrite(STBY, LOW);
  }

  //mode: rising
  void encoder1_isr() {
    int B = digitalRead(C2_left);
    int A = digitalRead(C1_left);
    if(B == LOW && A == HIGH || B == HIGH && A == LOW) {
      encoder1_value++;
    }
    else{
      encoder1_value--;
    }
  }

  // mode: Rising
  void encoder2_isr() {
    int B = digitalRead(C2_right);
    int A = digitalRead(C1_right);
    if(B == LOW && A == HIGH || B == HIGH && A == LOW) {
      encoder2_value++;
    }
    else{
      encoder2_value--;
    }
  }

  // ---------------------------------------------------------------------------------------

  // Đọc tổng xung quay (ổn định hơn dùng 1 bánh)
  static inline int spinCount() {
    noInterrupts();
    int e1 = encoder1_value;
    int e2 = encoder2_value;
    interrupts();
    return (abs(e1) + abs(e2)) / 2;
  }

  // Reset 2 encoder
  static inline void reset_encoder() {
    noInterrupts();
    encoder1_value = 0;
    encoder2_value = 0;
    interrupts();
  }

  // Áp dụng PWM theo hướng quay tại chỗ: dir = +1 (quay trái), -1 (quay phải)
  static inline void spinSetMotor(int dir, int pwm_mag) {
    pwm_mag = constrain(pwm_mag, 0, 255);
    int L = (dir > 0) ? +pwm_mag : -pwm_mag; // dir>0: trái tiến
    int R = (dir > 0) ? -pwm_mag : +pwm_mag; // dir>0: phải lùi
    set_motor(L, R);
  }

  // -------------------------------------------xoay góc----------------------------------------------
  // Hồ sơ vận tốc + phanh chủ động (blocking ~1–2s tùy tune)
  // ===== TURN V3 – MƯỢT NHẤT, CHUẨN MICROMOUSE =====

  #define TURN_90_COUNTS    820        // calibrate, lucs ddaauf laf 830
  #define TURN_MAX_PWM      240        // lực tối đa khi quay
  #define TURN_MIN_PWM       100        // vượt ma sát
  #define TURN_ACCEL      20000.0f     // gia tốc
  #define TURN_DECEL      22000.0f     // giảm tốc
  #define TURN_MAX_SPEED   8500.0f     // tốc độ đỉnh (counts/s)
  #define TURN_SETTLE_MS     20        // dừng ổn định

  static void turn_90_profiled(int dir) {
    reset_encoder();

    unsigned long last_us = micros();
    float speed = 0;  // tốc độ hiện tại (counts/s)

    while (true) {
        // dt
        unsigned long now = micros();
        float dt = (now - last_us) * 1e-6f;
        if (dt <= 0) dt = 0.001f;
        last_us = now;

        // tiến độ
        int C = spinCount();
        int rem = TURN_90_COUNTS - C;

        if (rem <= 0) break;  // hoàn thành

        // tăng/giảm tốc S-curve
        float v_accel = speed + TURN_ACCEL * dt;
        float v_brake = sqrtf(fmaxf(0, 2.0f * TURN_DECEL * rem));
        float v_cmd = fminf(TURN_MAX_SPEED, fminf(v_accel, v_brake));
        speed = v_cmd;

        // map tốc độ -> PWM
        int pwm = TURN_MIN_PWM + (int)((TURN_MAX_PWM - TURN_MIN_PWM) * (v_cmd / TURN_MAX_SPEED));
        pwm = constrain(pwm, TURN_MIN_PWM, TURN_MAX_PWM);

        // điều khiển quay tại chỗ bằng vi sai encoder để không lệch góc
        int eL, eR;

        noInterrupts();
        eL = encoder1_value;
        eR = encoder2_value;
        interrupts();

        int diff_err = (eL + eR);   // hiệu giữa 2 bánh
        int diff_fix = diff_err * 0.1f;  // hệ số hiệu chỉnh nhỏ

        int L = dir * pwm + diff_fix;
        int R = -dir * pwm - diff_fix;

        set_motor(L, R);

        delay(4);   // chu kỳ mềm, mượt
    }

    // stop mềm
    set_motor(0, 0);
    delay(TURN_SETTLE_MS);  // ổn định góc
    reset_encoder();
}


  void turn_left_90()  { turn_90_profiled(+1); }
  void turn_right_90() { turn_90_profiled(-1); }

  // ------------------------------------xoay góc theo hướng tốt nhất-------------------------

  void back(Adafruit_VL53L0X &sensorFront, int pwm_back = 50, int dist_stop = 40, int timeout_ms = 800) {
    VL53L0X_RangingMeasurementData_t meas;
    unsigned long t_start = millis();

    while (millis() - t_start < timeout_ms) {
      sensorFront.rangingTest(&meas, false);

      // Nếu đo hợp lệ
      if (meas.RangeStatus == 0) {
        int d = meas.RangeMilliMeter;

        // Nếu còn gần hơn ngưỡng -> tiếp tục lùi
        if (d < dist_stop) {
          set_motor(-pwm_back, -pwm_back);
        } 
        // Khi đã đủ xa thì dừng lại
        else {
          break;
        }
      } 
      else {
        // Nếu cảm biến lỗi tạm thời => cứ lùi nhẹ
        set_motor(-pwm_back, -pwm_back);
      }

      delay(10);  // nhịp đọc 100 Hz
    }

    motorStop();
    delay(10); // nghỉ nhỏ để tránh dao động
    reset_encoder();
  }

  DistMM read_sensor() {
    VL53L0X_RangingMeasurementData_t laser1, laser2, laser3;
    // sensor 1
    lox1.rangingTest(&laser1, false);
    // sensor 2
    lox2.rangingTest(&laser2, false);
    // sensor 3
    lox3.rangingTest(&laser3, false);

    DistMM d;
    d.f = laser1.RangeMilliMeter; d.f_ok = (laser1.RangeStatus == 0);
    d.l = laser2.RangeMilliMeter; d.l_ok = (laser2.RangeStatus == 0);
    d.r = laser3.RangeMilliMeter; d.r_ok = (laser3.RangeStatus == 0);
    return d;
  }


  // lùi đạp vào tường để cân bằng
  void back_wall(int count) {
    reset_encoder();
    while(abs(spinCount()) < count) {
      set_motor(-175, -190);
    }
    motorStop();
    delay(50);
    reset_encoder();
  }

  void rotate(Heading &h, Heading target) {
      int diff = (target - h + 4) % 4;

      DistMM d = read_sensor();
      bool left_wall  = d.l_ok && d.l < 120;
      bool right_wall = d.r_ok && d.r < 120;
      bool front_wall = d.f_ok && d.f < 120;

      if(diff == 1) {
        turn_right_90();
        // if (left_wall) {
        //   back_wall(1300); 
        //   // 1500
        //   n = 1;  // đạp tường trái
        // }
      }      
      else if(diff == 2) { 
        turn_left_90(); 
        DistMM d = read_sensor();
        if (d.f_ok && d.f < 45) {
          back(lox1, 50, 45, 800);
        }
        turn_left_90(); 
        // if (front_wall) {
        //   back_wall(1300);
        //   n = 1;
        // }
        }
      else if(diff == 3) {
        turn_left_90();
        // if (right_wall) {
        //   back_wall(1300); 
        //   n = 1;  // đạp tường trái
        // }
      }
      //quay xong mới đo lại phía trước
      // DistMM d = read_sensor();
      // if (d.f_ok && d.f < 45) {
      //   back(lox1, 50, 45, 800);
      // }
      h = target;
  }


  void go_one_cell() {
    if(n == 0) {
      target2 = 2400;
      // lúc đầu 2440
    }
    else {
      target2 = 2950;
    }
    int average = spinCount();

    remaining2 = target2 - average;
    Serial.println(remaining2);

    // if (remaining2 > 1700) {
    //   BASED_SPEED1 = 100;   
    //   BASED_SPEED2 = 110;
    //   Kp = 1.4;
    //   Kd = 1.8;
    // } 

    // else if (remaining2 > 1500) {
    //   BASED_SPEED1 = 150;   
    //   BASED_SPEED2 = 160;
    //   Kp = Kp + 0.043;
    //   Kd = Kd - 0.043;
    // } 

    if (remaining2 > 1200) {
      BASED_SPEED1 = 230;   
      BASED_SPEED2 = 240;
      // Kp = 1.4;  
      // // lucs daufd 1.35
      // Kd = 1.8;
      Kp = 2.9;
      Kd = 4.35;
      if(leg_count == 1) {
        Kp = 3;
        Kd = 4.5;
      }
    } 
    // else if (remaining2 > 1100) {
    //   BASED_SPEED1 = 205;   
    //   BASED_SPEED2 = 220;
    //   Kp = Kp + 0.035;
    //   Kd = Kd - 0.035;
    // } 
    else if (remaining2 > 1100) {
      // BASED_SPEED1 = 195;   
      // BASED_SPEED2 = 210;
      Kp = Kp + 0.08;
      Kd = Kd - 0.08;
    } 
    else if (remaining2 > 1000) {
      // BASED_SPEED1 = 185;   
      // BASED_SPEED2 = 200;
     Kp = Kp + 0.08;
      Kd = Kd - 0.08;
    }  
    else if (remaining2 > 900) {
      // BASED_SPEED1 = 175;   
      // BASED_SPEED2 = 190;
     Kp = Kp + 0.08;
      Kd = Kd - 0.08;
    }  
    else if (remaining2 > 800) {
      // BASED_SPEED1 = 165;   
      // BASED_SPEED2 = 180;
      Kp = Kp + 0.08;
      Kd = Kd - 0.08;
    }  
    else if (remaining2 > 700) {
      // BASED_SPEED1 = 155;   
      // BASED_SPEED2 = 170;
     Kp = Kp + 0.08;
      Kd = Kd - 0.08;
    }  
    else if (remaining2 > 600) {
      // BASED_SPEED1 = 145;   
      // BASED_SPEED2 = 160;
      Kp = Kp + 0.08;
      Kd = Kd - 0.08;
    }  
    else if (remaining2 > 500) {
      // BASED_SPEED1 = 135;   
      // BASED_SPEED2 = 150;
      Kp = Kp + 0.08;
      Kd = Kd - 0.08;
    }  
    else if (remaining2 > 400) {
      // BASED_SPEED1 = 120;   
      // BASED_SPEED2 = 135;
      Kp = Kp + 0.08;
      Kd = Kd - 0.08;
    }   
    else if (remaining2 > 300) {
      BASED_SPEED1 = 95;   
      BASED_SPEED2 = 110;
      Kp = Kp + 0.08;
      Kd = Kd - 0.08;
    } 
    else if (remaining2 > 200) {
      BASED_SPEED1 = 80;   
      BASED_SPEED2 = 95;
      Kp = Kp + 0.08;
      Kd = Kd - 0.08;
    } 
    else if (remaining2 > 100) {
      BASED_SPEED1 = 65;   
      BASED_SPEED2 = 80;
      Kp = Kp + 0.08;
      Kd = Kd - 0.08;
    } 
    else if (remaining2 > 50) {
      BASED_SPEED1 = 50;  
      BASED_SPEED2 = 65;
      Kp = Kp + 0.08;
      Kd = Kd - 0.08;
    } 
    else if (remaining2 > 20) {
      BASED_SPEED1 = 25;  
      BASED_SPEED2 = 40;
      Kp = Kp + 0.08;
      Kd = Kd - 0.08;
    }
    else if (remaining2 > 10) {
      BASED_SPEED1 = 10;  
      BASED_SPEED2 = 25;
      Kp = Kp + 0.08;
      Kd = Kd - 0.08;
    }
    else {
      BASED_SPEED1 = 10;  
      BASED_SPEED2 = 25;
      Kp = Kp + 0.08;
      Kd = Kd - 0.08;
    }
  }

  // ----------------------------------đi tiến và bám tường PID-----------------------------
  void go_PID(const DistMM& d) {
      go_one_cell();
        if( d.l_ok && d.r_ok && d.l < 120 && d.r < 120) {
        // Kp = 1.5;
        // Kd = 2;
        // // Ki = 0.01;
        float e = d.l - d.r;
        float dedt = e - eprev;
        eintegral += e;

        if (abs(e) < 10 && d.l > 20 && d.r > 20) { 
          last_z2 = d.l; // Lưu khoảng cách trái
          last_z3 = d.r; // Lưu khoảng cách phải
          both_wall = 1; // Bật cờ "đã chốt"
        }

        float u = Kp * e + Kd * dedt + Ki * eintegral;
        
        int correction = constrain((int)u, -50, 50);
        speedA = constrain(BASED_SPEED1 + correction, -255, 255);
        speedB = constrain(BASED_SPEED2 - correction, -255, 255);
        set_motor(speedA, speedB);
        eprev = e;

        // last_z2 = d.l;
        // last_z3 = d.r;
        // both_wall = 1;
      }
      else if(d.r_ok && d.r < 100) {
        // Kp = 1.5;
        // Kd = 2;
        // // Ki = 0.01;
        float e;
        if(both_wall == 0) {
          e = wall_only - d.r;
        }
        else {
          e = last_z3 - d.r;
        }
        float dedt = e - eprev;
        // eintegral += e*deltaT;

        float u = Kp * e + Kd * dedt + Ki * eintegral;
        
        int correction = constrain((int)u, -50, 50);
        speedA = constrain(BASED_SPEED1 + correction, -255, 255);
        speedB = constrain(BASED_SPEED2 - correction, -255, 255);
        set_motor(speedA, speedB);
        eprev = e;
      }
      else if(d.l_ok && d.l < 100) {
        // Kp = 1.5;
        // Kd = 2;
        // // Ki = 0.01;
        float e;
        if (both_wall == 0) {
          e = wall_only - d.l;
        }
        else {
          e = last_z2 - d.l;
        }
        float dedt = e - eprev;
        // eintegral += e*deltaT;

        float u = Kp * e + Kd * dedt + Ki * eintegral;
        
        int correction = constrain((int)u, -50, 50);
        speedA = constrain(BASED_SPEED1 - correction, -255, 255);
        speedB = constrain(BASED_SPEED2 + correction, -255, 255);
        set_motor(speedA, speedB);
        eprev = e;
      }
      else {
        noInterrupts();
        int eL = abs(encoder1_value);
        int eR = abs(encoder2_value);
        interrupts();
        Kp = 0.2;
        Kd = 0.35;
        float e = eL - eR;
        float dedt = e - eprev;
        float u = Kp * e + Kd * dedt;

        int correction = constrain((int)u, -50, 50);
        speedA = constrain(BASED_SPEED1 + correction, -255, 255);
        speedB = constrain(BASED_SPEED2 - correction, -255, 255);
        set_motor(speedA, speedB);
        eprev = e;
    }
  }

  void reset_PID_vars() {
    eprev = 0;
    eintegral = 0;
    // Reset các biến control khác nếu cần
}

  void move_one_cell() {
  reset_encoder();
  reset_PID_vars();
  if (n == 0) {
    target2 = 2470;
  }  
  else {
    target2 = 2950;
  }

  both_wall = 0; 

  while (spinCount() < target2) {
    DistMM d = read_sensor();
    go_PID(d);
  }

  motorStop();
  delay(1);
  n = 0;
  DistMM d = read_sensor();
  if (d.f_ok && d.f< 50) {
    back(lox1, 50, 45, 800);
  }
}

  // =================== Xử lý Nút Nhấn Mới =====================

void check_button_interrupt() {
    // Logic debounce đơn giản
    static unsigned long last_button_press = 0;
    if (digitalRead(buttonPin) == LOW && (millis() - last_button_press > 200)) {
        last_button_press = millis();
        
        // Tăng biến đếm trạng thái
        button_state_count++; 
        SerialBT.print("Button Press Count: ");
        SerialBT.println(button_state_count);

        if (is_speed_run && (button_state_count % 2 != 0)) {
            // Lần bấm LẺ khi đang chạy Speed Run: Dừng khẩn cấp
            is_emergency_stop = true;
            motorStop();
            SerialBT.println("!!! EMERGENCY STOP: Robot Stopped !!!");
        } else if (is_speed_run && (button_state_count % 2 == 0)) {
            // Lần bấm CHẴN khi đang dừng (sau lần lẻ): Reset & Khởi động lại Speed Run
            is_emergency_stop = false;
            
            // 1. Reset vị trí logic
            cx = 1; cy = 1;
            heading = SOUTH;
            
            // 2. Reset trạng thái Speed Run
            m = 0; // Đặt mục tiêu là goal_x2/y2 (đích)
            floodFill(goal_x2, goal_y2);
            
            // 3. Reset Encoder và PID
            reset_encoder();
            // Khởi động lại PID values (nếu cần thiết)
            Kp = 1.5; 
            Kd = 1.7; 

            SerialBT.println("!!! RESUMING SPEED RUN from START !!!");
        }
    }
}

    void setup() {
    Serial.begin(9600);
    SerialBT.begin("MicromouseESP32");
    Serial.println("Bluetooth SPP started!");

    delay(100);
    Wire.begin(SDA_pin, SCL_pin);
    Wire.setClock(400000); // thêm dòng này

    // cấu hình chân XSHUT
    pinMode(XSHUT1, OUTPUT);
    pinMode(XSHUT2, OUTPUT);
    pinMode(XSHUT3, OUTPUT);

    pinMode(buttonPin, INPUT_PULLUP);

    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(STBY, OUTPUT);
    pinMode(PWMA, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);

    pinMode(C1_left, INPUT);
    pinMode(C2_left, INPUT);
    pinMode(C1_right, INPUT);
    pinMode(C2_right, INPUT);

    attachInterrupt(digitalPinToInterrupt(C1_left), encoder1_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(C1_right), encoder2_isr, CHANGE);

    // tắt hết cảm biến
    digitalWrite(XSHUT1, LOW);
    digitalWrite(XSHUT2, LOW);
    digitalWrite(XSHUT3, LOW);
    delay(10);

    // KHỞI TẠO TỪNG BIẾN
    // cảm biến 1
    digitalWrite(XSHUT1, HIGH);
    delay(10);
    if(!lox1.begin(0x29, &Wire)) {
      Serial.println("fail init sensor 1");
      while(1);
    }
    lox1.setAddress(ADDR_1);

    //cảm biến 2
    digitalWrite(XSHUT2, HIGH);
    delay(10);
    if(!lox2.begin(0x29, &Wire)) {
      Serial.println("fail init sensor 2");
      while(1);
    }
    lox2.setAddress(ADDR_2);

    // cảm biến 3
    digitalWrite(XSHUT3, HIGH);
    delay(10);
    if(!lox3.begin(0x29, &Wire)) {
      Serial.println("fail init sensor 3");
      while(1);
    }
    lox3.setAddress(ADDR_3);

  lox1.setMeasurementTimingBudgetMicroSeconds(20000);
  lox2.setMeasurementTimingBudgetMicroSeconds(20000);
  lox3.setMeasurementTimingBudgetMicroSeconds(20000);



  }

  void loop() {

      // Gọi hàm kiểm tra nút để xử lý bấm lần lẻ/chẵn
    check_button_interrupt(); 
    
    // Nếu đang ở trạng thái dừng khẩn cấp (bấm nút lần lẻ trong Speed Run)
    if (is_emergency_stop) {
        motorStop();
        delay(10); // Cho hệ thống thở
        return;    // Thoát khỏi vòng lặp, chờ lần bấm nút tiếp theo
    }
      // đánh dấu ô đang đứng
      // maze[cx*2][cy*2] = 0;

      while (count_goal == 0) {
        DistMM d = read_sensor();

        if (d.l_ok && d.l < 50 && d.f_ok && d.f < 50) {
          goal_x1 = 1;
          goal_y1 = 1;
          cx = 1;
          cy = 1;
          goal_x2 = 17;
          goal_y2 = 17;
          count_goal = 1;
        }

        // cảm biến trái goal = (1,1)
        else if (d.l_ok && d.l < 50 ) {
          goal_x1 = 1;
          goal_y1 = 1;
          cx = 1;
          cy = 1;
          goal_x2 = 15;
          goal_y2 = 17;
          count_goal = 1;
        }

        else if (d.r_ok && d.r < 50 && d.f_ok && d.f < 50) {
          goal_x1 = 1;
          goal_y1 = 31;
          cx = 1;
          cy = 16;
          goal_x2 = 17;
          goal_y2 = 15;
          count_goal = 1;
        }

        // cảm biến phải goal = (1,16)
        else if (d.r_ok && d.r < 50) {
          goal_x1 = 1;
          goal_y1 = 31;
          cx = 1;
          cy = 16;
          goal_x2 = 15;
          goal_y2 = 15;
          count_goal = 1;
        }
      }

    if (leg_count == 1 && finish == 0) {
      motorStop();
     
     // Vòng lặp chờ nhấn nút
     while (digitalRead(buttonPin) == HIGH) {
        delay(100); // Chờ
     }

    //  nhấn lần 1
    button_state_count = 1;
     
     // Đã nhấn nút!
     Serial.println("BUTTON PRESSED! SPEED RUN START!");
     delay(1000); // Delay an toàn
     
     // Cấu hình lại cho Speed Run
     is_speed_run = true;

     cx = 1;
      cy = 1;
    //  leg_count = 0; // Reset đếm để chạy tiếp (hoặc logic khác tùy ý)
     m = 0; // Mục tiêu là đích
     floodFill(goal_x2, goal_y2); // Tính đường ngắn nhất trên map đã có
     Kp = 1.5;
     Kd = 1.7;
     heading = SOUTH;
     finish = 1;
  }


      // cập nhật tường
      // if (leg_count != 1) {
      //   detect_update_maze(cx, cy, heading);
      // }
      detect_update_maze(cx, cy, heading);

      // floodfill từ go
        floodFill(goal_x2, goal_y2);
      // else if(m == 1) {
      //   floodFill(goal_x1, goal_y1);
      // }
      // chọn hướng tối ưu
      Heading target = chooseBestDirection(cx, cy, heading);

      // DistMM d;
      // if(leg_count != 4) {
      //   d = read_sensor();
      // }
      // DistMM d = read_sensor();
      // xoay robot
      rotate(heading, target);

      move_one_cell();

      // cập nhật vị trí cell
      // cập nhật vị trí cell (đúng)
      if (heading == NORTH) cx--;
      else if (heading == SOUTH) cx++;
      else if (heading == EAST)  cy++;
      else if (heading == WEST)  cy--;


      // thêm: check goal
      if(cx*2-1 == goal_x2 && cy*2-1 == goal_y2) {
          Serial.println("Reached Goal!");
          // m = 1;
          leg_count++;
      }
      // else if(cx*2-1 == goal_x1 && cy*2-1 == goal_y1) {
      //     m = 0;
      //     leg_count++;
      // }
    

}
