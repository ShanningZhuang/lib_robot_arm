#include <chrono>
#include <thread>
int main() {
  auto start = std::chrono::high_resolution_clock::now(); // 获取开始时间
  while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count() < 1000) {
    // while循环内的代码
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 等待100毫秒
  }
  return 0;
}