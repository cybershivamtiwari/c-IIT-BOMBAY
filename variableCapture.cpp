#include <simplecpp>
#include <functional>
int main(){
  int m=10;
  std::function<void()> f = [m](){cout << m << endl;};
  std::function<void()> g = [&m](){cout << m << endl;};

  m++;

  f();
  g();
}
