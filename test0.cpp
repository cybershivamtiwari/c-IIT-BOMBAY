#include <simplecpp>

int main(){
  sim::post(15, [](){sim::log() << "Event A.\n";});
  sim::post(5, [](){sim::log() << "Event B.\n";});

  sim::processAll();
}
