#include <simplecpp>
int main(){
  int val, sum=0;
  while(cin >> val){     // file read in the loop test
    sum += val;
  }
  cout << sum << endl;
}
