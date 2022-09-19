#include <simplecpp>
float sum(float* v, int n){  // function to sum the elements of an array
  float s = 0;
  for(int i=0; i<n; i++) 
    s += v[i];
      
  return s;
}
int main(){
  float a[10] = {1.0, 2.0, 3.0, 4.0, 5.0};
  cout << sum(a, 5) << endl;
}
