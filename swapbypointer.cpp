#include <simplecpp>
void swap(int* pa, int* pb){
  int temp;
  temp = *pa;
  *pa = *pb;
  *pb = temp;
}

main_program{
  int x=5, y=6;
  swap(&x,&y);
  cout << x <<" "<< y << endl;
}
