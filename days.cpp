#include <simplecpp>
main_program{
  int month;
  cin >> month;
  switch(month){
    case 1:  // January
    case 3:  // March
    case 5:  // May
    case 7:  // July
    case 8:  // August
    case 10: // October
    case 12: // December
             cout << "This month has 31 days.\n";
             break;
    case 2:  // February
             cout << "This month has 28 or 29 days.\n";
             break;
    case 4:  // April
    case 6:  // June
    case 9:  // September
    case 11: // November
             cout << "This month has 30 days.\n";
             break;
    default: cout << "Invalid input.\n";
  }
}
