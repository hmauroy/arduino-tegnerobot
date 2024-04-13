/*
 * A4988 functions abstracted away
 */

#include <math.h>  // Include the math library for trigonometric and sqrt functions
#define PI 3.1415926535897932384626433832795


// Function to calculate the curve length of a sine wave between x = a and x = b
// chatGPT4 12.04.2024
double calculateLengthSinewave(double a, double b, int n, double A, double c, double phi) {
  double dx = (b - a) / n;  // Step size
  double x = a;
  double sum = 0.0;
  double f_x = 0;

  // Calculate the sum of sqrt(1 + cos^2(x)) at each interval
  for (int i = 0; i <= n; ++i) {
    f_x = sqrt(1 + c*c * A*A * cos(c*x + phi) * cos(c*x + phi));  // Function value at x
    x += dx;
    if (x == x0) {
      sum += f_x * dx/2.0;  // Add only once for the first term.
    } else {
      sum += 2 * f_x * dx/2.0;  // Add twice for the middle terms
    }
  }


  return sum;
}


double calculateIntervalSinewave(double length, double x0, double A, double c, double phi) {
  // length:  wanted length to find the interval for.
  // x0:      starting value for the line integral
  // F(x) = A * sin(cx + phi) + d
  // f(x) = F'(x) = c*A * cos(cx + phi)
  // Line length = Integral_a_b [ sqrt( 1 + (f(x)^2) )
  // (f(x))^2 = c^2 * A^2 * (cos(cx + phi))^2
  //          = c*c * A*A * cos(c*x + phi) * cos(c*x + phi)
  double dx = length / 1000;  // Step size is 1/1000 of the length
  double x = x0;
  double sum = 0.0;
  double f_x = 0;

  // Use a while loop to calculate the integral up to the "length" wanted.
  while (sum <= length) {
    f_x = sqrt(1 + c*c * A*A * cos(c*x + phi) * cos(c*x + phi));  // Function value at x
    x += dx;
    if (x == x0) {
      sum += f_x * dx/2.0;  // Add only once for the first term.
    } else {
      sum += 2 * f_x * dx/2.0;  // Add twice for the middle terms
    }
  }


  return x;
}


struct Koordinates {
  double x;
  double y;
};




