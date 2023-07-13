#include <softPwm.h>
#include <wiringPi.h>

#include <iostream>

const uint8_t in1_1 = 24;
const uint8_t in1_2 = 23;
const uint8_t en1 = 25;

const uint8_t in2_1 = 17;
const uint8_t in2_2 = 18;
const uint8_t en2 = 22;

int temp1 = 1;
int pwmValue = 50;

int main() {
  wiringPiSetupGpio();

  pinMode(in1_1, OUTPUT);
  pinMode(in1_2, OUTPUT);
  pinMode(en1, OUTPUT);

  digitalWrite(in1_1, LOW);
  digitalWrite(in1_2, LOW);

  softPwmCreate(en1, 0, 100);   // Create Software-controlled PWM
  softPwmWrite(en1, pwmValue);  // Write the value to the PWM output

  std::cout << "\n";
  std::cout << "The default speed & direction of motor is LOW & Forward.....\n";
  std::cout
      << "r-run s-stop f-forward b-backward l-low m-medium h-high e-exit\n";
  std::cout << "\n";

  char x;
  while (std::cin >> x) {
    switch (x) {
      case 'r':
        std::cout << "run\n";
        if (temp1 == 1) {
          digitalWrite(in1_1, HIGH);
          digitalWrite(in1_2, LOW);
          std::cout << "forward\n";
        } else {
          digitalWrite(in1_1, LOW);
          digitalWrite(in1_2, HIGH);
          std::cout << "backward\n";
        }
        break;

      case 's':
        std::cout << "stop\n";
        digitalWrite(in1_1, LOW);
        digitalWrite(in1_2, LOW);
        break;

      case 'f':
        std::cout << "forward\n";
        digitalWrite(in1_1, HIGH);
        digitalWrite(in1_2, LOW);
        temp1 = 1;
        break;

      case 'b':
        std::cout << "backward\n";
        digitalWrite(in1_1, LOW);
        digitalWrite(in1_2, HIGH);
        temp1 = 0;
        break;

      case 'l':
        std::cout << "low\n";
        pwmValue = 25;
        softPwmWrite(en1, pwmValue);
        break;

      case 'm':
        std::cout << "medium\n";
        pwmValue = 50;
        softPwmWrite(en1, pwmValue);
        break;

      case 'h':
        std::cout << "high\n";
        pwmValue = 75;
        softPwmWrite(en1, pwmValue);
        break;

      case 'e':
        std::cout << "Exiting...\n";
        digitalWrite(in1_1, LOW);
        digitalWrite(in1_2, LOW);
        return 0;

      default:
        std::cout << "<<<  wrong data  >>>\n";
        std::cout << "please enter the defined data to continue.....\n";
        break;
    }
  }

  return 0;
}
