//#include <avr/iom328p.h> // for vscode to understand PWM timer registry definitions

#define RPM_PIN 2 //D2, PD2
#define RPM_PULSES 2
#define LED_PIN LED_BUILTIN

const word PWM_FREQ_HZ = 25000; // Target freq for Noctua PWM control
const word TCNT1_TOP = 16000000 / (2 * PWM_FREQ_HZ);
const byte OC1A_PIN = 9;
const byte OC1B_PIN = 10; // PWM OUTPUT

int interruptCounter, rpm, currentPWM = 0;

// -----------------------------------------------------------------------------
// Main program setup
// -----------------------------------------------------------------------------
void setup() {
  delay(200); // Precatuion if "serial causes sketch to run twice" should occur.
  Serial.begin(115200);
  while (!Serial) {
    ;  // wait for serial port to connect. Needed for native USB port only
  }
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RPM_PIN, INPUT_PULLUP);
  setup_pwm();
  set_pwm(30);
  Serial.println("Setup complete.");
}

// -----------------------------------------------------------------------------
// Main program loop
// -----------------------------------------------------------------------------
void loop() {
  read_rpm();
  Serial.print("RPM: "); Serial.print(rpm);
  Serial.print(", Duty %: "); Serial.println(currentPWM);
  
  (rpm > 0) ? digitalWrite(LED_PIN, HIGH) : digitalWrite(LED_PIN, LOW);

  if (Serial.available() > 0){
    String inString = "";
    int inChar = 0;
    while (Serial.available() > 0){
      int inChar = Serial.read();
      if (isDigit(inChar)){
        inString += (char)inChar;
      }
    }
    set_pwm(inString.toInt());
  }
}

// -----------------------------------------------------------------------------
// Initializes the timer used for PWM control
// -----------------------------------------------------------------------------
void setup_pwm(){
  pinMode(OC1B_PIN, OUTPUT);

  // Clear Timer1 control and count registers
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  // Timer1 configuration description from 
  // https://projecthub.arduino.cc/KaptenJansson/pwn-fan-controller-with-temp-sensing-and-button-override-5306e0
  // COM1A(1:0) = 0b10   (Output A clear rising/set falling)
  // COM1B(1:0) = 0b00   (Output B normal operation)
  // WGM(13:10) = 0b1010 (Phase correct PWM)
  // ICNC1      = 0b0    (Input capture noise canceler disabled)
  // ICES1      = 0b0    (Input capture edge select disabled)
  // CS(12:10)  = 0b001  (Input clock select = clock/1)

  // https://github.com/mariuste/Fan_Temp_Control/blob/main/code/Fan_Temp_Control/Fan_Temp_Control.ino
  // Set PWM frequency to about 25khz on pins 9,10 (timer 1 mode 10, no prescale, count to 320)
  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
  TCCR1B = (1 << CS10) | (1 << WGM13);
  ICR1 = TCNT1_TOP;
  OCR1A = 0;
  OCR1B = 0;
}

// -----------------------------------------------------------------------------
// Sets the specific duty cycle (%) on the PWM pin
// -----------------------------------------------------------------------------
void set_pwm(int duty){
  duty = duty<0 ? 0 : duty>100 ? 100 : duty; // c++ ternary conditional operator
	float f = (float) duty / 100;
  OCR1B = (uint16_t) (TCNT1_TOP * f);
  currentPWM = duty;
}

// -----------------------------------------------------------------------------
// Reads and calculates the current fan RPM. 
// -----------------------------------------------------------------------------
int read_rpm() {
  interruptCounter = 0;
  sei(); // enables interrupts
  attachInterrupt(digitalPinToInterrupt(RPM_PIN), rpm_countup, RISING);
  delay(1000);
  detachInterrupt(digitalPinToInterrupt(RPM_PIN));
  cli(); // disables interrupts

  // Since fan tachometer outputs signal in hertz and not rpm, reading is
  // multiplied by 60. Additionally, as the fan outputs two impulses per
  // revolution, the reading is also divided by 2.
  // https://noctua.at/pub/media/wysiwyg/Noctua_PWM_specifications_white_paper.pdf
  // https://noctua.at/en/nf-a12x25-5v-pwm/specification
  rpm = (interruptCounter * 60) / 2;
}

// -----------------------------------------------------------------------------
// Called on RPM interrupts
// -----------------------------------------------------------------------------
void rpm_countup() {
  interruptCounter++;
}