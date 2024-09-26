#include <util/atomic.h>
#include <LiquidCrystal_I2C.h>

// Definicije pinova
#define ENCA 2 
#define ENCB 3 
#define IN1 9
#define IN2 10
#define PWM 11
#define POT A1
#define LED_RED 4
#define LED_GREEN 5
#define REGULATOR 6

LiquidCrystal_I2C lcd(0x27, 16, 2);

// Varijable
volatile int pos_i = 0; 
float e_integral = 0, e_prev_filt = 0, e_filt = 0, e_prev = 0;
const int impulses_per_revolution = 400;
const float kp = 9, ki = 0, kd = 0.1287, sampling_freq = 1000.0, cutoff_freq = 500;
float setpoint, deg_setpoint, a1, b0, b1;
int pot_val = 0, pwr, ready = 0;
long prevt = 0, prev_LCDT = 0;

// Prototipovi funkcija
void set_motor(int, int);
void compute_filter_coeficients(float, float, float &, float &, float &);

void setup() {
  Serial.begin(115200);   // Inicijalizacija serijske komunikacije

  // Inicijalizacija pinova za ulaze ili izlaze
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(REGULATOR, INPUT);

  // Inicijalizacija LCD-a
  lcd.init();
  lcd.backlight();
  lcd.clear();

  attachInterrupt(digitalPinToInterrupt(ENCA), read_encoder, RISING); // Inicijalizacija prekidne rutine za ciranje enkodera
  compute_filter_coeficients(cutoff_freq, sampling_freq, a1, b0, b1); // Racunanje koeficijenata filtera

  digitalWrite(LED_RED, HIGH);  // Ukljucivanje ceene LED diode na pocetku - sistem nije spreman 
  digitalWrite(LED_GREEN, LOW); // sve dok se ne ispuni pocetni uslov
}

void loop() {
  long currt = micros();  
  float dt = (currt - prevt) / 1.0e6; 

  if (dt >= 0.001) {  
    prevt = currt;

    // Citanje trenutne pozicije motora sa enkodera
    int pos = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      pos = pos_i;
    }

    pot_val = analogRead(POT);  // Citanje vrijednosti potenciometra
    deg_setpoint = map(pot_val, 0, 1023, 0, 360); // Mapiranje vrijednosti potenciometra 0 - 1023 na 0 - 360 
    setpoint = deg_setpoint / 360 * impulses_per_revolution;  //  zadata pozicija u broju impulsa enkodera
    
    // Uslov za pokretanje motora. Kada je zadata pozicija mala,
    // radi mogucnosti dovodjenja motora u pocetno polozaj 
    if (deg_setpoint < 5){
      ready = 1;
      digitalWrite(LED_RED, LOW);
      digitalWrite(LED_GREEN, HIGH);
    }
    
    // Racunajne i filtriranje greske
    float e = setpoint - pos;
    e_filt = b0 * e + b1 * e_prev - a1 * e_filt;
    e_prev = e;

    // Racunanje integralne i derivacijske greske
    e_integral += e_filt * dt;
    float dedt = (e_filt - e_prev_filt) / dt;
    e_prev_filt = e_filt;

    float u = kp * e_filt + ki * e_integral + kd * dedt;  // Racunanje upravljanja
    u = constrain(u, -70000, 70000);  // Ogranicavanje upravljanja 

    // Ako je upravljanje manje 0 smjer je negativan, u suprotnom pozitivan je
    int dir = (u < 0) ? -1 : 1;

    // Ukljucivanje i iskljucivanje regulatora radi pokazivanja 
    // razlike izmedju regulisanog i neregulisanog sistema
    if (digitalRead(REGULATOR)){
      pwr = map(fabs(u), 0, 70000, 50, 255);
      if (fabs(e) <= 1) pwr = 0;
    } else {
      pwr = 255;
    }
    
    // Ako je sistem spreman za pokretanje motora, motor se pokrece
    if (ready)
      set_motor(dir, pwr);
    
    // Ispisivanje na LCD-u svakih 100ms
    if (millis() - prev_LCDT >= 100) {
      prev_LCDT = millis();
      
      lcd.setCursor(0, 0);
      lcd.print("Pos: ");
      lcd.print(360.0 / impulses_per_revolution * pos);
      lcd.print("    ");

      lcd.setCursor(0, 1);
      lcd.print("Cmd: ");
      lcd.print(deg_setpoint);
      lcd.print("    ");
    }

    Serial.print(360.0 / impulses_per_revolution * pos);
    Serial.print(" ");
    Serial.print(deg_setpoint);
    Serial.println();
  }
}


// Slanje upravljanja na motor
void set_motor(int dir, int pwmVal) {
  analogWrite(PWM, pwmVal);
  digitalWrite(IN1, dir == 1);
  digitalWrite(IN2, dir == -1);
}


// Racunanje koeficijenata niskopropusnog filtera
void compute_filter_coeficients(float cutoff_freq, float sampling_freq, float &a1, float &b0, float &b1) {
  float omega = 2 * PI * cutoff_freq;
  float omega_tilde = 2 * sampling_freq * tan(omega / (2 * sampling_freq));
  
  b0 = b1 = omega_tilde / (omega_tilde + 2 * sampling_freq);
  a1 = (omega_tilde - 2 * sampling_freq) / (omega_tilde + 2 * sampling_freq);
}


// Interapt enkodera
void read_encoder() {
  pos_i += (digitalRead(ENCB) ? 1 : -1);
}
