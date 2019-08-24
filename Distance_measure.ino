/*
Reference:
  https://maxembedded.wordpress.com/2011/06/22/introduction-to-avr-timers/

  Configuring timer

  Timer Count = (Required Delay / ClockTimePeriod)
  
  CPU clock = 16 MHz
  Timer 1 = 16 bit -> 2*e16 = 65535 bytes;
  65535/16MHz = 0.00409 -> 4.09ms 
  When register is filled 65535 bits, counter starts from begin this is called overflow
  
  Time Period = 1/ Frequency, Frequency = 4MHz
  Flash LED every 2 seconds at a frequency of 0.5Hz. We have an XTAL of 16MHz
  Timer count = (Required delay/ clock Time Period) - 1
  
  HC-SR04
  Ranging distance: 2cm - 400cm
  Distance = (traveltime/2) * speed of sound;
  Speed of sound is 343m/s = 0.0343cm/uS
*/

const int trigPin = 12;
const int echoPin = 11;
const int sampleTableSize = 1000;  // Define max amount of samples
int calibrateSamples = 500;
int infinityValue = 0; // This tells us when nothing is detected.
int printedResults = 0;
int timer = 0;  
boolean timerEnabled = false;

boolean outdoor = false;  // If no reflection received while calibrating, we assume we are outside or there is nothing to reflect sound
const int maxOperatingRange = 420;

const int sampleTime = 2; // For 5 seconds take samples 
const int detectionRate = 30; // Area counted where the sample value can differ, if difference of infinity value and detectionRate is over x. We assume something has been detected.

boolean calibrateSuccess = false;

int sampleTable[sampleTableSize]; // Table for samples
int sampleTableCount = 0;  // Tells us size of sampleTable

// Initialize hardware
void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  //Serial.begin(9600);  // Define baudrate of usb
  Serial2.begin(9600);
  
  // Set timer Timer 1
  noInterrupts();  // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  
  OCR1A = 31250;  // Compare match register 16MHz/256/2Hz https://www.microchip.com/wwwproducts/en/ATmega2560
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS12);    // 256 prescaler
  TIMSK1 |= (1 << OCIE1A);  // Enable timer compare interrupt
  interrupts();             // Enable all interrupts
  delay(5000);              // Wait for 5 seconds

}

// Interrupt
ISR(TIMER1_COMPA_vect) {
   if(timerEnabled) {
     timer++;
   }
}

// Tells sensor to send ultravawe and read the distance.
int readDistance() {
 
 long duration = 0;
 int distance = 0; 
 digitalWrite(trigPin, LOW);
 delayMicroseconds(2);
 digitalWrite(trigPin, HIGH);
 delayMicroseconds(10);
 digitalWrite(trigPin, LOW);
 
 duration = pulseIn(echoPin, HIGH);
 distance = duration * 0.034/2;

 return distance;
 
}

// Take hundred samples. If distance stays equal define value as infinity
void calibrate() {

  Serial2.println("Calibrating");
  int lowest = 0;
  int highest = 0;
  int sample = 0;
  
  for(int i = 0; i < calibrateSamples; i++)  {
    sample = readDistance();
    
    if(lowest == 0 || lowest > sample )   {
       lowest = sample; 
    } else if(highest == 0 || highest > sample) {
      highest = sample;
    }
    
    int difference = (highest - lowest);
    // If highest and low difference is less than 5 and highest detected point is less than 450cm
    if(difference > 5 && highest < maxOperatingRange) {
    //  Serial.println("Failed to calibrate");
      Serial2.println("Failed to calibrate");
      calibrateSuccess = false;
      
      return;
    }
    // If highest point is over the operating range, consider system being outdoor
    if(highest > maxOperatingRange) {
      outdoor = true; 
      break;
    }
    
  }
  infinityValue = sample;
  calibrateSuccess = true;

  Serial2.print("Calibration success ");
  Serial2.println(infinityValue);
  // Get lowest and highest point;
}

// Detect if anything happens
// If conditions are met, start taking samples
boolean detectAction() {
  // Detect if something is happening
  int sample = readDistance();
  int getDifference = ((int)sample - (int)infinityValue);
  
  if(sample < maxOperatingRange && outdoor) {
    sampleTable[0] = sample;
    return true;
  }else if(!outdoor && (getDifference > detectionRate/2 || getDifference < -detectionRate/2))  {
    sampleTable[0] = sample;
    return true;
  }
  return false;
}

// Start timer 1
void timerStart() {
  timer = 0;
  timerEnabled = true;
}

// Stop timer 1
void timerStop() {
  timerEnabled = false;
}
// Take samples until 10 sample points reaches infinity
// Infinity means nothing is obstructing or reflecting ultrasound back to sensor
int filterDistance() {
  boolean finished = false;
  timerStart();
  int samplePoll = 1;
  while(timer < (sampleTime * 2)) {

      // Take samples until timer has triggered or infinity is detected
    sampleTable[samplePoll] = readDistance();
    samplePoll++;
    if(samplePoll > 5) {
      int count = 0; 
      for(int i = 0; i <= 6; i++) {
        if( sampleTable[samplePoll -i] > maxOperatingRange && outdoor) {
          count++;
        } else if( !outdoor && sampleTable[samplePoll -i] - infinityValue < detectionRate && sampleTable[samplePoll -i] - infinityValue > -detectionRate) {
          count++;
        }
        
      }
      if(count >= 5 || (samplePoll + 5) > sampleTableSize) {
        //Serial.println("Filter distance() Sampling ended");
        sampleTableCount = (samplePoll - 5);
        break;
      } else {
        sampleTableCount = (samplePoll - 1);
      }
    }
  }
  timerStop();
}

// Find lowest point from taken samples
int getLowPoint() {
  int lowPoint = 0;
  for(int i = 0; i < sampleTableCount; i++) {
    if( (sampleTable[i] < lowPoint || lowPoint == 0) && (lowPoint < maxOperatingRange) ) {
      if( sampleTable[i] <= 0) {
         continue;
      }
      
      lowPoint = sampleTable[i];
    }
  }
  return lowPoint;
}

// Find highest point from samples
int getHighPoint() {
  int highPoint = 0;
  for(int i = 0; i < sampleTableCount; i++) {
    if(sampleTable[i] > highPoint || highPoint == 0) {
      if( (sampleTable[i] < maxOperatingRange) && outdoor) {
          continue;
      } else if(!outdoor && (sampleTable[i] - infinityValue < detectionRate && sampleTable[i] - infinityValue > -detectionRate) ) {
         continue;
      }
      highPoint = sampleTable[i];
    }
  }
  return highPoint;
}

// Calculate avarage point from samples
int getAvg() {
  long sum = 0;
  int count = 0;;
  for(int i = 0; i < sampleTableCount; i++) {
    if( (sampleTable[i] < maxOperatingRange) && outdoor) {
      continue;
    } else if(!outdoor && sampleTable[i] - infinityValue < detectionRate && sampleTable[i] - infinityValue > -detectionRate) {
      continue;
    } else if (sampleTable[i] <= 0) {
      continue;
    }
    sum = sum + sampleTable[i];
    count++;
  }
  sum = sum/count;
  return sum;
}

void analyzeSamples() {
  printedResults++;
  Serial2.println("");
  Serial2.print("Results : ");
  Serial2.println(printedResults);
  Serial2.print("Find lowest point: ");
  Serial2.println(getLowPoint());
  Serial2.print("Find highest point: ");
  Serial2.println(getHighPoint());
  Serial2.print("Find avarage point: ");
  Serial2.println(getAvg());
  Serial2.println("");
}
  
void loop() {
  int height = 0;
  while(!calibrateSuccess) {
    calibrate();
  }
  // If action detected start taking samples for x amount of time;
  if(detectAction()) {
    filterDistance();  // For x amount of time take samples
    analyzeSamples();
  }
}


