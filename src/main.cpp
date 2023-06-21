#include <Arduino.h>
#include <EMGFilters.h>

#define TIMING_DEBUG 0

#define SensorInputPin 4

EMGFilters myFilter;
SAMPLE_FREQUENCY sampleRate = SAMPLE_FREQ_1000HZ;
NOTCH_FREQUENCY humFreq = NOTCH_FREQ_50HZ;

static int Threshold = 0;

unsigned long timeStamp;
unsigned long timeBudget;

void calibrate(int delayTime)
{
  Serial.println("configurando\r\n");
  delay(500);

  unsigned long endTime = millis() + 5000;

  while (millis() < endTime)
  {
    int Value = analogRead(SensorInputPin);
    int DataAfterFilter = myFilter.update(Value);
    int envelope = sq(DataAfterFilter);
    delayMicroseconds(500);
  }

  endTime = millis() + delayTime;

  int maxVal = 0;

  delay(500);
  Serial.println("iniciado\r\n");
  delay(500);

  while (millis() < endTime)
  {
    int Value = analogRead(SensorInputPin);
    int DataAfterFilter = myFilter.update(Value);
    int envelope = sq(DataAfterFilter);
    maxVal = max(maxVal, envelope);
    delayMicroseconds(500);

    if (TIMING_DEBUG)
    {
      Serial.print("Read Data: ");
      Serial.print(Value);
      Serial.print(". Filtered Data: ");
      Serial.print(DataAfterFilter);
      Serial.print(". Squared Data: ");
      Serial.print(envelope);
      Serial.print(". Threshold: ");
      Serial.println(Threshold);
    }
  }

  Threshold = maxVal;
  delay(500);
  Serial.println("finalizado\r\n");
  delay(500);
}

void measure(int delayTime)
{
  unsigned long endTime = millis() + delayTime;

  while (millis() < endTime)
  {
    timeStamp = micros();

    int Value = analogRead(SensorInputPin);
    int DataAfterFilter = myFilter.update(Value);
    int envelope = sq(DataAfterFilter);
    envelope = (envelope > Threshold) ? envelope : 0;

    Serial.println(String(envelope));

    timeStamp = micros() - timeStamp;
    if (TIMING_DEBUG)
    {
      if (envelope > Threshold)
      {
        Serial.print("Read Data: ");
        Serial.println(Value);
        Serial.print("Filtered Data: ");
        Serial.println(DataAfterFilter);
        Serial.print("Envelope: ");
        Serial.println(sq(DataAfterFilter));
        Serial.print("Squared Data: ");
        Serial.println(envelope);
        Serial.print("Filters cost time: ");
        Serial.println(timeStamp);
        Serial.print("Threshold: ");
        Serial.println(Threshold);
      }
    }
    delayMicroseconds(500);
  }
}

void setup()
{
  Serial.begin(115200);
  myFilter.init(sampleRate, humFreq, true, true, true);

  timeBudget = 1e6 / sampleRate;

  if (TIMING_DEBUG)
  {
    delay(5000);
    calibrate(10000);
    delay(5000);
  }
}

void loop()
{
  if (Serial.available() > 0)
  {
    delay(10);
    String inputLine = Serial.readStringUntil('\r\n');

    int commaIndex = inputLine.indexOf(',');

    if (commaIndex > 0)
    {
      String command = inputLine.substring(0, commaIndex);
      command.trim();

      if (command == "calibrar")
      {
        String durationStr = inputLine.substring(commaIndex + 1);
        durationStr.trim();

        int duration = durationStr.toInt();
        duration *= 1000;

        calibrate(duration);
        return;
      }

      if (command == "medir")
      {
        String durationStr = inputLine.substring(commaIndex + 1);
        durationStr.trim();

        int duration = durationStr.toInt();
        duration *= 1000;

        measure(duration);
        return;
      }
    }
  }
}
