/**
  BB Keyboard Badge Add-on Firmware

  Copyright (C) 2021 Uri Shaked
*/

#include <Arduino.h>
#include <Wire.h>
#include <RingBuf.h>

#define I2C_ADDR 0x42

#define CMD_LEDA12 bit(0)
#define CMD_LEDA34 bit(1)
#define CMD_LEDK14 bit(2)
#define CMD_LEDK23 bit(3)

#define CMD_INT_ENABLE bit(6)
#define CMD_RESET bit(7) /* Resets the keyboard buffer */

#define RESP_FLAG_KEYDOWN bit(6)
#define RESP_FLAG_KEYUP bit(7)
#define RESP_RESET 0xfe     /* Keyboard buffer was reset */
#define RESP_BUF_EMPTY 0xff /* Input buffer empty */

#define KBD_ROWS 7
#define KBD_COLS 5

#define KBD_BUFFER_SIZE 256

const uint8_t rowPins[KBD_ROWS] = {PA6, PA5, PA4, PA3, PA2, PA1, PA0};
const uint8_t colPins[KBD_COLS] = {PB15, PB14, PB13, PB12, PB11};
const uint8_t anodes[2] = {PA10, PA12};
const uint8_t cathodes[2] = {PA9, PA11};
const uint8_t ledPin = PB7;
const uint8_t intPin = PB6;

bool initialized = false;
bool interruptEnabled = false;
RingBuf<byte, KBD_BUFFER_SIZE> kbdBuffer;

/* Slow down clock to 8MHz. This saves a few milliamps! */
extern "C" void SystemClock_Config()
{
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

void i2cStarted()
{
  if (!initialized)
  {
    digitalWrite(ledPin, HIGH);
    pinMode(ledPin, INPUT);
  }
  initialized = true;
}

void updateInterrupt()
{
  if (interruptEnabled)
  {
    pinMode(intPin, OUTPUT);
    digitalWrite(intPin, kbdBuffer.isEmpty() ? HIGH : LOW);
  }
  else
  {
    pinMode(intPin, INPUT);
  }
}

void i2cReceive(int count)
{
  i2cStarted();
  while (Wire.available())
  {
    uint8_t cmd = Wire.read();
    if (cmd & CMD_RESET)
    {
      kbdBuffer.clear();
      kbdBuffer.push(RESP_RESET);
    }
    interruptEnabled = cmd & CMD_INT_ENABLE;
    digitalWrite(cathodes[0], cmd & CMD_LEDK23);
    digitalWrite(cathodes[1], cmd & CMD_LEDK14);
    digitalWrite(anodes[0], cmd & CMD_LEDA34);
    digitalWrite(anodes[1], cmd & CMD_LEDA12);
    updateInterrupt();
  }
}

void i2cRequest()
{
  i2cStarted();
  uint8_t item = RESP_BUF_EMPTY; // fallback value
  kbdBuffer.pop(item);
  updateInterrupt();
  Wire.write(item);
}

void setup()
{
  pinMode(ledPin, OUTPUT);
  pinMode(anodes[0], OUTPUT);
  pinMode(anodes[1], OUTPUT);
  pinMode(cathodes[0], OUTPUT);
  pinMode(cathodes[1], OUTPUT);
  for (int i = 0; i < KBD_ROWS; i++)
  {
    pinMode(rowPins[i], INPUT_PULLUP);
  }
  for (int i = 0; i < KBD_COLS; i++)
  {
    pinMode(colPins[i], INPUT);
  }

  // I2C Setup
  Wire.setSCL(PB8);
  Wire.setSDA(PB9);
  Wire.begin(I2C_ADDR);
  Wire.onReceive(i2cReceive);
  Wire.onRequest(i2cRequest);
}

void blinkKeyboard()
{
  digitalWrite(anodes[0], millis() % 1000 >= 500);
  digitalWrite(anodes[1], !digitalRead(anodes[0]));

  // Breathe LED
  int time = millis() % 5000;
  if (time >= 4096)
  {
    digitalWrite(ledPin, HIGH); // LED Off
  }
  else
  {
    uint8_t ledLevel = (time % 2048) / 8;
    analogWrite(ledPin, (time >= 2048) ? ledLevel : 255 - ledLevel);
  }
}

uint8_t keys[KBD_COLS] = {0};

void scanKeyboard()
{
  for (int col = 0; col < KBD_COLS; col++)
  {
    uint8_t colValue = 0;
    pinMode(colPins[col], OUTPUT);
    for (int row = 0; row < KBD_ROWS; row++)
    {
      colValue |= digitalRead(rowPins[row]) ? 0 : bit(row);
    }
    pinMode(colPins[col], INPUT);
    uint8_t changedRows = keys[col] ^ colValue;
    keys[col] = colValue;
    if (changedRows)
    {
      for (int row = 0; row < KBD_ROWS; row++)
      {
        if (changedRows & bit(row))
        {
          bool keyDown = colValue & bit(row);
          uint8_t flag = keyDown ? RESP_FLAG_KEYDOWN : RESP_FLAG_KEYUP;
          kbdBuffer.push((col + 1) | (row + 1) << 3 | flag);
          updateInterrupt();
        }
      }
    }
  }
}

void loop()
{
  if (initialized)
  {
    scanKeyboard();
  }
  else
  {
    blinkKeyboard();
  }
}