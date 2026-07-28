#include "drive_4wd.h"

#include "main.h"

#define DRIVE_PWM_MAX          (999U)
#define APP_PWM_MAX            (255U)
#define MIN_PWM_THRESHOLD      (15U)
#define DRIVE_TIMEOUT_MS       (500U)

static TIM_HandleTypeDef *drive_timer;
static uint32_t last_command_tick;
static uint8_t command_received;
static volatile uint8_t estop_latched;
static volatile uint8_t storage_inhibit;

static void SetMotorDirection(GPIO_TypeDef *in1_port,
                              uint16_t in1_pin,
                              GPIO_TypeDef *in2_port,
                              uint16_t in2_pin,
                              uint8_t direction,
                              uint8_t pwm)
{
  GPIO_PinState in1 = GPIO_PIN_RESET;
  GPIO_PinState in2 = GPIO_PIN_RESET;

  if (pwm != 0U)
  {
    if (direction == 0U)
    {
      in1 = GPIO_PIN_SET;
    }
    else
    {
      in2 = GPIO_PIN_SET;
    }
  }

  HAL_GPIO_WritePin(in1_port, in1_pin, in1);
  HAL_GPIO_WritePin(in2_port, in2_pin, in2);
}

static uint32_t ScalePwm(uint8_t pwm)
{
  if (pwm < MIN_PWM_THRESHOLD)
  {
    return 0U;
  }

  return ((uint32_t)pwm * DRIVE_PWM_MAX) / APP_PWM_MAX;
}

HAL_StatusTypeDef Drive4WD_Init(TIM_HandleTypeDef *timer)
{
  if (timer == NULL)
  {
    return HAL_ERROR;
  }

  drive_timer = timer;
  if ((HAL_TIM_PWM_Start(drive_timer, TIM_CHANNEL_1) != HAL_OK) ||
      (HAL_TIM_PWM_Start(drive_timer, TIM_CHANNEL_2) != HAL_OK))
  {
    return HAL_ERROR;
  }

  Drive4WD_Stop();
  estop_latched = 0U;
  storage_inhibit = 0U;
  return HAL_OK;
}

void Drive4WD_Apply(const BluetoothDriveCommand *command)
{
  uint32_t left_pwm;
  uint32_t right_pwm;

  if ((drive_timer == NULL) || (command == NULL))
  {
    return;
  }

  if (command->control == BLUETOOTH_DRIVE_ESTOP)
  {
    estop_latched = 1U;
    Drive4WD_Stop();
    command_received = 0U;
    return;
  }

  if (command->control == BLUETOOTH_DRIVE_ESTOP_CLEAR)
  {
    Drive4WD_Stop();
    estop_latched = 0U;
    command_received = 0U;
    return;
  }

  if ((estop_latched != 0U) || (storage_inhibit != 0U))
  {
    Drive4WD_Stop();
    return;
  }

  left_pwm = ScalePwm(command->left_pwm);
  right_pwm = ScalePwm(command->right_pwm);

  /* 방향 핀을 바꾸기 전에 PWM을 내려 순간적인 역방향 전류를 줄인다. */
  __HAL_TIM_SET_COMPARE(drive_timer, TIM_CHANNEL_1, 0U);
  __HAL_TIM_SET_COMPARE(drive_timer, TIM_CHANNEL_2, 0U);
  SetMotorDirection(MOTOR_L_IN1_GPIO_Port,
                    MOTOR_L_IN1_Pin,
                    MOTOR_L_IN2_GPIO_Port,
                    MOTOR_L_IN2_Pin,
                    command->left_direction,
                    (uint8_t)(left_pwm != 0U));
  SetMotorDirection(MOTOR_R_IN1_GPIO_Port,
                    MOTOR_R_IN1_Pin,
                    MOTOR_R_IN2_GPIO_Port,
                    MOTOR_R_IN2_Pin,
                    command->right_direction,
                    (uint8_t)(right_pwm != 0U));

  __HAL_TIM_SET_COMPARE(drive_timer, TIM_CHANNEL_1, left_pwm);
  __HAL_TIM_SET_COMPARE(drive_timer, TIM_CHANNEL_2, right_pwm);
  last_command_tick = command->received_tick;
  command_received = 1U;
}

void Drive4WD_Stop(void)
{
  HAL_GPIO_WritePin(MOTOR_L_IN1_GPIO_Port,
                    MOTOR_L_IN1_Pin,
                    GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_L_IN2_GPIO_Port,
                    MOTOR_L_IN2_Pin,
                    GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_R_IN1_GPIO_Port,
                    MOTOR_R_IN1_Pin,
                    GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_R_IN2_GPIO_Port,
                    MOTOR_R_IN2_Pin,
                    GPIO_PIN_RESET);

  if (drive_timer != NULL)
  {
    __HAL_TIM_SET_COMPARE(drive_timer, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(drive_timer, TIM_CHANNEL_2, 0U);
  }
}

void Drive4WD_CheckTimeout(void)
{
  if ((command_received != 0U) &&
      ((HAL_GetTick() - last_command_tick) > DRIVE_TIMEOUT_MS))
  {
    Drive4WD_Stop();
    command_received = 0U;
  }
}

void Drive4WD_SetStorageInhibit(uint8_t inhibit)
{
  storage_inhibit = (uint8_t)(inhibit != 0U);
  if (storage_inhibit != 0U)
  {
    Drive4WD_Stop();
    command_received = 0U;
  }
}
