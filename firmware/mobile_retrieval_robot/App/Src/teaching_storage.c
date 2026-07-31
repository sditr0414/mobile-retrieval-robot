#include "teaching_storage.h"

#include <stddef.h>
#include <string.h>

/*
 * 12개 티칭 시퀀스와 PID·서보 보정값을 Flash Sector 7에 저장한다.
 * 수신 조각은 RAM에 먼저 모으고 CRC와 전체 검증을 통과한 COMMIT에서만 기록한다.
 */

#define STORAGE_MAGIC          (0x4D525254UL)
#define STORAGE_VERSION        (8U)
#define STORAGE_VERSION_V7     (7U)
#define STORAGE_VERSION_V6     (6U)
#define STORAGE_VERSION_V5     (5U)
#define STORAGE_VERSION_V4     (4U)
#define STORAGE_VERSION_V3     (3U)
#define STORAGE_VERSION_V2     (2U)
#define STORAGE_FLASH_ADDRESS  (0x08060000UL)
#define STORAGE_FLASH_SECTOR   FLASH_SECTOR_7
#define PID_KP_MAX_MILLI       (2550L)
#define PID_GAIN_MAX_MILLI     (100000L)
#define PID_KP_STEP_MILLI      (10L)
#define TEACHING_NAME_PART_SIZE  (8U)
#define TEACHING_NAME_PART_COUNT (3U)

typedef struct
{
  TeachingWaypoint waypoints[TEACHING_MAX_WAYPOINTS];
  uint8_t count;
} TeachingSequenceV7;

typedef struct
{
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  TeachingSequenceV7 sequences[TEACHING_SEQUENCE_COUNT];
  uint32_t crc;
} TeachingImageV2;

typedef struct
{
  int32_t pid_kp_milli;
  int32_t pid_ki_milli;
  int32_t pid_kd_milli;
  int16_t servo_zero_trim_us[ROBOT_SETTINGS_SERVO_COUNT];
} RobotSettingsV3;

typedef struct
{
  int32_t pid_kp_milli;
  int32_t pid_ki_milli;
  int32_t pid_kd_milli;
  uint16_t servo_pulse_us[ROBOT_SETTINGS_SERVO_COUNT]
                         [ROBOT_SETTINGS_PULSE_COUNT];
} RobotSettingsV6;

typedef struct
{
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  TeachingSequenceV7 sequences[TEACHING_SEQUENCE_COUNT];
  RobotSettingsV3 settings;
  uint32_t crc;
} TeachingImageV3;

typedef struct
{
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  TeachingSequenceV7 sequences[TEACHING_SEQUENCE_COUNT];
  RobotSettingsV6 settings;
  uint32_t crc;
} TeachingImageV4;

typedef struct
{
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  TeachingSequenceV7 sequences[TEACHING_SEQUENCE_COUNT];
  RobotSettingsV6 settings;
  uint32_t crc;
} TeachingImageV5;

typedef struct
{
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  TeachingSequenceV7 sequences[TEACHING_SEQUENCE_COUNT];
  RobotSettingsV6 settings;
  uint32_t crc;
} TeachingImageV6;

typedef struct
{
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  TeachingSequenceV7 sequences[TEACHING_SEQUENCE_COUNT];
  RobotSettings settings;
  uint32_t crc;
} TeachingImageV7;

typedef struct
{
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  TeachingSequence sequences[TEACHING_SEQUENCE_COUNT];
  RobotSettings settings;
  uint32_t crc;
} TeachingImage;

_Static_assert(sizeof(TeachingImage) <= (128U * 1024U),
               "Teaching image must fit in STM32 Flash sector 7");
_Static_assert(sizeof(TeachingSequenceV7) == 181U,
               "Legacy teaching sequence layout must remain exactly 181 bytes");
_Static_assert(sizeof(TeachingSequence) == 204U,
               "Current teaching sequence layout must remain exactly 204 bytes");
_Static_assert(sizeof(RobotSettingsV3) == 24U,
               "Legacy robot settings must remain exactly 24 bytes");
_Static_assert(sizeof(RobotSettingsV6) == 48U,
               "Version 6 robot settings must remain exactly 48 bytes");
_Static_assert(sizeof(RobotSettings) == 56U,
               "Current robot settings layout must remain exactly 56 bytes");

static TeachingSequence sequences[TEACHING_SEQUENCE_COUNT];
static RobotSettings robot_settings;
static TeachingSequence upload_sequence;
static TeachingImage storage_image;
static uint8_t upload_parts[TEACHING_MAX_WAYPOINTS];
static uint8_t upload_sequence_id;
static uint8_t upload_active;
static uint8_t upload_name_parts;

#define DEFAULT_PID_KP_MILLI (2000L)
#define DEFAULT_PID_KI_MILLI (1400L)
#define DEFAULT_PID_KD_MILLI (0L)

/* 손상되거나 비현실적으로 큰 PID 값이 실행 제어기로 전달되지 않게 한다. */
static uint8_t IsPidSettingsValid(const RobotSettings *settings)
{
  return (uint8_t)((settings != NULL) &&
                   (settings->pid_kp_milli >= 0) &&
                   (settings->pid_kp_milli <= PID_KP_MAX_MILLI) &&
                   ((settings->pid_kp_milli % PID_KP_STEP_MILLI) == 0L) &&
                   (settings->pid_ki_milli >= 0) &&
                   (settings->pid_ki_milli <= PID_GAIN_MAX_MILLI) &&
                   (settings->pid_kd_milli >= 0) &&
                   (settings->pid_kd_milli <= PID_GAIN_MAX_MILLI));
}

/*
 * 각 행은 앱 표시 기준 -90도, 0도, +90도 펄스폭이다.
 * Gripper는 열림, 자동 중앙값, 닫힘 순서로 사용한다.
 */
static const uint16_t default_servo_pulse_us
    [ROBOT_SETTINGS_SERVO_COUNT][ROBOT_SETTINGS_PULSE_COUNT] = {
        {2300U, 1500U, 700U},
        {2300U, 1500U, 700U},
        {700U, 1500U, 2300U},
        {2300U, 1500U, 700U},
        {2300U, 1500U, 700U},
        {1800U, 1500U, 1200U}};

/* Gripper를 제외한 기존 이동 자세의 패킷값이다. */
static const uint8_t default_travel_pose_angles
    [ROBOT_SETTINGS_TRAVEL_JOINT_COUNT] = {
        90U, 40U, 180U, 90U, 150U};

static const char *const default_sequence_names[TEACHING_SEQUENCE_COUNT] = {
    "Sequence 1", "Sequence 2", "Sequence 3", "Sequence 4",
    "Sequence 5", "Sequence 6", "Sequence 7", "Sequence 8",
    "Travel", "Pickup Pose", "Pick", "Place"};

/* 새 Flash와 이전 버전에 이름이 없을 때 사용할 짧은 기본 이름을 넣는다. */
static void LoadDefaultSequenceName(uint8_t index,
                                    TeachingSequence *sequence)
{
  size_t length;

  if ((index >= TEACHING_SEQUENCE_COUNT) || (sequence == NULL))
  {
    return;
  }

  length = strlen(default_sequence_names[index]);
  sequence->name_length = (uint8_t)length;
  memcpy(sequence->name, default_sequence_names[index], length + 1U);
}

static void LoadDefaultSequenceNames(void)
{
  uint8_t index;

  for (index = 0U; index < TEACHING_SEQUENCE_COUNT; index++)
  {
    LoadDefaultSequenceName(index, &sequences[index]);
  }
}

/* 9~12번은 차량 안전 흐름이 번호에 의존하므로 이름도 고정한다. */
static uint8_t IsReservedSequenceNameValid(uint8_t sequence_id,
                                           const TeachingSequence *sequence)
{
  size_t length;
  const char *expected;

  if (sequence_id < 9U)
  {
    return 1U;
  }
  expected = default_sequence_names[sequence_id - 1U];
  length = strlen(expected);
  return (uint8_t)((sequence != NULL) &&
                   (sequence->name_length == length) &&
                   (memcmp(sequence->name, expected, length) == 0));
}

/* 이름이 없는 v2~v7 웨이포인트를 보존하고 기본 이름만 추가한다. */
static void MigrateLegacySequences(
    const TeachingSequenceV7 legacy[TEACHING_SEQUENCE_COUNT])
{
  uint8_t index;

  memset(sequences, 0, sizeof(sequences));
  for (index = 0U; index < TEACHING_SEQUENCE_COUNT; index++)
  {
    memcpy(sequences[index].waypoints,
           legacy[index].waypoints,
           sizeof(sequences[index].waypoints));
    sequences[index].count = legacy[index].count;
    LoadDefaultSequenceName(index, &sequences[index]);
  }
}

/* 실측 또는 임시 기본 서보 3점 보정값을 설정 구조체에 채운다. */
static void LoadDefaultServoCalibrations(RobotSettings *settings)
{
  memcpy(settings->servo_pulse_us,
         default_servo_pulse_us,
         sizeof(settings->servo_pulse_us));
}

/* 손상된 PID만 복구할 때 서보 보정과 이동 자세를 보존한다. */
static void LoadDefaultPidSettings(RobotSettings *settings)
{
  settings->pid_kp_milli = DEFAULT_PID_KP_MILLI;
  settings->pid_ki_milli = DEFAULT_PID_KI_MILLI;
  settings->pid_kd_milli = DEFAULT_PID_KD_MILLI;
}

/* 손상된 이동 자세만 복구할 때 PID와 서보 보정을 보존한다. */
static void LoadDefaultTravelPose(RobotSettings *settings)
{
  memcpy(settings->travel_pose_angles,
         default_travel_pose_angles,
         sizeof(settings->travel_pose_angles));
}

/* Flash가 비었을 때 사용할 전체 안전 초기 설정을 만든다. */
static void LoadDefaultSettings(RobotSettings *settings)
{
  memset(settings, 0, sizeof(*settings));
  LoadDefaultPidSettings(settings);
  LoadDefaultServoCalibrations(settings);
  LoadDefaultTravelPose(settings);
}

/* 이동 자세는 앱 표시 -90~+90도에 대응하는 패킷값 0~180만 허용한다. */
static uint8_t IsTravelPoseValid(const RobotSettings *settings)
{
  uint8_t joint;

  if (settings == NULL)
  {
    return 0U;
  }
  for (joint = 0U; joint < ROBOT_SETTINGS_TRAVEL_JOINT_COUNT; joint++)
  {
    if (settings->travel_pose_angles[joint] > 180U)
    {
      return 0U;
    }
  }
  return 1U;
}

/* v4~v6의 PID·서보 설정을 보존하고 이동 자세만 안전한 기본값으로 채운다. */
static void MigrateV6Settings(const RobotSettingsV6 *legacy,
                              RobotSettings *settings)
{
  LoadDefaultSettings(settings);
  settings->pid_kp_milli = legacy->pid_kp_milli;
  settings->pid_ki_milli = legacy->pid_ki_milli;
  settings->pid_kd_milli = legacy->pid_kd_milli;
  memcpy(settings->servo_pulse_us,
         legacy->servo_pulse_us,
         sizeof(settings->servo_pulse_us));
}

/* 이전 버전의 이동량을 펄스 범위 안으로 제한한다. */
static uint16_t ClampPulse(int32_t pulse, uint16_t minimum, uint16_t maximum)
{
  if (pulse < minimum)
  {
    return minimum;
  }
  if (pulse > maximum)
  {
    return maximum;
  }
  return (uint16_t)pulse;
}

/* 버전 3 설정을 현재 3점 서보 보정 형식으로 변환한다. */
static void MigrateV3Settings(const RobotSettingsV3 *legacy,
                              RobotSettings *settings)
{
  uint8_t joint;
  uint8_t point;
  uint16_t lower;
  uint16_t upper;

  LoadDefaultSettings(settings);
  if ((legacy->pid_kp_milli >= 0) &&
      (legacy->pid_kp_milli <= PID_KP_MAX_MILLI) &&
      ((legacy->pid_kp_milli % PID_KP_STEP_MILLI) == 0L) &&
      (legacy->pid_ki_milli >= 0) &&
      (legacy->pid_ki_milli <= PID_GAIN_MAX_MILLI) &&
      (legacy->pid_kd_milli >= 0) &&
      (legacy->pid_kd_milli <= PID_GAIN_MAX_MILLI))
  {
    settings->pid_kp_milli = legacy->pid_kp_milli;
    settings->pid_ki_milli = legacy->pid_ki_milli;
    settings->pid_kd_milli = legacy->pid_kd_milli;
  }

  for (joint = 0U; joint < ROBOT_SETTINGS_SERVO_COUNT; joint++)
  {
    lower = (default_servo_pulse_us[joint][0] <
             default_servo_pulse_us[joint][2])
                ? default_servo_pulse_us[joint][0]
                : default_servo_pulse_us[joint][2];
    upper = (default_servo_pulse_us[joint][0] >
             default_servo_pulse_us[joint][2])
                ? default_servo_pulse_us[joint][0]
                : default_servo_pulse_us[joint][2];

    for (point = 0U; point < ROBOT_SETTINGS_PULSE_COUNT; point++)
    {
      settings->servo_pulse_us[joint][point] =
          ClampPulse((int32_t)default_servo_pulse_us[joint][point] +
                         legacy->servo_zero_trim_us[joint],
                     lower,
                     upper);
    }
  }

  settings->servo_pulse_us[5][1] =
      (uint16_t)((settings->servo_pulse_us[5][0] +
                  settings->servo_pulse_us[5][2]) /
                 2U);
}

/* 버전 5 이미지에서 Wrist Rotate의 새 논리 방향만 반전한다. */
static void MigrateV5Settings(const RobotSettingsV6 *legacy,
                              RobotSettings *settings)
{
  uint16_t endpoint;

  MigrateV6Settings(legacy, settings);
  endpoint = settings->servo_pulse_us[4][0];
  settings->servo_pulse_us[4][0] = settings->servo_pulse_us[4][2];
  settings->servo_pulse_us[4][2] = endpoint;
}

/* 버전 4 이미지를 현재 저장 형식으로 변환한다. */
static void MigrateV4Settings(const RobotSettingsV6 *legacy,
                               RobotSettings *settings)
{
  uint16_t endpoint;

  MigrateV6Settings(legacy, settings);

  /* Shoulder와 Elbow의 표시 각도 방향이 바뀌어 두 끝점만 교환한다. */
  endpoint = settings->servo_pulse_us[1][0];
  settings->servo_pulse_us[1][0] = settings->servo_pulse_us[1][2];
  settings->servo_pulse_us[1][2] = endpoint;

  endpoint = settings->servo_pulse_us[2][0];
  settings->servo_pulse_us[2][0] = settings->servo_pulse_us[2][2];
  settings->servo_pulse_us[2][2] = endpoint;

  /* 현재 버전에서는 Wrist Rotate의 표시 각도 방향도 반전한다. */
  endpoint = settings->servo_pulse_us[4][0];
  settings->servo_pulse_us[4][0] = settings->servo_pulse_us[4][2];
  settings->servo_pulse_us[4][2] = endpoint;
}

/* Flash 이미지의 마지막 필드를 제외한 CRC-32를 계산한다. */
static uint32_t CalculateCrc(const void *data, size_t length)
{
  const uint8_t *bytes = data;
  uint32_t crc = 0xFFFFFFFFUL;
  size_t index;
  uint8_t bit;

  for (index = 0U; index < length; index++)
  {
    crc ^= bytes[index];
    for (bit = 0U; bit < 8U; bit++)
    {
      if ((crc & 1U) != 0U)
      {
        crc = (crc >> 1U) ^ 0xEDB88320UL;
      }
      else
      {
        crc >>= 1U;
      }
    }
  }

  return ~crc;
}

/* 시퀀스 개수와 모든 웨이포인트 각도의 범위를 검사한다. */
static uint8_t AreWaypointsValid(const TeachingWaypoint *waypoints,
                                 uint8_t count)
{
  uint8_t waypoint;
  uint8_t joint;

  if ((waypoints == NULL) || (count > TEACHING_MAX_WAYPOINTS))
  {
    return 0U;
  }

  for (waypoint = 0U; waypoint < count; waypoint++)
  {
    for (joint = 0U; joint < TEACHING_JOINT_COUNT; joint++)
    {
      if (waypoints[waypoint].angles[joint] > 180U)
      {
        return 0U;
      }
    }
  }

  return 1U;
}

static uint8_t IsLegacySequenceValid(const TeachingSequenceV7 *sequence)
{
  return (uint8_t)((sequence != NULL) &&
                   (AreWaypointsValid(sequence->waypoints,
                                      sequence->count) != 0U));
}

/* 이름은 UTF-8 바이트로 보관하며 NUL과 ASCII 제어 문자는 허용하지 않는다. */
static uint8_t IsSequenceValid(const TeachingSequence *sequence)
{
  uint8_t index;

  if ((sequence == NULL) ||
      (AreWaypointsValid(sequence->waypoints, sequence->count) == 0U) ||
      (sequence->name_length < 1U) ||
      (sequence->name_length > TEACHING_NAME_MAX_BYTES) ||
      (sequence->name[sequence->name_length] != 0U))
  {
    return 0U;
  }

  for (index = 0U; index < sequence->name_length; index++)
  {
    if ((sequence->name[index] == 0U) ||
        (sequence->name[index] < 0x20U) ||
        (sequence->name[index] == 0x7FU))
    {
      return 0U;
    }
  }
  return 1U;
}

/* 현재 버전 Flash 이미지의 Magic, Version, 내용과 CRC를 검사한다. */
static uint8_t IsImageValid(const TeachingImage *image)
{
  uint8_t sequence;

  if ((image->magic != STORAGE_MAGIC) ||
      (image->version != STORAGE_VERSION) ||
      (image->size != sizeof(TeachingImage)) ||
      (image->crc != CalculateCrc(image, offsetof(TeachingImage, crc))))
  {
    return 0U;
  }

  for (sequence = 0U; sequence < TEACHING_SEQUENCE_COUNT; sequence++)
  {
    if (IsSequenceValid(&image->sequences[sequence]) == 0U)
    {
      return 0U;
    }
  }

  return 1U;
}

static uint8_t IsV2ImageValid(const TeachingImageV2 *image)
{
  uint8_t sequence;

  if ((image->magic != STORAGE_MAGIC) ||
      (image->version != STORAGE_VERSION_V2) ||
      (image->size != sizeof(TeachingImageV2)) ||
      (image->crc != CalculateCrc(image, offsetof(TeachingImageV2, crc))))
  {
    return 0U;
  }

  for (sequence = 0U; sequence < TEACHING_SEQUENCE_COUNT; sequence++)
  {
    if (IsLegacySequenceValid(&image->sequences[sequence]) == 0U)
    {
      return 0U;
    }
  }

  return 1U;
}

static uint8_t IsV3ImageValid(const TeachingImageV3 *image)
{
  uint8_t sequence;

  if ((image->magic != STORAGE_MAGIC) ||
      (image->version != STORAGE_VERSION_V3) ||
      (image->size != sizeof(TeachingImageV3)) ||
      (image->crc != CalculateCrc(image, offsetof(TeachingImageV3, crc))))
  {
    return 0U;
  }

  for (sequence = 0U; sequence < TEACHING_SEQUENCE_COUNT; sequence++)
  {
    if (IsLegacySequenceValid(&image->sequences[sequence]) == 0U)
    {
      return 0U;
    }
  }

  return 1U;
}

static uint8_t IsV4ImageValid(const TeachingImageV4 *image)
{
  uint8_t sequence;

  if ((image->magic != STORAGE_MAGIC) ||
      (image->version != STORAGE_VERSION_V4) ||
      (image->size != sizeof(TeachingImageV4)) ||
      (image->crc != CalculateCrc(image, offsetof(TeachingImageV4, crc))))
  {
    return 0U;
  }

  for (sequence = 0U; sequence < TEACHING_SEQUENCE_COUNT; sequence++)
  {
    if (IsLegacySequenceValid(&image->sequences[sequence]) == 0U)
    {
      return 0U;
    }
  }

  return 1U;
}

static uint8_t IsV5ImageValid(const TeachingImageV5 *image)
{
  uint8_t sequence;

  if ((image->magic != STORAGE_MAGIC) ||
      (image->version != STORAGE_VERSION_V5) ||
      (image->size != sizeof(TeachingImageV5)) ||
      (image->crc != CalculateCrc(image, offsetof(TeachingImageV5, crc))))
  {
    return 0U;
  }

  for (sequence = 0U; sequence < TEACHING_SEQUENCE_COUNT; sequence++)
  {
    if (IsLegacySequenceValid(&image->sequences[sequence]) == 0U)
    {
      return 0U;
    }
  }

  return 1U;
}

static uint8_t IsV6ImageValid(const TeachingImageV6 *image)
{
  uint8_t sequence;

  if ((image->magic != STORAGE_MAGIC) ||
      (image->version != STORAGE_VERSION_V6) ||
      (image->size != sizeof(TeachingImageV6)) ||
      (image->crc != CalculateCrc(image, offsetof(TeachingImageV6, crc))))
  {
    return 0U;
  }

  for (sequence = 0U; sequence < TEACHING_SEQUENCE_COUNT; sequence++)
  {
    if (IsLegacySequenceValid(&image->sequences[sequence]) == 0U)
    {
      return 0U;
    }
  }

  return 1U;
}

static uint8_t IsV7ImageValid(const TeachingImageV7 *image)
{
  uint8_t sequence;

  if ((image->magic != STORAGE_MAGIC) ||
      (image->version != STORAGE_VERSION_V7) ||
      (image->size != sizeof(TeachingImageV7)) ||
      (image->crc != CalculateCrc(image, offsetof(TeachingImageV7, crc))))
  {
    return 0U;
  }

  for (sequence = 0U; sequence < TEACHING_SEQUENCE_COUNT; sequence++)
  {
    if (IsLegacySequenceValid(&image->sequences[sequence]) == 0U)
    {
      return 0U;
    }
  }
  return 1U;
}

/* 유효한 RAM 이미지를 Sector 7에 기록하고 다시 읽어 비교한다. */
static HAL_StatusTypeDef SaveAllData(void)
{
  FLASH_EraseInitTypeDef erase = {0};
  uint32_t sector_error = 0U;
  uint32_t address;
  uint32_t word;
  size_t offset;
  HAL_StatusTypeDef status = HAL_OK;

  /* 2 KiB가 넘는 이미지는 작은 RTOS 태스크 스택 대신 정적 RAM을 사용한다. */
  memset(&storage_image, 0, sizeof(storage_image));
  storage_image.magic = STORAGE_MAGIC;
  storage_image.version = STORAGE_VERSION;
  storage_image.size = (uint16_t)sizeof(TeachingImage);
  memcpy(storage_image.sequences, sequences, sizeof(sequences));
  storage_image.settings = robot_settings;
  storage_image.crc =
      CalculateCrc(&storage_image, offsetof(TeachingImage, crc));

  if (HAL_FLASH_Unlock() != HAL_OK)
  {
    return HAL_ERROR;
  }

  erase.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  erase.Sector = STORAGE_FLASH_SECTOR;
  erase.NbSectors = 1U;

  if (HAL_FLASHEx_Erase(&erase, &sector_error) != HAL_OK)
  {
    status = HAL_ERROR;
  }

  address = STORAGE_FLASH_ADDRESS;
  for (offset = 0U;
       (status == HAL_OK) && (offset < sizeof(storage_image));
       offset += sizeof(word))
  {
    memcpy(&word,
           ((const uint8_t *)&storage_image) + offset,
           sizeof(word));
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, word) != HAL_OK)
    {
      status = HAL_ERROR;
    }
    address += sizeof(word);
  }

  (void)HAL_FLASH_Lock();

  /* 기록한 내용 전체를 다시 비교해 전원이나 Flash 오류를 확인한다. */
  if ((status == HAL_OK) &&
      (memcmp((const void *)STORAGE_FLASH_ADDRESS,
              &storage_image,
              sizeof(storage_image)) != 0))
  {
    status = HAL_ERROR;
  }

  return status;
}

void TeachingStorage_Init(void)
{
  const TeachingImage *flash_image =
      (const TeachingImage *)STORAGE_FLASH_ADDRESS;
  const TeachingImageV2 *flash_image_v2 =
      (const TeachingImageV2 *)STORAGE_FLASH_ADDRESS;
  const TeachingImageV3 *flash_image_v3 =
      (const TeachingImageV3 *)STORAGE_FLASH_ADDRESS;
  const TeachingImageV4 *flash_image_v4 =
      (const TeachingImageV4 *)STORAGE_FLASH_ADDRESS;
  const TeachingImageV5 *flash_image_v5 =
      (const TeachingImageV5 *)STORAGE_FLASH_ADDRESS;
  const TeachingImageV6 *flash_image_v6 =
      (const TeachingImageV6 *)STORAGE_FLASH_ADDRESS;
  const TeachingImageV7 *flash_image_v7 =
      (const TeachingImageV7 *)STORAGE_FLASH_ADDRESS;

  memset(sequences, 0, sizeof(sequences));
  LoadDefaultSequenceNames();
  LoadDefaultSettings(&robot_settings);
  memset(&upload_sequence, 0, sizeof(upload_sequence));
  memset(upload_parts, 0, sizeof(upload_parts));
  upload_active = 0U;
  upload_name_parts = 0U;

  if (IsImageValid(flash_image) != 0U)
  {
    memcpy(sequences, flash_image->sequences, sizeof(sequences));
    robot_settings = flash_image->settings;
    if (IsPidSettingsValid(&robot_settings) == 0U)
    {
      LoadDefaultPidSettings(&robot_settings);
    }
    if (IsTravelPoseValid(&robot_settings) == 0U)
    {
      LoadDefaultTravelPose(&robot_settings);
    }
  }
  else if (IsV7ImageValid(flash_image_v7) != 0U)
  {
    /* v7 웨이포인트와 설정을 보존하고 시퀀스 기본 이름만 추가한다. */
    MigrateLegacySequences(flash_image_v7->sequences);
    robot_settings = flash_image_v7->settings;
    if (IsPidSettingsValid(&robot_settings) == 0U)
    {
      LoadDefaultPidSettings(&robot_settings);
    }
    if (IsTravelPoseValid(&robot_settings) == 0U)
    {
      LoadDefaultTravelPose(&robot_settings);
    }
  }
  else if (IsV6ImageValid(flash_image_v6) != 0U)
  {
    /* 기존 티칭·PID·서보 보정은 보존하고 이동 자세 기본값만 추가한다. */
    MigrateLegacySequences(flash_image_v6->sequences);
    MigrateV6Settings(&flash_image_v6->settings, &robot_settings);
    if (IsPidSettingsValid(&robot_settings) == 0U)
    {
      LoadDefaultPidSettings(&robot_settings);
    }
  }
  else if (IsV5ImageValid(flash_image_v5) != 0U)
  {
    /* 티칭과 설정을 보존하며 Wrist Rotate의 두 끝점만 교환한다. */
    MigrateLegacySequences(flash_image_v5->sequences);
    MigrateV5Settings(&flash_image_v5->settings, &robot_settings);
    if (IsPidSettingsValid(&robot_settings) == 0U)
    {
      LoadDefaultPidSettings(&robot_settings);
    }
  }
  else if (IsV4ImageValid(flash_image_v4) != 0U)
  {
    /* 기존 설정을 보존하며 변경된 세 관절의 각도 방향만 뒤집는다. */
    MigrateLegacySequences(flash_image_v4->sequences);
    MigrateV4Settings(&flash_image_v4->settings, &robot_settings);
    if (IsPidSettingsValid(&robot_settings) == 0U)
    {
      LoadDefaultPidSettings(&robot_settings);
    }
  }
  else if (IsV3ImageValid(flash_image_v3) != 0U)
  {
    /* 기존 티칭/PID와 원점 이동량을 새 3점 보정 형식으로 변환한다. */
    MigrateLegacySequences(flash_image_v3->sequences);
    MigrateV3Settings(&flash_image_v3->settings, &robot_settings);
  }
  else if (IsV2ImageValid(flash_image_v2) != 0U)
  {
    /* 기존 v2 티칭 데이터는 보존하고 새 설정은 기본 보정값을 사용한다. */
    MigrateLegacySequences(flash_image_v2->sequences);
  }
}

HAL_StatusTypeDef TeachingStorage_BeginUpload(uint8_t sequence_id,
                                              uint8_t waypoint_count,
                                              uint8_t name_length)
{
  if ((sequence_id < 1U) ||
      (sequence_id > TEACHING_SEQUENCE_COUNT) ||
      (waypoint_count < 1U) ||
      (waypoint_count > TEACHING_MAX_WAYPOINTS) ||
      (name_length < 1U) ||
      (name_length > TEACHING_NAME_MAX_BYTES))
  {
    return HAL_ERROR;
  }

  memset(&upload_sequence, 0, sizeof(upload_sequence));
  memset(upload_parts, 0, sizeof(upload_parts));
  upload_sequence.count = waypoint_count;
  upload_sequence.name_length = name_length;
  upload_sequence_id = sequence_id;
  upload_active = 1U;
  upload_name_parts = 0U;
  return HAL_OK;
}

HAL_StatusTypeDef TeachingStorage_WriteHalf(uint8_t sequence_id,
                                            uint8_t waypoint_index,
                                            uint8_t second_half,
                                            const uint8_t angles[3])
{
  uint8_t joint;
  uint8_t angle_offset = (second_half != 0U) ? 3U : 0U;
  uint8_t part_mask = (second_half != 0U) ? 2U : 1U;

  if ((upload_active == 0U) ||
      (sequence_id != upload_sequence_id) ||
      (waypoint_index >= upload_sequence.count) ||
      (angles == NULL))
  {
    return HAL_ERROR;
  }

  for (joint = 0U; joint < 3U; joint++)
  {
    if (angles[joint] > 180U)
    {
      return HAL_ERROR;
    }
    upload_sequence.waypoints[waypoint_index]
        .angles[angle_offset + joint] = angles[joint];
  }

  upload_parts[waypoint_index] |= part_mask;
  return HAL_OK;
}

HAL_StatusTypeDef TeachingStorage_WriteNamePart(uint8_t sequence_id,
                                                uint8_t part,
                                                const uint8_t name[8])
{
  uint8_t index;
  uint8_t length;
  uint8_t offset;

  if ((upload_active == 0U) ||
      (sequence_id != upload_sequence_id) ||
      (part >= TEACHING_NAME_PART_COUNT) ||
      (name == NULL))
  {
    return HAL_ERROR;
  }

  offset = (uint8_t)(part * TEACHING_NAME_PART_SIZE);
  if (offset >= upload_sequence.name_length)
  {
    return HAL_ERROR;
  }
  length = (uint8_t)(upload_sequence.name_length - offset);
  if (length > TEACHING_NAME_PART_SIZE)
  {
    length = TEACHING_NAME_PART_SIZE;
  }

  for (index = 0U; index < length; index++)
  {
    if ((name[index] == 0U) ||
        (name[index] < 0x20U) ||
        (name[index] == 0x7FU))
    {
      return HAL_ERROR;
    }
  }

  memcpy(&upload_sequence.name[offset], name, length);
  upload_sequence.name[upload_sequence.name_length] = 0U;
  upload_name_parts |= (uint8_t)(1U << part);
  return HAL_OK;
}

static uint8_t IsUploadComplete(uint8_t sequence_id)
{
  uint8_t name_part_count;
  uint8_t required_name_parts;
  uint8_t waypoint;

  if ((upload_active == 0U) ||
      (sequence_id != upload_sequence_id))
  {
    return 0U;
  }

  name_part_count = (uint8_t)((upload_sequence.name_length +
                               TEACHING_NAME_PART_SIZE - 1U) /
                              TEACHING_NAME_PART_SIZE);
  required_name_parts = (uint8_t)((1U << name_part_count) - 1U);
  if ((upload_name_parts != required_name_parts) ||
      (IsSequenceValid(&upload_sequence) == 0U) ||
      (IsReservedSequenceNameValid(sequence_id, &upload_sequence) == 0U))
  {
    return 0U;
  }

  for (waypoint = 0U; waypoint < upload_sequence.count; waypoint++)
  {
    if (upload_parts[waypoint] != 3U)
    {
      return 0U;
    }
  }
  return 1U;
}

const TeachingSequence *TeachingStorage_GetUpload(uint8_t sequence_id)
{
  return (IsUploadComplete(sequence_id) != 0U) ? &upload_sequence : NULL;
}

HAL_StatusTypeDef TeachingStorage_Commit(uint8_t sequence_id)
{
  TeachingSequence previous;
  HAL_StatusTypeDef status;

  if (IsUploadComplete(sequence_id) == 0U)
  {
    return HAL_ERROR;
  }

  previous = sequences[sequence_id - 1U];
  sequences[sequence_id - 1U] = upload_sequence;
  status = SaveAllData();
  if (status != HAL_OK)
  {
    sequences[sequence_id - 1U] = previous;
    return status;
  }

  upload_active = 0U;
  upload_name_parts = 0U;
  return HAL_OK;
}

HAL_StatusTypeDef TeachingStorage_Reset(uint8_t sequence_id)
{
  TeachingSequence previous;
  HAL_StatusTypeDef status;

  if ((sequence_id < 1U) || (sequence_id > TEACHING_SEQUENCE_COUNT))
  {
    return HAL_ERROR;
  }

  previous = sequences[sequence_id - 1U];
  memset(&sequences[sequence_id - 1U],
         0,
         sizeof(TeachingSequence));
  LoadDefaultSequenceName((uint8_t)(sequence_id - 1U),
                          &sequences[sequence_id - 1U]);
  status = SaveAllData();
  if (status != HAL_OK)
  {
    sequences[sequence_id - 1U] = previous;
  }

  return status;
}

const TeachingSequence *TeachingStorage_Get(uint8_t sequence_id)
{
  if ((sequence_id < 1U) || (sequence_id > TEACHING_SEQUENCE_COUNT))
  {
    return NULL;
  }

  return &sequences[sequence_id - 1U];
}

void TeachingStorage_GetSettings(RobotSettings *settings)
{
  if (settings != NULL)
  {
    *settings = robot_settings;
  }
}

void TeachingStorage_ResetServoCalibrations(void)
{
  LoadDefaultServoCalibrations(&robot_settings);
}

HAL_StatusTypeDef TeachingStorage_SaveSettings(const RobotSettings *settings)
{
  RobotSettings previous;
  HAL_StatusTypeDef status;

  if ((IsPidSettingsValid(settings) == 0U) ||
      (IsTravelPoseValid(settings) == 0U))
  {
    return HAL_ERROR;
  }

  previous = robot_settings;
  robot_settings = *settings;
  status = SaveAllData();
  if (status != HAL_OK)
  {
    robot_settings = previous;
  }

  return status;
}
