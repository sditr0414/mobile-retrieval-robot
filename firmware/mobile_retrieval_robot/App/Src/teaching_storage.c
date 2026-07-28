#include "teaching_storage.h"

#include <stddef.h>
#include <string.h>

#define STORAGE_MAGIC          (0x4D525254UL)
#define STORAGE_VERSION        (2U)
#define STORAGE_FLASH_ADDRESS  (0x08060000UL)
#define STORAGE_FLASH_SECTOR   FLASH_SECTOR_7

typedef struct
{
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  TeachingSequence sequences[TEACHING_SEQUENCE_COUNT];
  uint32_t crc;
} TeachingImage;

_Static_assert(sizeof(TeachingImage) <= (128U * 1024U),
               "Teaching image must fit in STM32 Flash sector 7");

static TeachingSequence sequences[TEACHING_SEQUENCE_COUNT];
static TeachingSequence upload_sequence;
static TeachingImage storage_image;
static uint8_t upload_parts[TEACHING_MAX_WAYPOINTS];
static uint8_t upload_sequence_id;
static uint8_t upload_active;

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

static uint8_t IsSequenceValid(const TeachingSequence *sequence)
{
  uint8_t waypoint;
  uint8_t joint;

  if (sequence->count > TEACHING_MAX_WAYPOINTS)
  {
    return 0U;
  }

  for (waypoint = 0U; waypoint < sequence->count; waypoint++)
  {
    for (joint = 0U; joint < TEACHING_JOINT_COUNT; joint++)
    {
      if (sequence->waypoints[waypoint].angles[joint] > 180U)
      {
        return 0U;
      }
    }
  }

  return 1U;
}

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

static HAL_StatusTypeDef SaveAllSequences(void)
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

  memset(sequences, 0, sizeof(sequences));
  memset(&upload_sequence, 0, sizeof(upload_sequence));
  memset(upload_parts, 0, sizeof(upload_parts));
  upload_active = 0U;

  if (IsImageValid(flash_image) != 0U)
  {
    memcpy(sequences, flash_image->sequences, sizeof(sequences));
  }
}

HAL_StatusTypeDef TeachingStorage_BeginUpload(uint8_t sequence_id,
                                              uint8_t waypoint_count)
{
  if ((sequence_id < 1U) ||
      (sequence_id > TEACHING_SEQUENCE_COUNT) ||
      (waypoint_count < 1U) ||
      (waypoint_count > TEACHING_MAX_WAYPOINTS))
  {
    return HAL_ERROR;
  }

  memset(&upload_sequence, 0, sizeof(upload_sequence));
  memset(upload_parts, 0, sizeof(upload_parts));
  upload_sequence.count = waypoint_count;
  upload_sequence_id = sequence_id;
  upload_active = 1U;
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

HAL_StatusTypeDef TeachingStorage_Commit(uint8_t sequence_id)
{
  TeachingSequence previous;
  uint8_t waypoint;
  HAL_StatusTypeDef status;

  if ((upload_active == 0U) ||
      (sequence_id != upload_sequence_id))
  {
    return HAL_ERROR;
  }

  for (waypoint = 0U; waypoint < upload_sequence.count; waypoint++)
  {
    if (upload_parts[waypoint] != 3U)
    {
      return HAL_ERROR;
    }
  }

  previous = sequences[sequence_id - 1U];
  sequences[sequence_id - 1U] = upload_sequence;
  status = SaveAllSequences();
  if (status != HAL_OK)
  {
    sequences[sequence_id - 1U] = previous;
    return status;
  }

  upload_active = 0U;
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
  status = SaveAllSequences();
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
