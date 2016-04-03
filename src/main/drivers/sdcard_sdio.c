/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Defines used by this driver:
 * #define USE_SDCARD
 * #define USE_SDIO
 * #define USE_4BIT_SDIO
 * #define AFATFS_USE_INTROSPECTIVE_LOGGING
 * #define SDCARD_DETECT_PIN PB14
 * #define SDCARD_DETECT_INVERTED
 *
 */

#include "sdcard.h"

#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "nvic.h"
#include "io.h"
#include "common/maths.h"

#include "drivers/bus_spi.h"
#include "drivers/system.h"

#include "sdcard_standard.h"
#include "fatfs_sd_sdio.h"

#ifdef USE_SDCARD

#ifdef AFATFS_USE_INTROSPECTIVE_LOGGING
    #define SDCARD_PROFILING
#endif

#define SDCARD_INIT_NUM_DUMMY_BYTES 10
#define SDCARD_MAXIMUM_BYTE_DELAY_FOR_CMD_REPLY 8
// Chosen so that CMD8 will have the same CRC as CMD0:
#define SDCARD_IF_COND_CHECK_PATTERN 0xAB

#define SDCARD_TIMEOUT_INIT_MILLIS      2000
#define SDCARD_MAX_CONSECUTIVE_FAILURES 8

/* Break up 512-byte SD card sectors into chunks of this size when writing without DMA to reduce the peak overhead
 * per call to sdcard_poll().
 */
#define SDCARD_NON_DMA_CHUNK_SIZE 256

#define STATIC_ASSERT(condition, name ) \
    typedef char assert_failed_ ## name [(condition) ? 1 : -1 ]

typedef enum {
    // In these states we run at the initialization 400kHz clockspeed:
    SDCARD_STATE_NOT_PRESENT = 0,
    SDCARD_STATE_RESET,
    SDCARD_STATE_CARD_INIT_IN_PROGRESS,

    // In these states we run at full clock speed
    SDCARD_STATE_READY,
    SDCARD_STATE_READING,
    SDCARD_STATE_SENDING_WRITE,
    SDCARD_STATE_WAITING_FOR_WRITE,
    SDCARD_STATE_WRITING_MULTIPLE_BLOCKS,
    SDCARD_STATE_STOPPING_MULTIPLE_BLOCK_WRITE,
} sdcardState_e;

typedef struct sdcard_t {
    struct {
        uint8_t *buffer;
        uint32_t blockIndex;
        uint8_t chunkIndex;

        sdcard_operationCompleteCallback_c callback;
        uint32_t callbackData;

#ifdef SDCARD_PROFILING
        uint32_t profileStartTime;
#endif
    } pendingOperation;

    uint32_t operationStartTime;

    uint8_t failureCount;

    uint8_t version;
    bool highCapacity;

    uint32_t multiWriteNextBlock;
    uint32_t multiWriteBlocksRemain;

    sdcardState_e state;

    sdcardMetadata_t metadata;
    sdcardCSD_t csd;

    SD_CardInfo cardinfo;

#ifdef SDCARD_PROFILING
    sdcard_profilerCallback_c profiler;
#endif
} sdcard_t;

static sdcard_t sdcard;

#ifdef SDCARD_DETECT_PIN
static IO_t sdCardDetctPin = IO_NONE;
#endif

#ifdef USE_4BIT_SDIO
#define FATFS_SDIO_4BIT 1
#endif

    // hard-code this to use DMA
    static const bool useDMAForTx = true;

STATIC_ASSERT(sizeof(sdcardCSD_t) == 16, sdcard_csd_bitfields_didnt_pack_properly);

void sdcardInsertionDetectDeinit(void)
{
#ifdef SDCARD_DETECT_PIN
    sdCardDetctPin = IOGetByTag(IO_TAG(SDCARD_DETECT_PIN));
    IOInit(sdCardDetctPin, OWNER_SYSTEM, RESOURCE_INPUT);
    IOConfigGPIO(sdCardDetctPin, IOCFG_IN_FLOATING);
#endif
}

void sdcardInsertionDetectInit(void)
{
#ifdef SDCARD_DETECT_PIN
    sdCardDetctPin = IOGetByTag(IO_TAG(SDCARD_DETECT_PIN));
    IOInit(sdCardDetctPin, OWNER_SYSTEM, RESOURCE_INPUT);
    IOConfigGPIO(sdCardDetctPin, IOCFG_IPU);
#endif
}

/**
 * Detect if a SD card is physically present in the memory slot.
 */
bool sdcard_isInserted(void)
{
    bool result = true;

#ifdef SDCARD_DETECT_PIN
    result = IORead(sdCardDetctPin) != 0;
#ifdef SDCARD_DETECT_INVERTED
    result = !result;
#endif
#endif

    return result;
}

/**
 * Returns true if the card has already been, or is currently, initializing and hasn't encountered enough errors to
 * trip our error threshold and be disabled (i.e. our card is in and working!)
 */
bool sdcard_isFunctional(void)
{
    return sdcard.state != SDCARD_STATE_NOT_PRESENT;
}

/**
 * Handle a failure of an SD card operation by resetting the card back to its initialization phase.
 *
 * Increments the failure counter, and when the failure threshold is reached, disables the card until
 * the next call to sdcard_init().
 */
static void sdcard_reset(void)
{
    sdcard.failureCount++;
    if (sdcard.failureCount >= SDCARD_MAX_CONSECUTIVE_FAILURES) {
        sdcard.state = SDCARD_STATE_NOT_PRESENT;
    } else {
        sdcard.operationStartTime = millis();
        sdcard.state = SDCARD_STATE_RESET;
    }
}

typedef enum {
    SDCARD_RECEIVE_SUCCESS,
    SDCARD_RECEIVE_BLOCK_IN_PROGRESS,
    SDCARD_RECEIVE_ERROR,
} sdcardReceiveBlockStatus_e;

/**
 * Begin the initialization process for the SD card. This must be called first before any other sdcard_ routine.
 */
void sdcard_init(bool useDMA)
{
    // DMA is always used
    (void) useDMA;

    SD_DiskInit();    

    sdcard.operationStartTime = millis();
    sdcard.state = SDCARD_STATE_RESET;
    sdcard.failureCount = 0;
}

/*
 * Returns true if the card is ready to accept read/write commands.
 */
static bool sdcard_isReady()
{
    return sdcard.state == SDCARD_STATE_READY;
}

/**
 * Call periodically for the SD card to perform in-progress transfers.
 *
 * Returns true if the card is ready to accept commands.
 */
bool sdcard_poll(void)
{
#ifdef SDCARD_PROFILING
    bool profilingComplete;
#endif

    doMore:
    switch (sdcard.state) {
        case SDCARD_STATE_RESET:
            SD_LowLevel_DeInit();
            SD_LowLevel_Init();

            if (SD_Init() == SD_OK) {
                // Check card voltage and version
                if (SD_GetCardInfo(&sdcard.cardinfo) == SD_OK) {
                    sdcard.state = SDCARD_STATE_CARD_INIT_IN_PROGRESS;
                    goto doMore;
                } else {
                    // Bad reply/voltage, we ought to refrain from accessing the card.
                    sdcard.state = SDCARD_STATE_NOT_PRESENT;
                }
            }
        break;

        case SDCARD_STATE_CARD_INIT_IN_PROGRESS:
            if (SD_GetState() == SD_CARD_TRANSFER) {
                sdcard.metadata.manufacturerID = sdcard.cardinfo.SD_cid.ManufacturerID;//uint8_t
                sdcard.metadata.oemID = sdcard.cardinfo.SD_cid.OEM_AppliID;//uint16_t
                sdcard.metadata.productName[0] = (sdcard.cardinfo.SD_cid.ProdName1 >> 24) & 0xFF;//char[5]
                sdcard.metadata.productName[1] = (sdcard.cardinfo.SD_cid.ProdName1 >> 16) & 0xFF;
                sdcard.metadata.productName[2] = (sdcard.cardinfo.SD_cid.ProdName1 >> 8) & 0xFF;
                sdcard.metadata.productName[3] = sdcard.cardinfo.SD_cid.ProdName1 & 0xFF;
                sdcard.metadata.productName[4] = sdcard.cardinfo.SD_cid.ProdName2;
                sdcard.metadata.productRevisionMajor = sdcard.cardinfo.SD_cid.ProdRev >> 4;//uint8_t
                sdcard.metadata.productRevisionMinor = sdcard.cardinfo.SD_cid.ProdRev & 0x0F;//uint8_t
                sdcard.metadata.productSerial = sdcard.cardinfo.SD_cid.ProdSN;//uint32_t
                sdcard.metadata.productionYear = sdcard.cardinfo.SD_cid.ManufactDate >> 4;//uint16_t
                sdcard.metadata.productionMonth = sdcard.cardinfo.SD_cid.ManufactDate & 0x0F; //uint8_t
                sdcard.metadata.numBlocks = sdcard.cardinfo.CardCapacity / sdcard.cardinfo.CardBlockSize;//uint32_t

                sdcard.version = (sdcard.cardinfo.CardType == SDIO_STD_CAPACITY_SD_CARD_V1_1) ? 1 : 2;
                sdcard.highCapacity = (sdcard.cardinfo.CardType == SDIO_HIGH_CAPACITY_SD_CARD);

                sdcard.multiWriteBlocksRemain = 0;

                sdcard.state = SDCARD_STATE_READY;
                goto doMore;
            } // else keep waiting for the CID to arrive
        break;
        case SDCARD_STATE_SENDING_WRITE:
          // Have we finished sending the write yet?
          if (SD_WaitWriteOperation() == SD_OK) {
            // The SD card is now busy committing that write to the card
            sdcard.state = SDCARD_STATE_WAITING_FOR_WRITE;
            sdcard.operationStartTime = millis();

            // Since we've transmitted the buffer we can go ahead and tell the caller their operation is complete
            if (sdcard.pendingOperation.callback) {
              sdcard.pendingOperation.callback(SDCARD_BLOCK_OPERATION_WRITE, sdcard.pendingOperation.blockIndex, sdcard.pendingOperation.buffer, sdcard.pendingOperation.callbackData);
            }
          } else {
            /* Our write was rejected! This could be due to a bad address but we hope not to attempt that, so assume
             * the card is broken and needs reset.
             */
            sdcard_reset();
            // Announce write failure:
            if (sdcard.pendingOperation.callback) {
               sdcard.pendingOperation.callback(SDCARD_BLOCK_OPERATION_WRITE, sdcard.pendingOperation.blockIndex, NULL, sdcard.pendingOperation.callbackData);
            }
            goto doMore;
          }
        break;
        case SDCARD_STATE_WAITING_FOR_WRITE:
            if (SD_GetStatus() == SD_TRANSFER_OK) {
#ifdef SDCARD_PROFILING
                profilingComplete = true;
#endif

                sdcard.failureCount = 0; // Assume the card is good if it can complete a write
/*
                // Still more blocks left to write in a multi-block chain?
                if (sdcard.multiWriteBlocksRemain > 1) {
                    sdcard.multiWriteBlocksRemain--;
                    sdcard.multiWriteNextBlock++;
                    sdcard.state = SDCARD_STATE_WRITING_MULTIPLE_BLOCKS;
                } else if (sdcard.multiWriteBlocksRemain == 1) {
                    // This function changes the sd card state for us whether immediately succesful or delayed:
                    if (sdcard_endWriteBlocks() == SDCARD_OPERATION_SUCCESS) {
		      goto doMore;
                    } else {
#ifdef SDCARD_PROFILING
                        // Wait for the multi-block write to be terminated before finishing timing
                        profilingComplete = false;
#endif
                    }
                } else {
                    sdcard.state = SDCARD_STATE_READY;
                }
*/
                sdcard.state = SDCARD_STATE_READY;

#ifdef SDCARD_PROFILING
                if (profilingComplete && sdcard.profiler) {
                    sdcard.profiler(SDCARD_BLOCK_OPERATION_WRITE, sdcard.pendingOperation.blockIndex, micros() - sdcard.pendingOperation.profileStartTime);
                }
#endif
            } else if (millis() > sdcard.operationStartTime + SDCARD_TIMEOUT_WRITE_MSEC) {
                /*
                 * The caller has already been told that their write has completed, so they will have discarded
                 * their buffer and have no hope of retrying the operation. But this should be very rare and it allows
                 * them to reuse their buffer milliseconds faster than they otherwise would.
                 */
                sdcard_reset();
                goto doMore;
            }
        break;
        case SDCARD_STATE_READING:
            //switch (sdcard_receiveDataBlock(sdcard.pendingOperation.buffer, SDCARD_BLOCK_SIZE)) {
            switch (SD_GetStatus()) {
                case SD_TRANSFER_OK:
                    sdcard.state = SDCARD_STATE_READY;
                    sdcard.failureCount = 0; // Assume the card is good if it can complete a read

#ifdef SDCARD_PROFILING
                    if (sdcard.profiler) {
                        sdcard.profiler(SDCARD_BLOCK_OPERATION_READ, sdcard.pendingOperation.blockIndex, micros() - sdcard.pendingOperation.profileStartTime);
                    }
#endif

                    if (sdcard.pendingOperation.callback) {
                        sdcard.pendingOperation.callback(
                            SDCARD_BLOCK_OPERATION_READ,
                            sdcard.pendingOperation.blockIndex,
                            sdcard.pendingOperation.buffer,
                            sdcard.pendingOperation.callbackData
                        );
                    }
                break;
                case SD_TRANSFER_BUSY:
                    if (millis() <= sdcard.operationStartTime + SDCARD_TIMEOUT_READ_MSEC) {
                        break; // Timeout not reached yet so keep waiting
                    }
                    // Timeout has expired, so fall through to convert to a fatal error

                case SD_TRANSFER_ERROR:
                    sdcard_reset();

                    if (sdcard.pendingOperation.callback) {
                        sdcard.pendingOperation.callback(
                            SDCARD_BLOCK_OPERATION_READ,
                            sdcard.pendingOperation.blockIndex,
                            NULL,
                            sdcard.pendingOperation.callbackData
                        );
                    }

                    goto doMore;
                break;
            }
        break;
        case SDCARD_STATE_NOT_PRESENT:
        default:
            ;
    }

    // Is the card's initialization taking too long?
    if (sdcard.state >= SDCARD_STATE_RESET && sdcard.state < SDCARD_STATE_READY
            && millis() - sdcard.operationStartTime > SDCARD_TIMEOUT_INIT_MILLIS) {
        sdcard_reset();
    }

    return sdcard_isReady();
}

/**
 * Write the 512-byte block from the given buffer into the block with the given index.
 *
 * If the write does not complete immediately, your callback will be called later. If the write was successful, the
 * buffer pointer will be the same buffer you originally passed in, otherwise the buffer will be set to NULL.
 *
 * Returns:
 *     SDCARD_OPERATION_IN_PROGRESS - Your buffer is currently being transmitted to the card and your callback will be
 *                                    called later to report the completion. The buffer pointer must remain valid until
 *                                    that time.
 *     SDCARD_OPERATION_SUCCESS     - Your buffer has been transmitted to the card now.
 *     SDCARD_OPERATION_BUSY        - The card is already busy and cannot accept your write
 *     SDCARD_OPERATION_FAILURE     - Your write was rejected by the card, card will be reset
 */
sdcardOperationStatus_e sdcard_writeBlock(uint32_t blockIndex, uint8_t *buffer, sdcard_operationCompleteCallback_c callback, uint32_t callbackData)
{
#ifdef SDCARD_PROFILING
    sdcard.pendingOperation.profileStartTime = micros();
#endif

    switch (sdcard.state) {
        case SDCARD_STATE_READY:
            if (SD_GetState() != SD_CARD_TRANSFER && SD_GetStatus() != SD_TRANSFER_OK) {
                sdcard_reset();

                return SDCARD_OPERATION_FAILURE;
            }
        break;
        default:
            return SDCARD_OPERATION_BUSY;
    }

    SD_WriteBlock(buffer, blockIndex * SDCARD_BLOCK_SIZE, SDCARD_BLOCK_SIZE);

    sdcard.pendingOperation.buffer = buffer;
    sdcard.pendingOperation.blockIndex = blockIndex;
    sdcard.pendingOperation.callback = callback;
    sdcard.pendingOperation.callbackData = callbackData;
    sdcard.pendingOperation.chunkIndex = 1; // (for non-DMA transfers) we've sent chunk #0 already
    sdcard.state = SDCARD_STATE_SENDING_WRITE;

    return SDCARD_OPERATION_IN_PROGRESS;
}

/**
 * Begin writing a series of consecutive blocks beginning at the given block index. This will allow (but not require)
 * the SD card to pre-erase the number of blocks you specifiy, which can allow the writes to complete faster.
 *
 * Afterwards, just call sdcard_writeBlock() as normal to write those blocks consecutively.
 *
 * It's okay to abort the multi-block write at any time by writing to a non-consecutive address, or by performing a read.
 *
 * Returns:
 *     SDCARD_OPERATION_SUCCESS     - Multi-block write has been queued
 *     SDCARD_OPERATION_BUSY        - The card is already busy and cannot accept your write
 *     SDCARD_OPERATION_FAILURE     - A fatal error occured, card will be reset
 */
sdcardOperationStatus_e sdcard_beginWriteBlocks(uint32_t blockIndex, uint32_t blockCount)
{
    if (sdcard.state != SDCARD_STATE_READY) {
        return SDCARD_OPERATION_BUSY;
    }
    UNUSED(blockIndex);
    UNUSED(blockCount);

//    if (
//        sdcard_sendAppCommand(SDCARD_ACOMMAND_SET_WR_BLOCK_ERASE_COUNT, blockCount) == 0
//        && sdcard_sendCommand(SDCARD_COMMAND_WRITE_MULTIPLE_BLOCK, sdcard.highCapacity ? blockIndex : blockIndex * SDCARD_BLOCK_SIZE) == 0
//    ) {
/*
    if (SD_GetState() == SD_CARD_TRANSFER && SD_GetStatus() == SD_TRANSFER_OK) {
        sdcard.multiWriteBlocksRemain = blockCount;
        sdcard.multiWriteNextBlock = blockIndex;

        sdcard.state = SDCARD_STATE_SENDING_WRITE;
*/        // Leave the card selected
        return SDCARD_OPERATION_SUCCESS;
/*
    } else {
        sdcard_reset();

        return SDCARD_OPERATION_FAILURE;
    }
*/
}

/**
 * Read the 512-byte block with the given index into the given 512-byte buffer.
 *
 * When the read completes, your callback will be called. If the read was successful, the buffer pointer will be the
 * same buffer you originally passed in, otherwise the buffer will be set to NULL.
 *
 * You must keep the pointer to the buffer valid until the operation completes!
 *
 * Returns:
 *     true - The operation was successfully queued for later completion, your callback will be called later
 *     false - The operation could not be started due to the card being busy (try again later).
 */
bool sdcard_readBlock(uint32_t blockIndex, uint8_t *buffer, sdcard_operationCompleteCallback_c callback, uint32_t callbackData)
{
    if (sdcard.state != SDCARD_STATE_READY)
        return false;

#ifdef SDCARD_PROFILING
    sdcard.pendingOperation.profileStartTime = micros();
#endif

    // Standard size cards use byte addressing, high capacity cards use block addressing
    //uint8_t status = sdcard_sendCommand(SDCARD_COMMAND_READ_SINGLE_BLOCK, sdcard.highCapacity ? blockIndex : blockIndex * SDCARD_BLOCK_SIZE);
    SD_Error status = SD_ReadBlock(buffer, blockIndex * SDCARD_BLOCK_SIZE, SDCARD_BLOCK_SIZE);

    if (status == SD_OK) {
        if (SD_WaitReadOperation() != SD_OK)
            return false;
        sdcard.pendingOperation.buffer = buffer;
        sdcard.pendingOperation.blockIndex = blockIndex;
        sdcard.pendingOperation.callback = callback;
        sdcard.pendingOperation.callbackData = callbackData;
        
        sdcard.state = SDCARD_STATE_READING;

        sdcard.operationStartTime = millis();

        // Leave the card selected for the whole transaction

        return true;
    } else {
        return false;
    }
}

/**
 * Returns true if the SD card has successfully completed its startup procedures.
 */
bool sdcard_isInitialized(void)
{
    return sdcard.state >= SDCARD_STATE_READY;
}

const sdcardMetadata_t* sdcard_getMetadata(void)
{
    return &sdcard.metadata;
}

#ifdef SDCARD_PROFILING

void sdcard_setProfilerCallback(sdcard_profilerCallback_c callback)
{
    sdcard.profiler = callback;
}

#endif

#endif
