#ifndef SYNC_UART_H_
#define SYNC_UART_H_
#include "stdint.h"
#include "stdbool.h"

#define MAX_DATA_SIZE 300

/**
 * @brief Identifier for synchronous UART communication.
 *
 * This macro defines the default identifier used in the synchronous UART communication.
 * The value is set to 0x7F.
 */
#define SYNC_UART_IDENTIFY_RX 0x7F
#define SYNC_UART_IDENTIFY_TX 0x7E
#define SYNC_UART_ACK_BYTE 0x06
#define SYNC_UART_TX_TIME 1000

#define SYNC_STACK_SIZE 500
#define SYNC_PRIORITY 5

extern struct CO_Static_MemoryMaintenance_t Sync_uart_rxTx;

#define MAX_NODES 21
#define TIMEOUT_MS 1000
#define DEFAULT_NODE_ID 0x7F // Node-ID default para novos motores
#define SEQUENCIAL_COMMS 7

/**
 * @brief Structure representing the status word of a motor.
 *
 * This structure contains various status flags that indicate the current state
 * of a motor. Each flag is represented by a single bit.
 */
struct CO_Statusword_t {
    uint16_t ready_to_switch_on     : 1; /**< Bit 0: Ready to switch on */
    uint16_t switched_on            : 1; /**< Bit 1: Switched on */
    uint16_t operation_enabled      : 1; /**< Bit 2: Operation enabled */
    uint16_t fault                  : 1; /**< Bit 3: Fault */
    uint16_t voltage_enabled        : 1; /**< Bit 4: Voltage enabled */
    uint16_t quick_stop             : 1; /**< Bit 5: Quick stop */
    uint16_t switch_on_disabled     : 1; /**< Bit 6: Switch on disabled */
    uint16_t warning                : 1; /**< Bit 7: Warning */
    uint16_t manufacturer_specific1 : 1; /**< Bit 8: Manufacturer specific 1 */
    uint16_t remote                 : 1; /**< Bit 9: Remote */
    uint16_t target_reached         : 1; /**< Bit 10: Target reached */
    uint16_t internal_limit_active  : 1; /**< Bit 11: Internal limit active */
    uint16_t operation_specific     : 1; /**< Bit 12: Operation specific */
    uint16_t manufacturer_specific2 : 1; /**< Bit 13: Manufacturer specific 2 */
    uint16_t manufacturer_specific3 : 1; /**< Bit 14: Manufacturer specific 3 */
    uint16_t manufacturer_specific4 : 1; /**< Bit 15: Manufacturer specific 4 */
};

/**
 * @brief Structure representing the transmitted data of a motor.
 *
 * This structure contains information about the motor, such as its error code,
 * actual velocity, current, temperature, and status word.
 */
#pragma pack(push, 1)
struct CO_Motor_tx_t {
    uint16_t errorCode;                 /**< Error code of the motor */
    int16_t velocityActual;             /**< Actual velocity of the motor */
    uint16_t current;                   /**< Current of the motor */
    uint16_t temperature;               /**< Temperature of the motor */
    struct CO_Statusword_t statusWord;  /**< Status word of the motor */
};
#pragma pack(pop)


/**
 * @brief Structure representing the transmitted maintenance data.
 *
 * This structure contains information about the maintenance status, including the
 * maximum number of nodes, sequence error count, motor update status, and an array
 * of motor data.
 */
#pragma pack(push, 1)
struct CO_Maintenance_tx_t {
    uint8_t maxNodes;                  /**< Maximum number of supported motors */
    uint16_t sequency_error;                 /**< Number of sequence errors encountered */
    uint32_t motor_update;                   /**< Bitmask representing motor updates in the motors array */ // update: for state motor charge enter 0 - rpm target !
    struct CO_Motor_tx_t motors[MAX_NODES];  /**< Array of motor data structures */
};
#pragma pack(pop)


/**
 * @brief Structure representing the received maintenance data.
 *
 * This structure is intended to hold the received maintenance data. Currently, it
 * is not defined with any members.
 */
struct CO_Maintenance_rx_t {
    // Add members for received maintenance data as needed
};


/**
 * @brief Structure representing the static memory for maintenance data.
 *
 * This structure contains the transmitted and received maintenance data, the last
 * sequence number, sequence error count, and function pointers for handling
 * received and transmitted data.
 */
struct CO_Static_MemoryMaintenance_t {
    struct CO_Maintenance_tx_t tx;        /**< Transmitted maintenance data */
    struct CO_Maintenance_rx_t rx;        /**< Received maintenance data */
    uint32_t last_sequency;               /**< Last sequence number */
    uint16_t sequency_error;              /**< Sequence error count */
    void(*pfuncRx)(void *data);           /**< Pointer to function handling received data */
    void(*pfuncTx)(void *data);           /**< Pointer to function handling transmitted data */
};

#define NUM_SEQUENCIAL_COMMS 7

/**
 * @brief Structure for UART multiplexing header.
 *
 * This structure defines the header used in synchronous UART communication to
 * identify the packet and provide addressing and sequencing information.
 *
 * @param identity Default value is 0x7F; see @ref SYNC_UART_IDENTIFY.
 * @param address Destination address.
 * @param sequency_number Transmission count (e.g., 1, 2, 3, etc.).
 */
#pragma pack(push, 1)
struct SyncUART_Mux_t {
    uint8_t identity;               /**< Default value is: 0x7F; see @ref SYNC_UART_IDENTIFY */
    uint8_t address;                /**< Destination address */
    uint32_t sequency_number;       /**< Transmission count (1, 2, 3, etc.) */
};
#pragma pack(pop)

/**
 * @brief Structure for a complete UART packet.
 *
 * This structure defines a complete packet used in synchronous UART communication,
 * including a header, data array, and CRC validation.
 *
 * @param header Header frame for identification.
 * @param size Size of the data array.
 * @param data Data array containing the payload.
 * @param crc32 CRC validation for the frame.
 */
#pragma pack(push, 1)
struct SyncUART_Pack_t {
    struct SyncUART_Mux_t header;   /**< Header frame for identification */
    uint16_t length;                /**< Size of the data array */
    uint8_t data[MAX_DATA_SIZE];    /**< Data array */
    uint32_t crc32;                 /**< CRC validation for the frame */
};
#pragma pack(pop)

/**
 * @brief Buffer structure for synchronous UART.
 * 
 * @param temp_buffer Pointer to the temporary buffer that stores the received data.
 * @param size Current size of the data in the temporary buffer.
 * @param last_time Timestamp of the last data reception.
 * @param has Boolean indicator to check if there is data in the buffer.
 */
struct SyncUART_buffer_t {
    uint8_t temp_buffer[sizeof(struct SyncUART_Mux_t) + MAX_DATA_SIZE + sizeof(uint32_t) + sizeof(uint16_t)]; /**< Pointer to the temporary buffer. */
    uint8_t size;        /**< Current size of the data in the temporary buffer. */
    uint64_t last_time;  /**< Timestamp of the last data reception. */
    bool has;            /**< Boolean indicator for data in the buffer. */
};

#endif