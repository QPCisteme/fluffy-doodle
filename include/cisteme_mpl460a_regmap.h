/* COMMANDS ==================================================================*/
#define PL460_WR 0x0000
#define PL460_MULT_WR 0x0001
#define PL460_MULT_RD 0x0002
#define PL460_RD 0x0003
#define PL460_DEC_PKT_N 0x0004
#define PL460_DEC_INIT_VEC 0x0005
#define PL460_DEC_SIGN 0x0006
#define PL460_BUF_WR_FUSES 0x0007
#define PL460_BUF_WR_KEY_ENC 0x0008
#define PL460_BUF_WR_KEY_TAG 0x0009
#define PL460_BUF_WR_CTRL 0x000B
#define PL460_BLOW_FUSES 0x000C
#define PL460_TAM_WR_KEY_ENC 0x000D
#define PL460_TAM_WR_KEY_TAG 0x000E
#define PL460_TAM_WR_CTRL 0x0010
#define PL460_RD_TAM 0x0011
#define PL460_RD_BOOT_STATUS 0x0012
#define PL460_DECRYPT 0x0013
#define PL460_BOOT_HOST 0x0014
#define PL460_DECRYPT_PLUS 0x0015
#define PL460_MISO_M7 0xA55A
#define PL460_MISO_M7_NCLK 0xA66A
#define PL460_BOOT_UNLOCK 0xDE05

/* CONSTANTS =================================================================*/

#define PL460_BOOT_HEADER 0x5634
#define PL460_FW_HEADER 0x1122
#define PL460_BOOT_PASS_0 0x5345ACBA
#define PL460_BOOT_PASS_1 0xACBA5345

#define PL460_MISCR 0x400E1800

/* G3 Memory =================================================================*/
#define PL460_G3_STATUS 0x0000
#define PL460_G3_TX_PARAM 0x0001
#define PL460_G3_TX_DATA 0x0002
#define PL460_G3_TX_CONFIRM 0x0003
#define PL460_G3_RX_PARAM 0x0004
#define PL460_G3_RX_DATA 0x0005
#define PL460_G3_REG_INFO 0x0006

#define PL460_G3_BASE_PIB 0x80000000

/* Flags =====================================================================*/
#define PL460_TX_CFM_FLAG 0x0001
#define PL460_RX_DATA_FLAG 0x0002
#define PL460_REG_DATA_FLAG 0x0008
#define PL460_RX_PARAM_FLAG 0x0010

/* TX_PARAMETERS =============================================================*/

typedef struct __attribute__((packed))
{
    // Time_ref to trigger TX
    uint32_t timeIni;
    // Transmission length (CARE + 2)
    uint16_t dataLength;
    // Carrier preemphasis (for uniform gain)
    uint8_t preemphasis[24];
    // Available tones
    uint8_t toneMap[3];
    // Can be FORCED, RELATIVE, CANCEL, ...
    uint8_t mode;
    // LSB = -3dB
    uint8_t attenuation;
    // Can be BPSK, QPSK, ...
    uint8_t modType;
    // Can be differential, coherent
    uint8_t modScheme;
    // IDK
    uint8_t pdc;
    // Only for FCC
    uint8_t rs2Blocks;
    // Can be NO_RES
    uint8_t delimiterType;
} CENA_TX_PARAM;

typedef enum
{
    PL460_BAND_CENA = 0x00,
    PL460_BAND_CENB = 0x01,
    PL460_BAND_ARIB = 0x02,
    PL460_BAND_FCC = 0x04

} PL460_BAND;

typedef enum
{
    PL460_TX_MODE_ABSOLUTE = 0x00,
    PL460_TX_MODE_FORCED = 0x01,
    PL460_TX_MODE_RELATIVE = 0x02,
    PL460_TX_MODE_SYNCP = 0x04,
    PL460_TX_MODE_SYM_CONT = 0x08,
    PL460_TX_MODE_CANCEL = 0x10
} PL460_TX_MODE;

typedef enum
{
    MOD_TYPE_BPSK = 0,
    MOD_TYPE_QPSK = 1,
    MOD_TYPE_8PSK = 2,
    MOD_TYPE_BPSK_ROBO = 4,
} PL460_MOD_TYPE;

typedef enum
{
    MOD_SCHEME_DIFFERENTIAL = 0,
    MOD_SCHEME_COHERENT = 1,
} PL460_MOD_SCHEME;

typedef enum
{
    DT_SOF_NO_RESP = 0,
    DT_SOF_RESP = 1,
    DT_ACK = 2,
    DT_NACK = 3,
} PL460_DEL_TYPE;

/* GET_EVENT structure =======================================================*/
typedef struct __attribute__((packed))
{
    uint16_t flag;
    uint32_t tref;
    uint32_t info;
} PL460_EVENT_DATA;

/* ERROR CODES ===============================================================*/

#define PL460_BAD_HEADER 0xFFFE
#define PL460_TIMEOUT 0xFFFD
#define PL460_UNEXPECTED_EVENT 0xFFFB