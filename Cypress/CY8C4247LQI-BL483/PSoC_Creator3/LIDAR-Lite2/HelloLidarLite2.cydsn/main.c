/* ========================================
 *
 * CY8C4247LOI-B483 project to 
 *
 * MIT LICENSED SOFTWARE.
 *
 * @Author
 *   jrgdre: J.Drechsler
 *
 * @Version
 *   2016-05-16 jrgdre, initial release
 *
 * ========================================
*/
#include <project.h>
#include <stdio.h>

#define LIDAR_I2C_ADR       0x62 // LIDAR's I2C address
#define LIDAR_I2C_MAX_RETRY 250  // max number of times we try to contact the LIDAR via I2C

#define LIDAR_AUTOINCREMENT_REG (1 << 7) // if MSB of a register address is set to one, LIDAR auto increments the reg address for the next read message

// LIDAR I2C control registers
#define LIDAR_REG_COMMAND_CONTROL          0x00
#define LIDAR_REG_MODE_CONTROL             0x04
#define LIDAR_REG_OUTER_LOOP_COUNT         0x11
#define LIDAR_REG_MEASUREMENT_DELAY        0x45

// LIDAR I2C operation registers
#define LIDAR_REG_DISTANCE                 0x0F
#define LIDAR_REG_HARDWARE_VERSION         0x41
#define LIDAR_REG_SOFTWARE_VERSION         0x4F
#define LIDAR_REG_VELOCITY                 0x09

// LIDAR commands
#define LIDAR_COMMAND_REBOOT               0x00 // command control: reset LIDAR-Lite2 FPGA
#define LIDAR_COMMAND_START_ACQUSITION     0x04 // command control: start acquisition & correction processing with DC correction

// LIDAR acquisition mode flags (can be ORed together)
#define LIDAR_MC_VELOCITY                         (1 << 7) // bit to set in mode control register (0x04) to activate velocity measurement
#define LIDAR_MC_INHIBIT_REFERENCE                (1 << 6)
#define LIDAR_MC_VELOCITY_SCALE_MPS               (1 << 5) // set the velocity scale to m/s (default is dm/s)
#define LIDAR_MC_DISABLE_REFERENCE_FILTER         (1 << 4)
#define LIDAR_MC_DISABLE_SHORT_SIGNAL_ACQUISITION (1 << 3) // runs to the correlation limit of 250
#define LIDAR_MC_DISABLE_SHORT_REFERENCE          (1 << 2) // allows reference maximum count from reference acquisition count register rather than default value of 0x05
#define LIDAR_MC_CLK_SHUT                         (1 << 1)
#define LIDAR_MC_PREAMP_OFF                       (1 << 0)

// LIDAR I2C control messages
uint8_t LIDAR_I2C_Msg_ResetFPGA[]          = {LIDAR_REG_COMMAND_CONTROL , LIDAR_COMMAND_REBOOT}; 
uint8_t LIDAR_I2C_Msg_ModeControl[]        = {LIDAR_REG_MODE_CONTROL    , LIDAR_MC_VELOCITY | LIDAR_MC_DISABLE_SHORT_SIGNAL_ACQUISITION | LIDAR_MC_DISABLE_SHORT_REFERENCE};
uint8_t LIDAR_I2C_Msg_OuterLoopCount[]     = {LIDAR_REG_OUTER_LOOP_COUNT, 0xFF}; // outer loop count: 0xFF - continous measurement
uint8_t LIDAR_I2C_Msg_StartAcquisition[]   = {LIDAR_REG_COMMAND_CONTROL , LIDAR_COMMAND_START_ACQUSITION}; 

// LIDAR I2C operation messages
uint8_t LIDAR_I2C_Msg_GetDistance[]        = {LIDAR_REG_DISTANCE | LIDAR_AUTOINCREMENT_REG}; // read registers 0x0f and 0x10 in one call (0x0f with MSB set to 1 = 0x8F => LIDAR register auto increment)     
uint8_t LIDAR_I2C_Msg_GetHardwareVersion[] = {LIDAR_REG_HARDWARE_VERSION};     
uint8_t LIDAR_I2C_Msg_GetSoftwareVersion[] = {LIDAR_REG_SOFTWARE_VERSION};     
uint8_t LIDAR_I2C_Msg_GetVelocity[]        = {LIDAR_REG_VELOCITY}; 

/* write an I2C master API status message to the UART interface
 */
void UART_Send_I2CMasterApiStatus(uint32 i2cStatus)
{
    if( i2cStatus & I2C_I2C_MSTR_ERR_ARB_LOST   ) {UART_UartPutString("I2C_I2C_MSTR_ERR_ARB_LOST\r\n"   );} // (0x01u)  Master lost arbitration: INTR_MASTER_I2C_ARB_LOST    
    if( i2cStatus & I2C_I2C_MSTR_ERR_LB_NAK     ) {UART_UartPutString("I2C_I2C_MSTR_ERR_LB_NAK\r\n"     );} // (0x02u)  Last Byte Naked: INTR_MASTER_I2C_NACK                
    if( i2cStatus & I2C_I2C_MSTR_NOT_READY      ) {UART_UartPutString("I2C_I2C_MSTR_NOT_READY\r\n"      );} // (0x04u)  Master on the bus or Slave operation is in progress  
    if( i2cStatus & I2C_I2C_MSTR_BUS_BUSY       ) {UART_UartPutString("I2C_I2C_MSTR_BUS_BUSY\r\n"       );} // (0x08u)  Bus is busy, process not started                     
    if( i2cStatus & I2C_I2C_MSTR_ERR_ABORT_START) {UART_UartPutString("I2C_I2C_MSTR_ERR_ABORT_START\r\n");} // (0x10u)  Slave was addressed before master begin Start gen    
    if( i2cStatus & I2C_I2C_MSTR_ERR_BUS_ERR    ) {UART_UartPutString("I2C_I2C_MSTR_ERR_BUS_ERR\r\n"    );} // (0x100u) Bus error has: INTR_MASTER_I2C_BUS_ERROR             
}

/* write an I2C master status message to the UART interface
 */
void UART_Send_I2CMasterStatus(uint32 i2cStatus)
{
    if( i2cStatus & I2C_I2C_MSTAT_RD_CMPLT       ) {UART_UartPutString("I2C_MSTAT_RD_CMPLT\r\n" );} // ((uint16) 0x01u)  Read complete
    if( i2cStatus & I2C_I2C_MSTAT_WR_CMPLT       ) {UART_UartPutString("I2C_MSTAT_WR_CMPLT\r\n" );} // ((uint16) 0x02u)  Write complete             
    if( i2cStatus & I2C_I2C_MSTAT_XFER_INP       ) {UART_UartPutString("I2C_MSTAT_XFER_INP\r\n" );} // ((uint16) 0x04u)  Master transfer in progress
    if( i2cStatus & I2C_I2C_MSTAT_XFER_HALT      ) {UART_UartPutString("I2C_MSTAT_XFER_HALT\r\n");} // ((uint16) 0x08u)  Transfer is halted         

    if( i2cStatus & I2C_I2C_MSTAT_ERR_SHORT_XFER ) {UART_UartPutString("I2C_MSTAT_ERR_SHORT_XFER\r\n");} // ((uint16) 0x10u)  Master NAKed before end of packet            
    if( i2cStatus & I2C_I2C_MSTAT_ERR_ADDR_NAK   ) {UART_UartPutString("I2C_MSTAT_ERR_ADDR_NAK\r\n"  );} // ((uint16) 0x20u)  Slave did not ACK                            
    if( i2cStatus & I2C_I2C_MSTAT_ERR_ARB_LOST   ) {UART_UartPutString("I2C_MSTAT_ERR_ARB_LOST\r\n"  );} // ((uint16) 0x40u)  Master lost arbitration during communication 
    if( i2cStatus & I2C_I2C_MSTAT_ERR_ABORT_XFER ) {UART_UartPutString("I2C_MSTAT_ERR_ABORT_XFER\r\n");} // ((uint16) 0x80u)  The Slave was addressed before the Start gen 
    if( i2cStatus & I2C_I2C_MSTAT_ERR_BUS_ERROR  ) {UART_UartPutString("I2C_MSTAT_ERR_BUS_ERROR\r\n" );} // ((uint16) 0x100u) The misplaced Start or Stop was occurred     
}

/* send a message via I2C to the LIDAR
 *
 * also implements some simple I2C bus write error reporting
 */
void LIDAR_I2C_SendMsg(uint8 * msg, uint32 cnt) 
{
    uint32_t i2cMasterApiStatus = I2C_I2C_MSTR_NO_ERROR;
    uint8_t  tried = 0;
    
    // send the message via I2C to the LIDAR
    i2cMasterApiStatus = I2C_I2CMasterWriteBuf(LIDAR_I2C_ADR, msg, cnt, I2C_I2C_MODE_COMPLETE_XFER);
    while( !(I2C_I2CMasterStatus() & I2C_I2C_MSTAT_WR_CMPLT) )
    {
    }
    while( i2cMasterApiStatus != I2C_I2C_MSTR_NO_ERROR )
    {
        tried++;
        if( tried >= LIDAR_I2C_MAX_RETRY )
        {
            break;
        }
        i2cMasterApiStatus = I2C_I2CMasterWriteBuf(LIDAR_I2C_ADR, msg, cnt, I2C_I2C_MODE_COMPLETE_XFER);
        while( !(I2C_I2CMasterStatus() & I2C_I2C_MSTAT_WR_CMPLT) )
        {
        }
    }
    UART_Send_I2CMasterApiStatus(i2cMasterApiStatus); // report errors, if any
}

/* send a message via I2C to the LIDAR
 *
 * also implements some simple I2C bus read error reporting
 */
void LIDAR_I2C_ReadBuf(uint8 * buf, uint32 cnt) 
{
    uint32_t i2cMasterApiStatus = I2C_I2C_MSTR_NO_ERROR;
    uint8_t  tried = 0;
    
    // read bytes via I2C from the LIDAR
    i2cMasterApiStatus = I2C_I2CMasterReadBuf(LIDAR_I2C_ADR, buf, cnt, I2C_I2C_MODE_REPEAT_START);
    while( !(I2C_I2CMasterStatus() & I2C_I2C_MSTAT_RD_CMPLT) )
    {
    }
    while( i2cMasterApiStatus != I2C_I2C_MSTR_NO_ERROR )
    {
        tried++;
        if( tried >= LIDAR_I2C_MAX_RETRY )
        {
            break;
        }
        i2cMasterApiStatus = I2C_I2CMasterReadBuf(LIDAR_I2C_ADR, buf, cnt, I2C_I2C_MODE_REPEAT_START); // retry
        while( !(I2C_I2CMasterStatus() & I2C_I2C_MSTAT_RD_CMPLT) )
        {
        }
    }
    UART_Send_I2CMasterApiStatus(i2cMasterApiStatus); // report errors, if any
}

/* initialize the LIDAR-Lite2 module
 */
void LIDAR_Init()
{
    LIDAR_I2C_SendMsg(&LIDAR_I2C_Msg_ResetFPGA[0]  , sizeof(LIDAR_I2C_Msg_ResetFPGA  )); // reset the LIDAR-Lite2 FPGA
    
    CyDelay(15); // wait a couple of milliseconds for the LIDAR-Lite2 to reboot
    
    LIDAR_I2C_SendMsg(&LIDAR_I2C_Msg_ModeControl[0], sizeof(LIDAR_I2C_Msg_ModeControl)); // set LIDAR mode
}

/* read LIDAR's hardware version
 */
uint8_t LIDAR_HardwareVersion() 
{
    // prepare the LIDAR that we want to read the hardware version next
    LIDAR_I2C_SendMsg(&LIDAR_I2C_Msg_GetHardwareVersion[0], sizeof(LIDAR_I2C_Msg_GetHardwareVersion));
    
    // read the one byte hardware version
    uint8_t result = 0x00;
    LIDAR_I2C_ReadBuf(&result, 1);
    
    return result;
}

/* read LIDAR's software version
 */
uint8_t LIDAR_SoftwareVersion() 
{
    // prepare the LIDAR that we want to read the software version next
    LIDAR_I2C_SendMsg(&LIDAR_I2C_Msg_GetSoftwareVersion[0], sizeof(LIDAR_I2C_Msg_GetSoftwareVersion));

    // read the one byte software version

    uint8_t result = 0x00;
    LIDAR_I2C_ReadBuf(&result, 1);

    return result;
}

/* start LIDAR's data acquisition
 */
void LIDAR_StartMeasuringContinously()
{
    LIDAR_I2C_SendMsg(&LIDAR_I2C_Msg_OuterLoopCount[0]  , sizeof(LIDAR_I2C_Msg_OuterLoopCount  )); // set outer loop count
    LIDAR_I2C_SendMsg(&LIDAR_I2C_Msg_StartAcquisition[0], sizeof(LIDAR_I2C_Msg_StartAcquisition));
}

/* read the distance mesured (cm)
 */
uint16_t LIDAR_Distance()
{
    // prepare the LIDAR that we want to read the hardware version next
    LIDAR_I2C_SendMsg(&LIDAR_I2C_Msg_GetDistance[0], sizeof(LIDAR_I2C_Msg_GetDistance));
    
    // read the one byte hardware version
    uint8_t result[2] = {0x00, 0x00};
    LIDAR_I2C_ReadBuf(&result[0], 2);
    
    return result[0]<<8 | result[1];
}

/* read the velocity mesured (m/s)
 */
int8_t LIDAR_Velocity()
{
    // prepare the LIDAR that we want to read the hardware version next
    LIDAR_I2C_SendMsg(&LIDAR_I2C_Msg_GetVelocity[0], sizeof(LIDAR_I2C_Msg_GetVelocity));
    
    // read the one byte hardware version
    int8_t result = 0x00;
    LIDAR_I2C_ReadBuf((uint8 *)&result, 1);
    
    return result;
}

#define AVERAGE_EXPONENT 0 // 2^x = number of repeated measurements averaged for one measurement point 
                           // calculates like: x=3 => 2^3 = 8 measurements average
                           // higher numbers mean more stable results, but less dynamic response

/* read the measurement values from the LIDAR
 *
 * the function returns averaged values for distance and velocity
 */
void LIDAR_Measure(uint16_t *distance, int8_t *velocity)
{
    uint8_t  i;
    uint32_t distanceSum = 0u;
    int32_t  velocitySum = 0;
    
    *distance = 0u;
    *velocity = 0;
    
    for( i = 0u; i < (1 << AVERAGE_EXPONENT); i++ )
    {
        distanceSum += LIDAR_Distance(); // at 100kHZ I2C bus speed
        velocitySum += LIDAR_Velocity(); // it takes ~1ms to get the two values
        CyDelay(99);                     // the LIDAR has a new set of values each 100ms 
                                         // (LIDAR_MC_VELOCITY_SCALE_MPS=0 => always 100ms measurement delay)
    }
    
    *distance = distanceSum >> AVERAGE_EXPONENT;
    *velocity = velocitySum >> AVERAGE_EXPONENT;
}

int main()
{
    CyGlobalIntEnable; // Enable global interrupts.

    // initialization/startup code
    UART_Start();
    I2C_Start();

    LIDAR_Init();

    // let's write a nice welcome message to the terminal
    char str[256];
    sprintf(str, "\r\nLIDAR-Lite2 (hw: %d, sw: %d)\r\n", LIDAR_HardwareVersion(), LIDAR_SoftwareVersion());
    UART_UartPutString(str);

    // kick of the continious gathering of measurement data by the LIDAR
    LIDAR_StartMeasuringContinously();

    uint16_t distance = 0u;
    int8_t   velocity = 0;
    for(;;)
    {
        LIDAR_Measure(&distance, &velocity);
        sprintf(str, "%dcm %ddm/s    \r", distance, velocity); // create a string from the measurement values 
        UART_UartPutString(str);                               // and send it over the UART
    }
}

/* [] END OF FILE */
