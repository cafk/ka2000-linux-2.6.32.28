#ifndef _I2C_INTERFACE_H_
#define _I2C_INTERFACE_H_


typedef unsigned char uint8;
typedef unsigned char INT8U;
typedef unsigned char BOOLEAN;
typedef char INT8S;


#define KA_IIC_BASE 0xA000D000
#define IIC_CR_ADDR (KA_IIC_BASE  + 0x0)
#define IIC_SR_ADDR (KA_IIC_BASE  + 0x4)
#define IIC_DR_ADDR (KA_IIC_BASE  + 0x8)
#define IIC_AR_ADDR (KA_IIC_BASE  + 0xC)
#define IIC_TR_ADDR (KA_IIC_BASE  + 0x10)



/*==================================================================================================
                                 STRUCTURES AND OTHER TYPEDEFS
==================================================================================================*/

// structure that holds the parameters for I2C communiation
typedef struct
{
  INT8U slave_addr;
  INT8U length;
  INT8U slave_reg_addr;
  INT8U *data_ptr;
  		
}i2c_rw_cmd_type;

/*==================================================================================================
                                 GLOBAL VARIABLE DECLARATIONS
==================================================================================================*/

/*==================================================================================================
                                     FUNCTION PROTOTYPES
==================================================================================================*/




#endif //_I2C_INTERFACE_H_
