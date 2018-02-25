#ifndef KA2000_SDIO_H
#define KA2000_SDIO_H


//------STATES
#define CMD_TIME_ERR	(1<<16)
#define CMD_CRC_ERR		(1<<17)
#define CMD_END_ERR		(1<<18)
#define CMD_IND_ERR		(1<<19)

#define DATA_CRC_ERR	(1<<21)
#define DATA_END_ERR	(1<<22)


//------TRAN

#define CMD_DONE		(1<<2)
#define DATA_DONE		(1<<1)

//------SDIO_DMA_INTS_REG

#define CH0_INT			(1<<0)
#define CH1_INT			(1<<1)


struct ka_sdio_platdata {	
	unsigned int	max_width;	
	unsigned int	host_caps;	
	char			**clocks;	
	int				irq_num;
};


struct ka_sdio_host {
	/* Internal data */
	struct platform_device			*pdev;
	struct resource					*ioarea;
	struct ka_sdio_platdata      	*pdata;
	struct clk						*clk_io;
	
	spinlock_t		lock;		/* Mutex */
	
	struct mmc_host		*mmc;		/* MMC structure */
	struct mmc_request	*mrq;		/* Current request */
	
	
	unsigned int		cur_clk;
	unsigned int		max_clk;	/* Max possible freq (MHz) */
	unsigned int		timeout_clk;	/* Timeout freq (KHz) */
#define SDHCI_USE_SDMA          (1<<0)          /* Host is SDMA capable */
#define SDHCI_USE_ADMA          (1<<1)          /* Host is ADMA capable */
#define SDHCI_REQ_USE_DMA       (1<<2)          /* Use DMA for this req. */
#define SDHCI_DEVICE_DEAD       (1<<3)          /* Device unresponsive */	
	unsigned int 		flags;
	unsigned int		clock;		/* Current clock (MHz) */
	unsigned int 		power;		/* Current voltage */
	unsigned int 	bus_width_4_flag;
	unsigned int 	sdio_card_int_flag;
	int seq;  
	int w_seq;
	int r_seq;
	int error;
};

struct ka_sdio_request {
	struct mmc_host		*mmc;		/* MMC structure */
	struct mmc_request	*mrq;		/* Current request */
	struct mmc_command	*cmd;		/* Current command */
	struct mmc_data		*data;		/* Current data request */	
};




#endif
